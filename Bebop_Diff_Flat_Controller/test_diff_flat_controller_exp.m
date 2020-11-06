clear; close all; clc;
%sample time / total time
Ts = 0.02;
T_total = 0:Ts:20;
T_steps = length(T_total);

%position gains (x, y, z)
K_p =5*eye(3, 3);
% K_p(2, 2) = 20;
% K_p(3, 3) = 40;

% velocity gains (v_x, v_y, v_z)
K_v = 3.7*eye(3, 3); % 20
% K_v(2, 2) = 3;
% K_v(3, 3) = 10;

% orientation gains (phi, theta, psi)
K_r  = 2*eye(3, 3); % 1
K_r(2, 2) = 2;
K_r(3, 3) = 0.00035;

% angular velocity gains (w_x, w_y, w_z)
K_w  = .3*eye(3, 3); % 2
K_w(2, 2) = .3;
K_w(3, 3) = 0.00015;

%initialize bebop model
initialState=[0 0 0 0 0 0 0 0 0 0 0 0]';
bebop=Bebop2(initialState);
diff_flat_ctr = Controller_Diff_Flat(bebop.m, bebop.g, bebop.I, K_p, K_v, K_r, K_w, Ts);
omegas = zeros(4, T_steps);
% 
global pos 
pos = zeros(T_steps, 3);
global orient %euler angles ZYX
orient = zeros(T_steps, 3);
global commands
commands = zeros(T_steps, 4);

% trajectory
p_t = trajectory_generator(T_total(end), 'circle', Ts);


%% Create chain of integrators system for trajectory following
X = zeros(4, 4);
A = [0, 1, 0, 0; 0, 0, 1, 0; 0, 0, 0, 1; 0, 0, 0, 0];
B = [0; 0; 0; 1];
p = [-15; -15.2; -15.4; -15.8];
K = place(A, B, p);
nominal_force_vect = [0, 0, bebop.m*bebop.g]';

X_state = zeros(1, 16);
init_pos  = [0, 0, 1];
X(1, 1:3) = init_pos;


%% set ROS params (publishers, subscribers, node starts)

%The communication with the ROS server is closed if active
rosshutdown

%A new connection is estabilished with the ROS master
IP_ROS_Master = 'http://127.0.1.1:11311/';
rosinit(IP_ROS_Master)

%set node name
node1 = ros.Node('/traj_follow');



%publish and subscribe topics for nodes
% pub_diff_flat = ros.Publisher(node1, '/cmd_diff', "geometry_msgs/Twist");
bebop_name = "bebop1";
bebop_cmd_diff = '/cmd_diff';
bebop_goal_msg = '/goal';
bebop_speeds = '/motor_speeds';
s_diff = strcat('/', bebop_name, bebop_cmd_diff);
s_goal = strcat('/', bebop_name, bebop_goal_msg);
s_rpms = strcat('/', bebop_name, bebop_speeds);

    
pub_cmd_diff = ros.Publisher(node1, s_diff, "geometry_msgs/Twist");
pub_cmd_goal = ros.Publisher(node1, s_goal, "geometry_msgs/PoseStamped");
sub_mocap = ros.Subscriber(node1,"/vrpn_client_node/bebop1/pose", "geometry_msgs/PoseStamped");
sub_rpms = ros.Subscriber(node1, s_rpms, "bebop_experiments/mtrspeeds");



bebop_pose = '/pose';
cmd_goal_msg = rosmessage("geometry_msgs/PoseStamped");


% s = strcat('/',bebop_name, bebop_pose);
% sub_odom = ros.Subscriber(node1, s); % contains the drone pose (position, orientation)

% node_1 = ros.Node(script);

%% Start Experiment
for i=1:T_steps
    fprintf("time: %1.4f \n", T_total(i));
    
    if i < length(p_t)
        des_point = p_t(i, :)';
    else
        des_point = p_t(end, :)';
    end
    
    
    des_mat = zeros(4, 4);
    des_mat(1, :) = des_point(1:4);
    des_mat(2, :) = des_point(5:8);
    des_mat(3, :) = des_point(9:12);
    des_mat(4, :) = des_point(13:16);
           
    u = -K*(X-des_mat);
    X_DOT = A*X + B*u;
    X = X + X_DOT*Ts;    
    X_state = reshape(X', 1, 16)'
    
    [roll, pitch, yaw_rate, thrust] = diff_flat_ctr.diff_flat_term(X_state)
    
    %pose update
    pose = sub_mocap.LatestMessage;
    if ~isempty(pose)
        point=zeros(1, 3);
        point(1)=pose.Pose.Position.X;
        point(2)=pose.Pose.Position.Y;
        point(3)=pose.Pose.Position.Z;
        pos(i, :) = point;
        point

        quat=zeros(4,1);
        quat(1)=pose.Pose.Orientation.W;
        quat(2)=pose.Pose.Orientation.X;
        quat(3)=pose.Pose.Orientation.Y;
        quat(4)=pose.Pose.Orientation.Z;
        orient(i, :) = quat2eul( quat', 'ZYX');
        quat2eul( quat', 'ZYX')
    end
    
    motor_msg = sub_rpms.LatestMessage;
    if ~isempty(motor_msg)
        motor_ang_vels = motor_speeds_callback(sub_rpms);
        command = inv(bebop.allocMat)\(motor_ang_vels').^2;
        commands(i, :) = motor_ang_vels;
    end
    
       
    
    %publish goal pose
    quat = eul2quat([0, 0, 0], 'ZYX');
    cmd_goal_msg.Header.Stamp = rostime("now");
    cmd_goal_msg.Header.Seq = cmd_goal_msg.Header.Seq + 1;
    cmd_goal_msg.Pose.Position.X = 0; %X_state(1, 1);
    cmd_goal_msg.Pose.Position.Y = 0; % X_state(2, 1);
    cmd_goal_msg.Pose.Position.Z = 1; % X_state(3, 1);
    cmd_goal_msg.Pose.Orientation.X = quat(2);
    cmd_goal_msg.Pose.Orientation.Y = quat(3);
    cmd_goal_msg.Pose.Orientation.Z = quat(4);
    cmd_goal_msg.Pose.Orientation.W = quat(1);
    send(pub_cmd_goal, cmd_goal_msg);
    
    
    % publish feedforward term
    sent_thrust = (thrust - nominal_force_vect(3))
    cmd_vel_msg = rosmessage("geometry_msgs/Twist");
    cmd_vel_msg.Linear.X  = 0; %roll;
    cmd_vel_msg.Linear.Y = 0; %pitch;
    cmd_vel_msg.Angular.Z = 0.0; %yaw_rate;
    cmd_vel_msg.Linear.Z = 0;
    send(pub_cmd_diff, cmd_vel_msg);
    pause(0.02);
end
    %publish goal pose
    quat = eul2quat([0, 0, 0], 'ZYX');
    cmd_goal_msg.Header.Stamp = rostime("now");
    cmd_goal_msg.Header.Seq = cmd_goal_msg.Header.Seq + 1;
    cmd_goal_msg.Pose.Position.X = 0;
    cmd_goal_msg.Pose.Position.Y = 0;
    cmd_goal_msg.Pose.Position.Z = 0;
    cmd_goal_msg.Pose.Orientation.X = quat(2);
    cmd_goal_msg.Pose.Orientation.Y = quat(3);
    cmd_goal_msg.Pose.Orientation.Z = quat(4);
    cmd_goal_msg.Pose.Orientation.W = quat(1);
    send(pub_cmd_goal, cmd_goal_msg);
     
    
    % publish feedforward term
    cmd_vel_msg = rosmessage("geometry_msgs/Twist");
    cmd_vel_msg.Linear.X = 0;
    cmd_vel_msg.Linear.Y = 0;
    cmd_vel_msg.Angular.Z = 0;
    cmd_vel_msg.Linear.Z = 0;
    send(pub_cmd_diff, cmd_vel_msg);
    pause(0.02);
rosshutdown
pause(3.0);
%% plot errors
% x-y plane
xTraj = p_t(:,1)';
yTraj = p_t(:,2)';

pos = pos';
xBebop=pos(:,1);
yBebop=pos(:,2);

figure()
subplot(1,2,1)
plot(xTraj,yTraj,'b'); hold on; grid on; axis equal;
plot(xBebop,yBebop,'r'); 
legend('desired trajectory','real path');
title('comparison between trajectory and real path')
subplot(1,2,2)
t_len = min([T_steps, length(xTraj), length(xBebop)]);
plot(1:t_len,xTraj(1:t_len)-xBebop(1:t_len)');hold on; grid on;
plot(1:t_len,yTraj(1:t_len)-yBebop(1:t_len)');
legend('x axis','y axis');
title('Errors: traj-path');


% height
zBebop = pos(:,3);
figure()
plot(1:length(zBebop),zBebop); grid on;
title('height');

% yaw angle
yawBebop = orient(:,1);
figure()
plot(1:length(yawBebop),rad2deg(yawBebop)); grid on;
title('yaw angle');
ylabel('deg');

%motor velocities
figure()
plot(1:t_len, commands(1:t_len, :));
legend('omega 1','omega 2','omega 3','omega 4');
ylabel('rad/s'); xlabel('s');
title('bebop2 inputs');

%% aux functions
function [pos, orient] = mocap_callback(msg, ~)

    pose = msg.LatestMessage.Pose;
    point=zeros(1, 3);
    point(1)=pose.Position.X;
    point(2)=pose.Position.Y;
    point(3)=pose.Position.Z;
    pos = point;
    
    quat=zeros(4,1);
    quat(1)=pose.Orientation.W;
    quat(2)=pose.Orientation.X;
    quat(3)=pose.Orientation.Y;
    quat(4)=pose.Orientation.Z;
    orient = quat2eul( quat', 'ZYX');
end
    

