clear; close all;

% control system sampling and simulation time
Ts=0.02;
times = 0:Ts:20;
experimentSteps=length(times);

%bebop model
initialState=[0 0 1 0 0 0 0 0 0 0 0 0]';
bebop=Bebop2(initialState);
ueq=bebop.omegaHover*ones(4,1);
forcesEq = [bebop.m*bebop.g 0 0 0]';
seq=[0 0 1 0 0 0 0 0 0 0 0 0]';
%seq=initialState;

% -?- I think that we are not using these values for the controllers,
% right? 
% 2 CONTROLLERS
% controller for taking off and landing
xref=0; yref=0; zref=1; yawref=0;

%controller for trajectory tracking
%position gains (x, y, z)
K_p =5*eye(3, 3);

% velocity gains (v_x, v_y, v_z)
K_v = 3.7*eye(3, 3); 

% orientation gains (phi, theta, psi)
K_r  = 2*eye(3, 3); 
K_r(2, 2) = 2;
K_r(3, 3) = 0.00035;

% angular velocity gains (w_x, w_y, w_z)
K_w  = .3*eye(3, 3); 
K_w(2, 2) = .3;
K_w(3, 3) = 0.00015;

% differential flatness controller
diff_flat_ctr = Controller_Diff_Flat(bebop.m, bebop.g, bebop.I, K_p, K_v, K_r, K_w, Ts);

K_d = 0.2; %-?- This is used

% state and forces filter
initialStateGuess = zeros(12,1);
initialStateGuess(1:3)=initialState(1:3)';

stateForcesFilter=unscentedKalmanFilter(@stateForcesFilterTransitionNOvel,...
    @stateFilterMeas,[initialStateGuess;zeros(4,1)]);
stateForcesFilter.StateCovariance=deg2rad(1); %the software uses the scalar value to create an Nx-by-Nx diagonal matrix.
stateForcesFilter.ProcessNoise= 1e-4*eye(16); %1e-2eye(16)
stateForcesFilter.ProcessNoise(9,9)=1e-2; %yaw
sigmaPosition = 0.001; %0.001
sigmaAttitude = deg2rad(1); %0.001
R = diag( [ones(3,1)*sigmaPosition^2 ; ones(3,1)*sigmaAttitude^2; ] );
stateForcesFilter.MeasurementNoise=R;

% the filter story
yMeas = zeros(6,experimentSteps);
stateForcesResidBuf = zeros(6,experimentSteps);
stateForcesXcorBuf = zeros(16,experimentSteps); %corrected
stateForcesXpredBuf = zeros(16,experimentSteps); %predicted

% admittance controller
virtualMasses = 1e-1*[20 20 20 1]';
virtualDampings = 1e-1*[5 5 5 0.5]';
adCtr=[];

% store position derivatives
pddd_all =zeros(4, experimentSteps);
pdd_all = zeros(4, experimentSteps);
pd_all = zeros(4, experimentSteps);
p_all = zeros(4, experimentSteps);

angles_n_thrust = zeros(4, experimentSteps);
admittance_ctrl_vals = zeros(4, experimentSteps);
commands = zeros(4, 1);

global commands_vect
commands_vect = zeros(experimentSteps, 4);

takingOff=true;
tol=5e-2;
yawTol=deg2rad(10);
%force=[0 0 -0.05]';
forceTime=[2 15]; %
% tunable force and torque tresholds
forceTreshold= 0.5;
torqueTreshold=0.1;


%% set ROS params (publishers, subscribers, node starts)

%The communication with the ROS server is closed if active
rosshutdown

%A new connection is estabilished with the ROS master
IP_ROS_Master = 'http://127.0.1.1:11311/';
rosinit(IP_ROS_Master)

%set node name
node1 = ros.Node('/admittance_follow');


%publish and subscribe topics for nodes
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

% hover condition to be reached at the end of the taking off phase
p = [xref, yref, zref, yawref];
roll = 0;
pitch = 0;
yawrate = 0;

filterCycles=4;
Ts_motor=0.1;
lastData = 1;

%% Start Experiment
for i=1:experimentSteps
    fprintf('time: %1.4f \n',times(i));
    
    %read pose measurement
    pose = sub_mocap.LatestMessage;
    if ~isempty(pose)
        [point, orientation] = mocap_callback(sub_mocap);
        curState = [point(1); point(2); point(3); ...
            0; 0; 0; ...
            orientation(1); orientation(2); orientation(3); ...
            0; 0; 0];
    end
    yMeas(:,i) = curState([1 2 3 8 9 7]); %x, y, z, phi (roll), theta (pitch), psi (yaw)
    
    %read rotors speed command
    motor_msg = sub_rpms.LatestMessage;
    
    % compute [ft mx my mz]' from rotors speed
    if ~isempty(motor_msg)
        if (motor_msg.M1 + motor_msg.M2 + motor_msg.M3 + motor_msg.M4) > 0 
            motor_ang_vels = deg2rad(6.*motor_speeds_callback(sub_rpms)); 
            commands = bebop.allocMat*(motor_ang_vels').^2; 
            commands_vect(i, :) = motor_ang_vels;
        end
    end
    % check if still in the taking off phase
    if takingOff
        near = norm(yMeas(1,i)-xref)<tol &&...
            norm(yMeas(2,i)-yref)<tol &&...
            norm(yMeas(3,i)-zref)<tol &&...
            norm(yMeas(6,i)-yawref)<yawTol;
        if near || times(i) > 5
            takingOff=false;
            
            % switch on the admittance controller
            state = [curState(1:3)', curState(7)]; %position, yaw
            adCtr = AdmittanceCtrlComplete( [state; zeros(1,4)],...
                virtualMasses, virtualDampings, Ts);
            switchStep=i;
            fprintf('tracking starting from step %d\n',switchStep);
            %             X(1, 1:3) = [point(1), point(2), 1]
        end
    end
    
    filterCycles=filterCycles+1;
    if filterCycles==5
        lastData=i;       
        % state estimation with the new measurement   
        %[Residual,~] = residual(stateForcesFilter,yMeas(:,i));
        %stateForcesResidBuf(:,i) = Residual;
        [correctedState,~] = correct(stateForcesFilter,yMeas(:,i));
        stateForcesXcorBuf(:,i) = correctedState;
    end
    
    if ~takingOff
        
        % check if the estimated force and torque are bigger than related
        % tresholds
        force_norm =norm(stateForcesXcorBuf(13:15,lastData))
        
        estForceOK = norm(stateForcesXcorBuf(13:15,lastData))>= forceTreshold && norm(stateForcesXcorBuf(13:15,lastData)) < 2;
        estTorqueOK = norm(stateForcesXcorBuf(16,lastData))>=torqueTreshold;
        
        if estForceOK && estTorqueOK
            admittance_ctrl_vals(:, i) = stateForcesXcorBuf(13:16, lastData);
            adCtr.update(stateForcesXcorBuf(13:16,lastData));
            fprintf('force and torque above tresholds   ')
        else
            if estForceOK
                admittance_ctrl_vals(:, i) = [stateForcesXcorBuf(13:15, lastData);0];
                adCtr.update( [stateForcesXcorBuf(13:15,lastData);0] );
                fprintf('force above treshold   ')
            else
                if estTorqueOK
                    admittance_ctrl_vals(:, i) = [zeros(3,1); stateForcesXcorBuf(16,lastData)];
                    adCtr.update( [zeros(3,1);stateForcesXcorBuf(16,lastData)] );
                    fprintf('torque above treshold   ')
                else
                    admittance_ctrl_vals(:, i) = zeros(4,1);
                    adCtr.update( zeros(4,1) );
                    fprintf('send zero')
                end
            end
        end
        
        p = adCtr.getRef();
        p_all(:, i) = p;
        pd = adCtr.getRefd();
        pd_all(:, i) = pd;
        pdd = adCtr.getRefddFilter();
        pdd_all(:, i) = pdd;
        pddd = adCtr.getRefdddFilter();
        pddd_all(:, i) = pddd;
        des_x = [p', pd', pdd', pddd']
        [roll, pitch, yawrate] = diff_flat_ctr.diff_flat_term(des_x')
    end
    
    %if in the taking off phase still holds p = [xref, yref, zref, yawref];
    %roll = 0; pitch = 0; yawrate = 0;
      
    
    if ~takingOff
        % publish goal pose, the reference values for the PID code. 
        % maybe it is unuseful to publish this many times during the taking off phase...
        quat = eul2quat([p(4), 0, 0], 'ZYX');
        cmd_goal_msg.Header.Stamp = rostime("now");
        cmd_goal_msg.Header.Seq = cmd_goal_msg.Header.Seq + 1;
        cmd_goal_msg.Pose.Position.X =p(1);
        cmd_goal_msg.Pose.Position.Y =p(2);
        cmd_goal_msg.Pose.Position.Z =p(3);
        cmd_goal_msg.Pose.Orientation.X = quat(2);
        cmd_goal_msg.Pose.Orientation.Y = quat(3);
        cmd_goal_msg.Pose.Orientation.Z = quat(4);
        cmd_goal_msg.Pose.Orientation.W = quat(1);
        send(pub_cmd_goal, cmd_goal_msg);  
        
        % publish feedforward term used by the PID code
        des_velocity = pd(3);
        cmd_vel_msg = rosmessage("geometry_msgs/Twist");
        cmd_vel_msg.Linear.X  = K_d*roll;
        cmd_vel_msg.Linear.Y = K_d*pitch;
        cmd_vel_msg.Angular.Z = K_d*yawrate;
        cmd_vel_msg.Linear.Z = des_velocity;
        send(pub_cmd_diff, cmd_vel_msg);
        angles_n_thrust(:, i) = [K_d*roll, K_d*pitch, K_d*yawrate, des_velocity];
        %pause value DO NOT DELETE
        pause(0.02);  
    end
    
    if filterCycles==5
    filterCycles=0;
    % compute the new predicted state
    [predictedState,PredictedStateCovariance] = predict(stateForcesFilter,...
        commands,Ts_motor,bebop.m,bebop.I);
    stateForcesXpredBuf(:,i) = predictedState;
    end
    


end

% end of experiment, landing bebop
% publish goal pose
quat = eul2quat([0, 0, 0], 'ZYX');
cmd_goal_msg.Header.Stamp = rostime("now");
cmd_goal_msg.Header.Seq = cmd_goal_msg.Header.Seq + 1;
cmd_goal_msg.Pose.Position.X = p(1);
cmd_goal_msg.Pose.Position.Y = p(2);
cmd_goal_msg.Pose.Position.Z = 0.0;
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



%bebop.plotState();
%bebop.plotInputs();
adCtr.plotOutputs();

%%
load(' measured_interaction_11_yaw.mat')
load('adm_ctr_interaction_11_yaw.mat')
load('ang_thr_interaction_11_yaw.mat')
load('motor_speeds_interaction_11_yaw.mat')
load('p_all_arr_interaction_11_yaw.mat')
load('prediction_ekf_interaction_11_yaw.mat')
load('times.mat')

%%
start_i=0.02/0.02;
end_i = 15/0.02;
p_all = p_all(:, start_i:end_i);
times = times(:,start_i:end_i);
yMeas = yMeas(:, start_i:end_i);
angles_n_thrust = angles_n_thrust(:, start_i:end_i);
t_jumps = 0:0.1:20;
adCtrTime =times;% adCtr.getTimes() + Ts*(switchStep-1);
adCtrValues = p_all; %adCtr.getPositions;

%% All in One (positions, forces)
fig3=figure();
fig3.Name='Comparison between admittance controller reference and actual positions';
%title('Comparison between admittance controller reference and actual positions', 'FontSize', 30);



subplot(4,1,1);
plot(adCtrTime, adCtrValues(1,:),'--', 'LineWidth', 3);hold on; grid on;
plot(times, yMeas(1,:), 'LineWidth', 3);
stairs(t_jumps,nonzeros(stateForcesXpredBuf(13,:)), 'LineWidth', 3);
xlim([5 15])
ylim([-1 1])
plot(ones(1,10)*(5),linspace(-2,2,10),'k--', 'LineWidth', 3);
title('X', 'FontSize', 12); ylabel({'X position [m]', 'X Force [N]'}, 'FontSize', 12);
legend('ad. ctrl. reference','bebop x position','x force', 'Location','best', 'FontSize', 12);

subplot(4,1,2);
plot(adCtrTime, adCtrValues(2,:),'--', 'LineWidth', 3);hold on; grid on;
plot(times, yMeas(2,:), 'LineWidth', 3);
stairs(t_jumps,nonzeros(stateForcesXpredBuf(14,:)), 'LineWidth', 3);
xlim([5 15])
ylim([-2 2])
plot(ones(1,10)*(5),linspace(-2,2,10),'k--', 'LineWidth', 3);
title('Y', 'FontSize', 12); ylabel({'Y position [m]', 'Y Force [N]'});
legend('ad. ctrl. reference','bebop y position', 'y force','Location','best');

subplot(4,1,3);
plot(adCtrTime, adCtrValues(3,:),'--', 'LineWidth', 3);hold on; grid on;
plot(times, yMeas(3,:), 'LineWidth', 3);
stairs(t_jumps,nonzeros(stateForcesXpredBuf(15,:)), 'LineWidth', 3);
xlim([5 15])
ylim([-2 2])
plot(ones(1,10)*(5),linspace(-2,2,10),'k--', 'LineWidth', 3);
title('Z', 'FontSize', 12); ylabel({'Z position [m]', 'Z Force [N]'});
legend('ad. ctrl. reference','bebop z position','z force','Location','best');

subplot(4,1,4);
plot(adCtrTime, rad2deg(adCtrValues(4,:)),'--', 'LineWidth', 3);hold on; grid on;
plot(times, rad2deg(yMeas(4,:)), 'LineWidth', 3);
stairs(t_jumps(2:end),nonzeros(stateForcesXpredBuf(16,:))*1e2, 'LineWidth', 3);
plot(ones(1,10)*(5),linspace(-100,100,10),'k--', 'LineWidth', 3);
xlim([5 15])
ylim([-12 12])
title('Yaw', 'FontSize', 12); ylabel({'Yaw angle [deg]', 'Torque [cN m]'}); xlabel('Time [s]', 'FontSize', 12);
legend('ad. ctrl. reference','bebop yaw', 'torque','Location','best');

%% Forces

fig5=figure();
fig5.Name='Forces';
%title('Comparison between admittance controller reference and actual positions', 'FontSize', 30);

subplot(4,1,1); 
plot(adCtrTime, adCtrValues(1,:),'--', 'LineWidth', 3);hold on; grid on;
plot(times, yMeas(1,:), 'LineWidth', 3);
stairs(t_jumps,nonzeros(stateForcesXpredBuf(13,:)), 'LineWidth', 3);grid on;
xlim([5 15])
ylim([-1 1])
%plot(ones(1,10)*(5),linspace(-2,2,10),'k--', 'LineWidth', 3);
title('Force on X-Axis', 'FontSize', 12); ylabel({'X Force [N]'}, 'FontSize', 12);
%legend('ad. ctrl. reference','bebop x position','Location','best', 'FontSize', 12);

subplot(4,1,2); 
plot(adCtrTime, adCtrValues(2,:),'--', 'LineWidth', 3);hold on; grid on;
plot(times, yMeas(2,:), 'LineWidth', 3);
stairs(t_jumps,nonzeros(stateForcesXpredBuf(14,:)), 'LineWidth', 3);grid on;
xlim([5 15])
ylim([-1 1])
%plot(ones(1,10)*(5),linspace(-2,2,10),'k--', 'LineWidth', 3);
title('Force on Y-Axis', 'FontSize', 12); ylabel({'Y Force [N]'});
%legend('ad. ctrl. reference','bebop y position', 'Location','best');

subplot(4,1,3); 
plot(adCtrTime, adCtrValues(3,:),'--', 'LineWidth', 3);hold on; grid on;
plot(times, yMeas(3,:), 'LineWidth', 3);
stairs(t_jumps,nonzeros(stateForcesXpredBuf(15,:)), 'LineWidth', 3);grid on;
xlim([5 15])
ylim([0 0.5])
%plot(ones(1,10)*(5),linspace(-2,2,10),'k--', 'LineWidth', 3);
title('Force on Z-Axis', 'FontSize', 12); ylabel({'Z Force [N]'});
%legend('ad. ctrl. reference','bebop z position','Location','best');

subplot(4,1,4); 
plot(adCtrTime, rad2deg(adCtrValues(4,:)),'--', 'LineWidth', 3);hold on; grid on;
plot(times, rad2deg(yMeas(4,:)), 'LineWidth', 3);
stairs(t_jumps(2:end),nonzeros(stateForcesXpredBuf(16,:))*1e3, 'LineWidth', 3); grid on;
%plot(ones(1,10)*(5),linspace(-100,100,10),'k--', 'LineWidth', 3);
xlim([5 15])
ylim([-100 100])
title('Torque on Yaw-Axis', 'FontSize', 12); ylabel({'Z Torque [mN m]'}); xlabel('Time [s]', 'FontSize', 12);
%legend('ad. ctrl. reference','bebop yaw','Location','best');


%% Save Data
label = 'interaction_1';
ekf = 'prediction_ekf_' + string(label)+'.mat';
save(ekf, 'stateForcesXcorBuf');

measured_stuff = ' measured_' + string(label)+'.mat';
save(measured_stuff, 'yMeas');

speeds = 'motor_speeds_'+ string(label)+'.mat';
save(speeds, 'commands_vect');

p_all_arr = 'p_all_arr_' + string(label)+'.mat';
save(p_all_arr, 'p_all');

times_mat = 'times.mat';
save(times_mat, 'times');

ang_n_thr_mat = 'ang_thr_' + string(label)+'.mat';
save(ang_n_thr_mat, 'angles_n_thrust');

adr_ctr_mat = 'adm_ctr_'+string(label) +'.mat';
save(adr_ctr_mat, 'adCtrValues');


%motor velocities
figure()
plot(1:experimentSteps, commands_vect(1:experimentSteps, :));
legend('omega 1','omega 2','omega 3','omega 4');
ylabel('rad/s'); xlabel('s');
title('Inputs');
%%
figure()
plot(times,stateForcesXpredBuf(16,:)); grid on; hold on
legend('Mz')

fig(2) = figure();
switchTime=switchStep*Ts;

stairs(times,stateForcesXpredBuf(13:15,:)', 'LineWidth', 5);
hold on; grid on;
% plot(linspace(switchTime+forceTime(1),switchTime+forceTime(2),10),...
%     ones(1,10)*force(1),'-.r','LineWidth',2);
% plot(linspace(switchTime+forceTime(1),switchTime+forceTime(2),10),...
%     ones(1,10)*force(2),'--r','LineWidth',2);
% plot(linspace(switchTime+forceTime(1) ,switchTime+forceTime(2),10),...
%     ones(1,10)*force(3),'-r','LineWidth',2);


ax=axis;
plot(ones(1,10)*(switchTime),linspace(ax(3),ax(4),10),'k--', 'LineWidth', 5)
% plot(ones(1,10)*(switchTime+forceTime(2)),linspace(ax(3),ax(4),10),'k--', 'LineWidth', 5)
ylabel('N'); xlabel('s'); title('Estimated forces for bebop')
legend('x ext force','y ext force','z ext force','Location','best', 'FontSize', 25);
title('Estimated Forces', 'LineWidth', 10);

%% estimated Force (ukf test)
fig(2) = figure();
stairs(times(1:500),stateForcesXpredBuf(15,1:500)', 'LineWidth', 5);
hold on; grid on;
ax=axis;
plot(times(1:500),-ones(1, 500),'k--', 'LineWidth', 10)
ylim([-1.1 0]);
ylabel('Force (N)'); xlabel('Time(s)'); title('Estimated forces for bebop')
legend('Z-force (est)','Z-force (actual)', 'Location','best', 'FontSize',30);
title('Estimated Forces', 'FontSize', 30);
