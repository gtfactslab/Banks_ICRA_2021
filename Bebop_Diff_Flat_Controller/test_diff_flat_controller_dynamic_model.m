clear; close all


%sample time / total time
Ts = 0.02;
T_total = 0:Ts:30;
sim_len = length(T_total);

%initialize bebop model
initialState=[0 0 0 0 0 0 0 0 0 0 0 0]';
bebop=Bebop2(initialState);

% initialize hovering
u_eq=bebop.omegaHover*ones(4,1);
torques_eq = [bebop.m*bebop.g 0 0 0]';
s_eq=[0 0 1 0 0 0 0 0 0 0 0 0]';

%% Controllers

% takeOff and Landing
hov_ctr = SubSysController(bebop.m,bebop.I,Ts);
xref=0; yref=0; zref=1; psiref=0;
hov_ctr.setRef([xref yref zref]'-s_eq(1:3), psiref);
delta_omegas = zeros(4, sim_len);


% diff flat trajectory tracking
takingOff = true;

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

diff_flat_ctr = Controller_Diff_Flat(bebop.m, bebop.g, bebop.I, K_p, K_v, K_r, K_w, Ts);
omegas = zeros(4, sim_len);

% trajectory
p_t = trajectory_generator(T_total(end), 'line', Ts);

%% Create chain of integrators system for trajectory following
X = zeros(4, 4);
A = [0, 1, 0, 0; 0, 0, 1, 0; 0, 0, 0, 1; 0, 0, 0, 0];
B = [0; 0; 0; 1];
p = [-3; -3.2; -3.4; -3.8];
K = place(A, B, p);
X(1, 1:3) = [0, 0, 1];

X_state = zeros(1, 16);

x_state_record = zeros(sim_len, 16);

%% Run Simulation

for i=1:sim_len
    fprintf("time: %f \n", T_total(i))
    state = bebop.s(:, end);
    
    
    if takingOff == true
        if norm(state(1:3) - s_eq(1:3)) > 0.05 && T_total(i) < 5
            disp("take_off")
            delta_state = state - s_eq;
        
            delta_torques = hov_ctr.update(delta_state);
            delta_omegas(:,i) = 1/(2*bebop.omegaHover)*(bebop.allocMat\delta_torques);
            bebop.applyConstOmega(delta_omegas(:,i)+bebop.omegaHover,Ts);        
        
        else
            takingOff = false;
            disp("done taking off")
            X(1, :) = [bebop.s(1, end), bebop.s(2, end), bebop.s(3, end), bebop.s(9, end)]
           
        end
       
    else
        
        if i < length(p_t)
            des_point = p_t(i,:)'
        else
            des_point = p_t(end, :)'
        end
        
        des_mat = zeros(4, 4);
        des_mat(1, :) = des_point(1:4);
        des_mat(2, :) = des_point(5:8);
        des_mat(3, :) = des_point(9:12);
        des_mat(4, :) = des_point(13:16);
        des_mat
        
        u = -K*(X-des_mat);
        X_DOT = A*X + B*u;
        X = X + X_DOT*Ts
        
        X_state = reshape(X', 1, 16)'
        
        %x_state_record(i, :) = X_state;
        moments = diff_flat_ctr.update(X_state, bebop.s(:, end))
        
        omegas = bebop.allocMat\moments'
        % rotor_speed = (sqrt(omegas)/bebop.omegaHover) * (bebop.omegaHover)
        rotor_speed = sqrt(omegas);
        omegas(:, i) = rotor_speed;
        bebop.applyConstOmega(omegas(:, i), Ts);
             
    end
    
end

%% Plot Stuff
bebop.plotState();
bebop.plotInputs();
diff_flat_ctr.plotOutput();
hov_ctr.plotOutputs();

figure 
subplot(1, 3, 1)
hold on
% plot(T_total, x_state_record(:, 1))
plot(T_total, p_t(:, 1))
plot(bebop.t, bebop.s(1, :))
title("X state")
legend("desired trajectory", "bebop_x")
hold off

subplot(1, 3, 2)
hold on
plot(T_total, p_t(:, 2))
% plot(T_total, x_state_record(:, 2))
plot(bebop.t, bebop.s(2, :))
title("Y state")
legend("desired trajectory", "bebop_y")
hold off

subplot(1, 3, 3)
hold on
plot(T_total, p_t(:, 3))
%plot(T_total, x_state_record(:, 3))
plot(bebop.t, bebop.s(3, :))
title("Z state")
legend("desired trajectory", "bebop_z")
hold off

%des angles
figure 
subplot(1, 3, 1)
hold on
plot(T_total, p_t(:, 5))
plot(bebop.t, bebop.s(4, :))
title("X vel")
legend("desired trajectory", "bebop_x")
hold off

subplot(1, 3, 2)
hold on
plot(T_total, p_t(:,6))
plot(bebop.t, bebop.s(5, :))
title("Y vel")
legend("desired trajectory", "bebop_y")
hold off

subplot(1, 3, 3)
hold on
plot(T_total, p_t(:, 7))
plot(bebop.t, bebop.s(6, :))
title("Z vel")
legend("desired trajectory", "bebop_z")
hold off


%% End
    


