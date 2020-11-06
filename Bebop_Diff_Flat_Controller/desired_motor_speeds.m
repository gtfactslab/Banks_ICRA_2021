
%set desired motor speeds (angular velocities) based on motor dynamics
function v =  desired_motor_speeds(u_1, u_2, u_3, u_4)
%input: thrust and moments
%output: motor angular velocities
 % roll, pitch and yaw and thrust in the body frame (?)
 %roll_rate(?) / pitch_rate(?) / yaw_rate(?) 
 disp("get motor speeds")
 load('params.mat', 'k_f', 'k_m', 'inertia',  'L');
 u_1; % thrust
 u_2; % t_x
 u_3; % t_y
 u_4; % t_z
 
 %sigma is the sign matrix of v
%  SIGMA = [1, 1, -1, -1; -1, 1, 1, -1;
%          1, -1, 1, -1; 1, 1, 1, 1];
%  
%  B = [((inertia(3, 3) - inertia(2, 2))*w_z*w_y) / L*k;
%      ((inertia(1, 1) - inertia(3, 3))*w_z*w_x)/L*k;
%      ((inertia(2, 2) - inertia(1, 1))*w_y*w_x) / b;
%      thrust];
%motor 1: bottom left
%motor 2: top left
%motor 3: top right
%motor 4: bottom right
% X configuration


SIGMA = [k_f, k_f, k_f, k_f;
        -k_f*(L/sqrt(2)), -k_f*(L/sqrt(2)), k_f*(L/sqrt(2)), k_f*(L/sqrt(2));
        -k_f*(L/sqrt(2)), k_f*(L/sqrt(2)), k_f*(L/sqrt(2)), -k_f*(L/sqrt(2));
        -k_m*k_f, k_m*k_f, -k_m*k_f, k_m*k_f];
    
B = [u_1;
    u_2;
    u_3;
    u_4];
        

 v = sqrt(linsolve(SIGMA, B));
end