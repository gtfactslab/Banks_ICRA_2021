function [state] = quad_dynamics(state, inputs, rotor_vel, params_c)
% state: [x, y, z, v_x, v_y, v_z, phi, theta, psi, p, q, r] (12 x 1) -> 
%4 x 3
% inputs: moments (thrust, torque_xyz) (4 x1)
g = params_c.g; 
m = params_c.m;
dt = params_c.dt;
inertia = params_c.inertia;
disp("state before dynamics")
state = reshape(state, [4, 3])
%R (Z-Y-X)
disp("phi, theta, psi of quad")
phi = state(3, 1);
theta = state(3, 2);
psi = state(3, 3);

w_x = state(4, 1);
w_y = state(4, 2);
w_z = state(4, 3);

R = eul2rotm([psi, theta, phi], 'ZYX');

inputs
rotor_vel

omegaSum = sum( abs( rotor_vel ) );
kDragXY = -9.1785*1e-7;
kDragZ  = -10.311*1e-7;
kDrag   = diag([kDragXY kDragXY kDragZ]);
fAero   = kDrag*omegaSum*R'*state(2, :)';


Angl_Mat_B_W = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);...
    0, cos(phi), -sin(phi);...
    0, sin(phi)*sec(theta), cos(phi)*sec(theta)];


x_dot = zeros(4, 3);
x_dot(1, :) = state(2, :);
x_dot(2, :) = [0, 0, -g]' + (1/m)*R*[0, 0, inputs(1)]' + (1/m)*fAero;
x_dot(3, :) = Angl_Mat_B_W*[w_x, w_y, w_z]';

x_dot(4, :) = [inputs(2) / inertia(1, 1);
    inputs(3) / inertia(2, 2);
    inputs(4)/ inertia(3, 3)] - ...
    [((inertia(2,2) - inertia(3, 3))/(inertia(1, 1)))*w_y*w_z;
    ((inertia(3,3) - inertia(1, 1))/(inertia(2, 2)))*w_x*w_z;
    ((inertia(1,1) - inertia(2, 2))/(inertia(3, 3)))*w_x*w_y];

state(1, :) = state(1, :) + x_dot(1, :)*dt;
state(2, :) = state(2, :) + x_dot(2, :)*dt;
state(3, :) = state(3, :) + x_dot(3, :)*dt;
state(4, :) = state(4, :) + x_dot(4, :)*dt;


% new_state = state + x_dot*dt;
% state = [state(1, 1), state(2, 1), state(3, 1), state(4, 1), state
% if new_state(1, 3) < 0
%     new_state(1, 3) = 0;
%     new_state(1, :) = [0,0,0];
%     new_state(2, :) = [0, 0,0];
%     new_state(3, :) = [0,0,0];
%     new_state(4, :) = [0,0,0];
% end

end