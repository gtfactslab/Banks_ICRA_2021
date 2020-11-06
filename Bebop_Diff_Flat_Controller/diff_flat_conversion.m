function [w_x, w_y, w_z, thrust, roll, pitch, yaw] = diff_flat_conversion(state, params_c)
% Generate diff flat outputs based on state
% input : desired state (up to 3rd derivative)
% return : angular velocity vector(w_x, w_y, w_z), thrust
% hover = params_c.nom_thrust; % nominal thrust (?
disp("diff flat conversion")
state;
g= params_c.g;
m = params_c.m;
beta_1 = state(3, 1);
beta_2 = state(3, 3) + g;
beta_3 = state(3, 2);

roll = atan2(beta_3, sqrt(beta_1^2 + beta_2^2));
pitch = atan2(beta_1, beta_2);
yaw = 0; %fixed yaw


%calculate angular velocity
t = [beta_1, beta_3, beta_2];
z_b = t / norm(t);
z_b = z_b';

x_c = [cos(yaw), sin(yaw), 0];

y_b = cross(z_b, x_c) / norm(cross(z_b, x_c));

x_b = cross(y_b, z_b);

acc = m*state(3, 1:3)+ m*g*[0,0, g];
acc = acc';
pr_z = z_b'*z_b;

u_1 = (z_b'*acc) / pr_z;

%projection of jerk
% dot(z_b, state(4, 1:3));
% dot(z_b,state(4, 1:3))*z_b;
h_w = (m/u_1)*(state(4, 1:3) - (dot(z_b,state(4, 1:3))*z_b)');

%angular velocity
w_x = -dot(h_w,y_b);
w_y = dot(h_w,x_b);
w_z = dot(state(2, 4)*[0, 0, 1],z_b);
    

% a_temp = norm([0, 0, g] - state(3, 1:3));
% thrust = (a_temp/g)*hover;
% thrust = norm(m*state(3, 1:3)-m*[0,0, g]);
thrust = m*norm(t);
end