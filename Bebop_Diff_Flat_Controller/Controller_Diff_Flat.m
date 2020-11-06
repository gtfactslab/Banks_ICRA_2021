classdef Controller_Diff_Flat < handle
    %% Differential Flatness Controller for Bebop
    % Generate motor moments based on desired state
             
   %%  Code
   properties
       % params initalization
       m; %mass
       g; % gravity 
       Ixx;
       Iyy;
       Izz;    
       e_p; % error on position (%actual - des)
       e_v; % error on velocity
       e_r; % error on rotation matrix
       e_w  % error on angular velocity
       K_p; % gain on position
       K_v; % gain on velocity
       K_r; % gain on roation matrix
       K_w; %gain on angular velocity
       hover;
       m_x;
       m_y;
       m_z;
       thrust;
       I;
       Ts;
   end
   
   
   methods
       % constructor
       function obj = Controller_Diff_Flat(m, g, inertia, K_p, K_v, K_r, K_w, Ts)
        obj.m = m;
        obj.g = g;
        obj.Ixx = inertia(1, 1);
        obj.Iyy = inertia(2, 2);
        obj.Izz = inertia(3, 3);
        obj.I = inertia;
        obj.K_p = K_p;
        obj.K_r = K_r;
        obj.K_v = K_v;
        obj.K_w = K_w;
        obj.hover = [0, 0, 1]'*m*g;
        obj.Ts = Ts;
       end
       
       % update step (input state at point)
       function moments = update(obj, des_state, bebop_state)
           
            obj.e_p = bebop_state(1:3) - des_state(1:3);
            obj.e_v = bebop_state(4:6) - des_state(5:7);

            R = eul2rotm([bebop_state(9), bebop_state(8), bebop_state(7)],'ZYX');
            
            [R_des, omega_v_des, thrust_des] = obj.diff_flat_conversion(des_state, R);
            
            omega_v_des(3) = des_state(8);
            obj.e_w =  bebop_state(10:12) - R_des'*R*omega_v_des;
            
            R_mat = 0.5*(R_des'*R - R'*R_des);
            R_mat_vee = [-R_mat(2,3); R_mat(1, 3); -R_mat(1, 2)];
            obj.e_r = R_mat_vee;
            
            moment_x_y_z  = -obj.K_r*obj.e_r - obj.K_w*obj.e_w + cross(bebop_state(10:12), obj.I*bebop_state(10:12));    
             
            moments = [thrust_des, moment_x_y_z(1), moment_x_y_z(2), moment_x_y_z(3)];
            
            obj.thrust(:, end+1) = moments(1);
            obj.m_x(:, end+1) = moments(2);
            obj.m_y(:, end+1) = moments(3);
            obj.m_z(:, end+1) = moments(4);
       
       end
       
       function [roll, pitch, yaw_rate, thrust] = diff_flat_term(obj, des_state)
           
           beta_1 = -cos(des_state(4))*des_state(9) - sin(des_state(4))*des_state(10);
           beta_2 = -sin(des_state(4))*des_state(9) + cos(des_state(4))*des_state(10);
           beta_3 = -des_state(11) + obj.g;
           
           roll = atan2(beta_2, sqrt(beta_1^2 + beta_3^2));
           pitch = atan2(beta_1, beta_3);
           yaw_rate = des_state(8);

           acc_norm = norm([0, 0, 9.8] - des_state(9:11));%-?- I don't understand this two lines         
           thrust = norm((acc_norm/obj.g)*obj.hover); 
       end
       
       % diff flat conversion
       function [R_des, omega_v, thrust_des] = diff_flat_conversion(obj, des_state, R)
            % Generate diff flat outputs based on state
            % input : desired state (up to 3rd derivative) [x, y, z, yaw]
            % return : angular velocity vector(w_x, w_y, w_z), thrust,
            % desired rotation matrix
            yaw = 0;
      
            disp("diff flat conversion")
            
            beta_1 = des_state(9);
            beta_2 = des_state(11) + obj.g;
            beta_3 = des_state(10);
% 

            % calculate angular velocity
            t = [beta_1, beta_3, beta_2]; %acceleration
            % t = des_state(9:11)'
            % acc_norm = norm([0, 0, -9.8] + des_state(9:11));
%             acc = t / norm(t);
            
            acc_des = (-obj.K_p*obj.e_p - obj.K_v*obj.e_v) + t';
                        
            z_b_bebop = R*[0, 0, 1]';
            
            thrust_des = obj.m*dot(acc_des, z_b_bebop);
            % thrust_des = (thrust_des/obj.g) * (norm(obj.hover))
            
            z_b = acc_des / norm(acc_des);

            x_c = [cos(yaw), sin(yaw), 0]';

            y_b = (cross(z_b, x_c) / norm(cross(z_b, x_c)))';

            x_b = cross(y_b, z_b);
            
            %z_w = [0, 0, 1]

            % projection of jerk
            %u_1 = obj.m*norm(t);
            %h_w = (obj.m/thrust_des)*(des_state(13:15) - (dot(z_b,des_state(13:15))*z_b))

            % angular velocity
            %w_x = -dot(h_w,y_b)
            %w_y = dot(h_w,x_b)    
            
            %ma = [x_c, y_b', z_w']
            %w_z = dot(ma(3, :),[0, 0, des_state(8)]')
    
            % check which angles are closer to original orientations 
            R_des = [x_b', y_b', z_b];
            
            omega_v = zeros(3, 1);
       end
       
       function fig = plotOutput(obj)
        
            fig=figure();
            times = (1:length(obj.thrust))*obj.Ts;
            
            subplot(1,4,1)
            plot(times,obj.thrust); 
            title('thrust')
            xlabel('s'); ylabel('N');
            
            subplot(1, 4, 2)
            plot(times,obj.m_x); 
            title('M_x')
            xlabel('s'); ylabel('Nm');
            
            subplot(1, 4, 3)
            plot(times,obj.m_y);
            title('M_y')
            xlabel('s'); ylabel('Nm');
            
            subplot(1, 4, 4)
            plot(times,obj.m_z); 
            title('M_z')
            xlabel('s'); ylabel('Nm');
            
       end 
   end
   
end
