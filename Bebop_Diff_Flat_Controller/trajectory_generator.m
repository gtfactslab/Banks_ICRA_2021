function [p_t] = trajectory_generator(T, type, dt)
% generate C^n differentiable trajectory for t seconds where type
%indicates line, circle, etc.
% p_t = [x, y, z, yaw] -> [x''', y''', z''', yaw''']

%default dt
tot_time = 0:dt:T;
p_t = zeros(length(tot_time), 16); % t states, state = [x, y, z, yaw; x',y', z', yaw';...;x''', y''', z''', yaw''']
yaw = 0;
disp("string")
if strcmp(type, 'line')
    x = 0:dt:1;
    y = zeros(1, length(x));
    z = 0.8*ones(1, length(x));
    t_pts = linspace(0, T, length(x));
% 
%     
%     length(x)
%     length(y)
%     length(z)
%     length(t_pts)
%     length(tot_time)
    wpts = [x; %x
            y; %y 
            z]; %z
        
        
    [q, qd, qdd, pp] = quinticpolytraj(wpts, t_pts, tot_time);
%     
%     q(1, 1)
%     size(q(1, :))
%     size(qd)
%     size(qdd)
    yaw_zeros = zeros(length(q), 1);
    p_t = [q' yaw_zeros,...
        qd' yaw_zeros,...
        qdd' yaw_zeros,...
        yaw_zeros yaw_zeros yaw_zeros yaw_zeros];
    

elseif strcmp(type, 'circle')

    disp("circle trajectory")
    radii = 1.0;
    des_height = 0.8;
    om = 0.5*pi;
    count = 1;
    for i=0:dt:T
        thet = i*om + pi/2 ;
        arr = [ radii * cos(thet),         radii * sin(thet),        des_height, yaw,...
               -radii * om * sin(thet),    radii * om * cos(thet),   0,          yaw,...
               -radii * om^2 * cos(thet), -radii * om^2 * sin(thet), 0,          yaw,...
                radii * om^3 * sin(thet), -radii * om ^3 * cos(thet),0,          yaw];
                 
        p_t(count, :) = arr;  
        count = count +1;
    end
    
    
else 
    disp("no shape defined")
              
end
    

end