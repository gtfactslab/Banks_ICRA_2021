% bebop2 physical parameters according to bebopS package
% Z axis is upward so gravity force is negative
g=9.81;

%mass
m=0.5;

%arm length
l=0.12905;

%inertia
J=diag([0.00389 0.00389 0.0078]);

% rotor radius
% r=0.0508

% propellers
% force = w^2 * motorconstant
motor_constant = 8.54858e-06;
% torque = w^2 * motor_constant * moment_constant
moment_constant = 0.016;

drag_constant = 8.06428e-05;

% allocation matrix for a quadcopter in X configuration 
% [Mx My Mz T] = alloc * [w1; w2; w3; w4]^2
% rotation direction of each motor
% motor0 (front-right) ccw; 
% motor1 (front-left) cw;
% motor2 (back-left) ccw;
% motor3 (back-right) cw
kt = motor_constant;
kq = motor_constant*moment_constant;
allocMatXconf = [   l*kt/sqrt(2)*[-1 -1 +1 +1] ;... first 2 lines comes from a sum(PixFi)
                    l*kt/sqrt(2)*[-1 +1 +1 -1];...
                    kq*[-1 1 -1 1];... cw yields to a positive induced momentum
                    kt*[1 1 1 1]]; % thrust force
                