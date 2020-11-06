% script to compute the linearized version of the quadcopter model
clear; close all;

% state s=[s1,...,s12]=[x y z xdot ydot zdot phi theta psi p q r]
% input u=[u1,...,u4]=[thrust mx my mz]; 

%symbolic parameters of the drone
syms g m d ct cq Ixx Iyy Izz positive real;
%symbolic state
syms s [1 12] real
%symbolic input u =[thrust, mx, my, mz]
syms u [1 4] real;
%rotors angular velocities. Not necessary for the computation of the
%linearized model
syms w [1 4] real;

%inertia matrix
I = diag([Ixx Iyy Izz]);

% Z-Y-X euler angle convention
Rz =[cos(s9) -sin(s9) 0; 
     sin(s9) cos(s9)  0;
        0       0     1];
    
Ry = [cos(s8)  0  sin(s8);
        0      1       0;
      -sin(s8) 0  cos(s8)];
  
Rx = [1     0       0;
      0 cos(s7) -sin(s7);
      0 sin(s7)  cos(s7)];

R = Rz*Ry*Rx;

% matrix to transform the angular velocity vector to roll,pitch,yaw
% velocities
T = [ 1 sin(s7)*tan(s8)  cos(s7)*tan(s8);
      0 cos(s7)             -sin(s7);
      0 sin(s7)/cos(s8)  cos(s7)/cos(s8)];

%symbolic non linear model
f(1:3,1) = [s4 s5 s6]';
f(4:6,1) = -[0 0 g]'+ 1/m*R*([0 0 u(1)]');
f(7:9,1) = T*[s10 s11 s12]';
f(10:12,1) = inv(I)*( u(2:4)'-cross([s10 s11 s12]',I*[s10 s11 s12]'));

% hover equilibrium
ueq = [m*g, 0, 0, 0,];
seq = [ s1 s2 s3 0 0 0 0 0 0 0 0 0];

% jacobian matrices
JA = jacobian(f,s);
JB = jacobian(f,u);

% compute symbolic A and B matrices substituting hover condition in the
% jacobian matrices
A = subs(JA,[s,u],[seq,ueq])
B = subs(JB,[s,u],[seq,ueq])

%thrust force
ft = ct*(w1^2+w2^2+w3^2+w4^2);

% moments vector 
Mb = [d*ct/sqrt(2)*(-w1^2-w2^2+w3^2+w4^2);
      d*ct/sqrt(2)*(-w1^2+w2^2+w3^2-w4^2);
      cq*(-w1^2+w2^2-w3^2+w4^2)];

%allocation matrix
Tau = [ ct*[1 1 1 1];
        (d/sqrt(2))*ct*[-1 -1 1 1];
        (d/sqrt(2))*ct*[-1  1 1 -1];
        cq*[-1 1 -1 1]];

TauInv = inv(Tau);

%% Subsystems
Ts=0.05;

prefilter=zpk([],[-1 -1],1);
prefilter=c2d(prefilter,Ts);

%longitudinal
Ax = [0 1 0 0; 0 0 g 0;0 0 0 1; 0 0 0 0]; 
Bx = [0 0 0 1/Iyy]';
C = eye(4);
D = zeros(size(C,1),size(Bx,2));

Ax=double(subs(Ax,g,9.81));
Bx=double(subs(Bx,Iyy,1.657171e-5));

Gx = ss(Ax,Bx,C,D);
p=[-15 -16 -17 -18]/2;
K=place(Ax,Bx,p);
Cy=[1 0 0 0];
G=inv(Cy*inv(-Ax+Bx*K)*Bx);

Q=[0.5 0 0 0;
    0 0.5 0 0; 
    0 0 1 0;
    0 0 0 1];
R=0.01;
[Kd,S,e] = lqrd(Ax,Bx,Q,R,Ts);

Gxi = ss(Ax,Bx,[1 0 0 0],0);
Ki = lqi(Gxi,eye(5),1,zeros(5,1));

%% altitude
Az=[0 1; 0 0]; Bz=[0 1/m]';
Bz=double(subs(Bz,m,0.25));



Gz = ss(Az,Bz,eye(2),zeros(2,1));


% Gz=ss(Az,Bz,[1 0],0);
% pidz=pidtune(Gz,'PIDF',3);
% step(feedback(pidz*Gz,1))
% step(pidz/(1+pidz*Gz))

%Gz = zpk(Gz);
%Gx=zpk([],[0 0 0 0],1)


Gz = ss(Az,Bz,eye(2),zeros(2,1));
Kd2= lqrd(Az,Bz,eye(2),0.5,Ts);
Wz = feedback(Kd2*Gz,1);
step(Wz)
step(Kd2/(1+Kd2*Gz))


