clear; close all;
 % state s=[s1,...,s12]=[x y z xdot ydot zdot phi theta psi p q r]
syms g m d ct cq Ixx Iyy Izz positive real;
syms s [1 12] real
syms u [1 4] positive real;
%syms ue;


I = diag([Ixx Iyy Izz]);

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

T = [ 1 sin(s7)*tan(s8)  cos(s7)*tan(s8);
      0 cos(s7)             -sin(s7);
      0 sin(s7)/cos(s8)  cos(s7)/cos(s8)];


ft = ct*(u1^2+u2^2+u3^2+u4^2);
    
Mb = [d*ct/sqrt(2)*(-u1^2-u2^2+u3^2+u4^2);
      d*ct/sqrt(2)*(-u1^2+u2^2+u3^2-u4^2);
      cq*(-u1^2+u2^2-u3^2+u4^2)];

f(1:3,1) = [s4 s5 s6]';
f(4:6,1) = -[0 0 g]'+ 1/m*R*([0 0 ft]');
f(7:9,1) = T*[s10 s11 s12]';
f(10:12,1) = inv(I)*( Mb-cross([s10 s11 s12]',I*[s10 s11 s12]'));


% ueq = ones(1,4) *sqrt(m*g/(4*ct));
ueq = ones(1,4) *sqrt(m*g/(4*ct));
%ueq= ones(1,4)*ue;
seq = [ s1 s2 s3 0 0 0 0 0 0 0 0 0];

JA = jacobian(f,s);
JB = jacobian(f,u);

A = subs(JA,[s,u],[seq,ueq]);
B = subs(JB,[s,u],[seq,ueq]);

Tau = B([6 10 11 12],:);

TauInv = inv(Tau);

save('linOmega', 'A','B','Tau','TauInv');
