function xk1 = stateTransition(xk,uk,omegaRotors,Ts,m,J)
%STATETRANSITION(xk,uk,Ts,m,J) discrete time state transition function
%for a quadcopter
%   Model discretized by forward euler method.
%   The state is x=[px py pz vx vy vz phi theta psi p q r]. angular
%   velocities [p q r] are esxpressed in the body-frame
%   xk state at step k
%   uk input at step k (Mx, My, Mz, T)'
%   m mass
%   J inertia diagonal matrix


g = 9.81;
rotm = eul2rotm([ xk(9) xk(8) xk(7)],'ZYX');
%rotm = eul2rotm([  xk(7) xk(8) xk(9)  ],'XYZ');


% omegaSum = sum( abs( omegaRotors ) );
% kDragXY = 130* -9.1785*1e-7;
% kDragZ  = 130* -10.311*1e-7;
% kDrag   = diag([kDragXY kDragXY kDragZ]);
% fAreo   = kDrag*omegaSum*rotm'*[xk(4) xk(5) xk(6)]';

% uav states
xk1(1)=  xk(1)  + Ts* xk(4);
xk1(2)=  xk(2)  + Ts* xk(5);
xk1(3)=  xk(3)  + Ts* xk(6);


xk1(4:6) = xk(4:6)+Ts*( 1/m*rotm*( [0 0 uk(4)]')-[0 0 g]' );

% transformation from body-frame to inertial-frame
tranMatrix = [ 1 sin(xk(7))*tan(xk(8))  cos(xk(7))*tan(xk(8));
               0 cos(xk(7))             -sin(xk(7));
               0 sin(xk(7))/cos(xk(8))  cos(xk(7))/cos(xk(8))];
tmp = tranMatrix * [xk(10); xk(11); xk(12)];

xk1(7)=  xk(7) + Ts* tmp(1);
xk1(8)=  xk(8) + Ts* tmp(2);
xk1(9)=  xk(9) + Ts* tmp(3);

xk1(10)= xk(10) + Ts*( 1/J(1,1)*( uk(1)-( J(3,3)-J(2,2) )*xk(11)*xk(12) ) );
xk1(11)= xk(11) + Ts*( 1/J(2,2)*( uk(2)-( J(1,1)-J(3,3) )*xk(10)*xk(12) ) );
xk1(12)= xk(12) + Ts*( 1/J(3,3)*( uk(3)-( J(2,2)-J(1,1) )*xk(10)*xk(11) ) );


end

