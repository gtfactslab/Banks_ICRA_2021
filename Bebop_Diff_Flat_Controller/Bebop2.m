classdef Bebop2 < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(GetAccess = public, SetAccess = private)
        g = 9.81;
        m = 0.5; %0.5
        d = 0.12905;
        ct = 1.65e-06;%8.54858e-06; %2.248e-06;
        cq =  1.65e-06 * 0.016; %8.54858e-06 * 0.016; %2.248e-06;
        Ixx = 0.00389;
        Iyy = 0.00389;
        Izz = 0.0078;
        I = diag([0.00389,0.00389,0.0078]);
        
        omegaHover %880(?) rad/s
        omegaMax=1475; %rad/s
        
        s
        u=zeros(4,1);
        t=0; tf;
        
        %x configuration allocation matrix
        allocMat
        
        
    end
    
    methods
        function obj = Bebop2(s0)
          
            if size(s0,1)~=12 || size(s0,2)~=1
                error('initial state must be a 12x1 vector');
            end
                               
            obj.s = s0;
            obj.omegaHover=sqrt(obj.m*obj.g/(4*obj.ct));
            obj.allocMat = [ obj.ct*[+1 +1 +1 +1]; 
                        obj.ct*obj.d/sqrt(2)*[-1 -1 +1 +1]; 
                        obj.ct*obj.d/sqrt(2)*[-1 +1 +1 -1]; 
                        obj.cq*[-1 +1 -1 +1]];
        end
        
        function omega = forces2omega(obj,forces)
                omega = obj.Tau\forces;
                omega=sqrt(omega);
        end
        
        
        function [y,ty] = applyConstOmega(obj,omega,tspan)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            if size(omega,1)~=4 || size(omega,2)~=1
                error('omega vector must be 4x1');
            end
            if  size(tspan,1)~=1 || size(tspan,2)~=1 || tspan<=0 
                error('tspan must be a positive real')
            end
            
            limit= omega > obj.omegaMax;
            if any(limit) 
                warning('input rotors velocity is bigger than the physical max rotors velocity 1475 rad/s')
                for i=1:4
                    if limit(i)
                            omega(i)=obj.omegaMax;    
                    end
                end
                    
            end
            
            limit= omega < 0;
            if any(limit) 
                warning('input rotors velocity is negative')
                for i=1:4
                    if limit(i)
                            omega(i)=0;    
                    end
                end
                    
            end
            
            
            [ty,y] = ode45( @(t,s)obj.constantInput(t,s,...
                omega(1),omega(2),omega(3),omega(4),zeros(3,1),zeros(3,1)),...
                [obj.t(end) obj.t(end)+tspan], obj.s(:,end));
            
            obj.u = [obj.u omega*ones(1,length(ty)-1)];
            
            obj.s=[obj.s, y(2:end,:)'];
            obj.t=[obj.t, ty(2:end)'];
            obj.tf=obj.t(end);
            
        end
        
        function [y,ty] = applyConstOmegaForce(obj,omega,forceDisturb,tspan)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            if size(omega,1)~=4 || size(omega,2)~=1 || ~isreal(omega)
                error('omega vector must be a real 4x1 vector');
            end
            
            if size(forceDisturb,1)~=3 || size(forceDisturb,2)~=1 || ~isreal(forceDisturb)
                error('the force disturb vector must be a real 3x1 vector');
            end
            
            
            if  size(tspan,1)~=1 || size(tspan,2)~=1 || tspan<=0 
                error('tspan must be a positive real')
            end
            
            limit= omega > obj.omegaMax;
            if any(limit) 
                warning('input rotors velocity is bigger than the physical max rotors velocity 1475 rad/s')
                for i=1:4
                    if limit(i)
                            omega(i)=obj.omegaMax;    
                    end
                end
                    
            end
            
            limit= omega < 0;
            if any(limit) 
                warning('input rotors velocity is negative')
                for i=1:4
                    if limit(i)
                            omega(i)=0;    
                    end
                end
                    
            end
            
            
            [ty,y] = ode45( @(t,s)obj.constantInput(t,s,...
                omega(1),omega(2),omega(3),omega(4),forceDisturb,zeros(3,1)),...
                [obj.t(end) obj.t(end)+tspan], obj.s(:,end));
            
            obj.u = [obj.u omega*ones(1,length(ty)-1)];
            
            obj.s=[obj.s, y(2:end,:)'];
            obj.t=[obj.t, ty(2:end)'];
            obj.tf=obj.t(end);
            
        end
        
        function [y,ty] = applyConstOmegaForceTorque(obj,omega,forceDisturb,torqueDisturb,tspan)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            if size(omega,1)~=4 || size(omega,2)~=1 || ~isreal(omega)
                error('omega vector must be a real 4x1 vector');
            end
            
            if size(forceDisturb,1)~=3 || size(forceDisturb,2)~=1 || ~isreal(forceDisturb)
                error('the force disturb vector must be a real 3x1 vector');
            end
            
            if size(torqueDisturb,1)~=3 || size(torqueDisturb,2)~=1 || ~isreal(torqueDisturb)
                error('the torque disturb vector must be a real 3x1 vector');
            end
            
            
            if  size(tspan,1)~=1 || size(tspan,2)~=1 || tspan<=0 
                error('tspan must be a positive real')
            end
            
            limit= omega > obj.omegaMax;
            if any(limit) 
                warning('input rotors velocity is bigger than the physical max rotors velocity 1475 rad/s')
                for i=1:4
                    if limit(i)
                            omega(i)=obj.omegaMax;    
                    end
                end
                    
            end
            
            limit= omega < 0;
            if any(limit) 
                warning('input rotors velocity is negative')
                for i=1:4
                    if limit(i)
                            omega(i)=0;    
                    end
                end
                    
            end
            
            
            [ty,y] = ode45( @(t,s)obj.constantInput(t,s,...
                omega(1),omega(2),omega(3),omega(4),forceDisturb,torqueDisturb),...
                [obj.t(end) obj.t(end)+tspan], obj.s(:,end));
            
            obj.u = [obj.u omega*ones(1,length(ty)-1)];
            
            obj.s=[obj.s, y(2:end,:)'];
            obj.t=[obj.t, ty(2:end)'];
            obj.tf=obj.t(end);
            
        end
        
        function [y,ty] = applyVarOmega(obj,omega,omegaT,tspan)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            if size(omega,1)~=4
                error('omega must be a 4xN matrix where N is the number off different inputs');
            end
            if size(omegaT,1)~=1 && size(omegaT,2)~=1
                error('omegaT must be a vector');
            end
            if lenght(omegaT)~=size(omega,2)
                error('omega and omegaT must have the same number of samples')
            end
             
            if  size(tspan,1)~=1 || size(tspan,2)~=1 || tspan<=0
                error('tspan must be a positive real')
            end
           
            if any(any(omega>obj.omegaMax,2))
                warning('input rotors velocity is bigger than the physical max rotors velocity 1475 rad/s')
            end
            
            [ty,y] = ode45( @(t,s)obj.varInput(t,s,...
                omega(1,:)',omega(2,:)',omega(3,:)',omega(4,:)', obj.t(end)+omegaT),...
                [obj.t(end), obj.t(end)+tspan], obj.s(:,end) ) ;
            % obj.updateVar(y,ty,tspan,omega,omegaT);
            
            omegaI(1,:)=interp1(ty(1)+omegaT,omega(1,:),ty(2:end))';
            omegaI(2,:)=interp1(ty(1)+omegaT,omega(2,:),ty(2:end))';
            omegaI(3,:)=interp1(ty(1)+omegaT,omega(3,:),ty(2:end))';
            omegaI(4,:)=interp1(ty(1)+omegaT,omega(4,:),ty(2:end))';
            obj.u = [obj.u, omegaI];
            
            obj.s=[obj.s, y(2:end,:)'];
            obj.t=[obj.t, ty(2:end)'];
            obj.tf=obj.t(end);
        end
        
        function fig = plotState(obj)
            
            fig= figure();
            fig.Name='bebop2 non linear model';
            subplot(3,4,1);
            plot(obj.t,obj.s(1,:)); grid on;
            legend('x')
            ylabel('m'); xlabel('s')
            title('positions')
            subplot(3,4,5);
            plot(obj.t,obj.s(2,:)); grid on;
            legend('y')
            ylabel('m'); xlabel('s')
            subplot(3,4,9);
            plot(obj.t,obj.s(3,:)); grid on;
            legend('z')
            ylabel('m'); xlabel('s')
            
            subplot(3,4,2);
            plot(obj.t,obj.s(4,:)); grid on;
            legend('xdot')
            ylabel('m/s'); xlabel('s')
            title('velocities')
            subplot(3,4,6);
            plot(obj.t,obj.s(5,:)); grid on;
            legend('ydot')
            ylabel('m/s'); xlabel('s')
            subplot(3,4,10);
            plot(obj.t,obj.s(6,:)); grid on;
            legend('zdot')
            ylabel('m/s'); xlabel('s')
            
            subplot(3,4,3);
            plot(obj.t,rad2deg(obj.s(7,:))); grid on;
            legend('phi')
            ylabel('deg'); xlabel('s')
            title('angles')
            subplot(3,4,7);
            plot(obj.t,rad2deg(obj.s(8,:))); grid on;
            legend('theta')
            ylabel('deg'); xlabel('s')
            subplot(3,4,11);
            plot(obj.t,rad2deg(obj.s(9,:))); grid on;
            legend('psi')
            ylabel('deg'); xlabel('s')
            
            subplot(3,4,4);
            plot(obj.t,rad2deg(obj.s(10,:))); grid on;
            legend('p')
            ylabel('deg/s'); xlabel('s')
            title('angle velocity vector')
            subplot(3,4,8);
            plot(obj.t,rad2deg(obj.s(11,:))); grid on;
            legend('q')
            ylabel('deg/s'); xlabel('s')
            subplot(3,4,12);
            plot(obj.t,rad2deg(obj.s(12,:))); grid on;
            legend('r')
            ylabel('deg/s'); xlabel('s')


        end
        
        function fig=plotInputs(obj)
        
            fig= figure();
            fig.Name='bebop2 non linear model';
            %u(0) = 0;
            plot(obj.t(2:end),obj.u(1,2:end)); grid on; hold on;
            plot(obj.t(2:end),obj.u(2,2:end)); 
            plot(obj.t(2:end),obj.u(3,2:end)); 
            plot(obj.t(2:end),obj.u(4,2:end)); 
            plot(obj.t(2:end),obj.omegaHover*ones(1,length(obj.t(2:end))),'--k')
            legend('omega 1','omega 2','omega 3','omega 4','omega hover')
            ylabel('rad/s'); xlabel('s')
            title('bebop2 inputs')
        end
        
    end
    
    
    methods(Access=private)
        
        
        function sp = constantInput(obj,t,s,omega1,omega2,omega3,omega4,fw,tw)
            
            thrust = obj.ct*(omega1^2+omega2^2+omega3^2+omega4^2);
            
            Mx = obj.d*obj.ct/sqrt(2)*(-omega1^2-omega2^2+omega3^2+omega4^2);
            My = obj.d*obj.ct/sqrt(2)*(-omega1^2+omega2^2+omega3^2-omega4^2);
            Mz = obj.cq*(-omega1^2+omega2^2-omega3^2+omega4^2);
            Mc = [Mx; My; Mz];
            
            sp=obj.derivatives(s,thrust,Mc,fw,tw);
            
        end
        
        function sp = varInput(obj,t,s,omega1,omega2,omega3,omega4,omegaT)
            
            omega1i=interp1(omegaT,omega1,t);
            omega2i=interp1(omegaT,omega2,t);
            omega3i=interp1(omegaT,omega3,t);
            omega4i=interp1(omegaT,omega4,t);
                        
            thrust = obj.ct*(omega1i^2+omega2i^2+omega3i^2+omega4i^2);
            
            Mx = obj.d*obj.ct/sqrt(2)*(-omega1i^2-omega2i^2+omega3i^2+omega4i^2);
            My = obj.d*obj.ct/sqrt(2)*(-omega1i^2+omega2i^2+omega3i^2-omega4i^2);
            Mz = obj.cq*(-omega1i^2+omega2i^2-omega3i^2+omega4i^2);
            Mb = [Mx; My; Mz];
            
            sp=obj.derivatives(s,thrust,Mb);
            
        end
        
        function sp = derivatives(obj, s, thrust, Mc, fw, tw)
            
            R = eul2rotm([ s(9) s(8) s(7)],'ZYX');
            
            T = [   1 sin(s(7))*tan(s(8))  cos(s(7))*tan(s(8));
                    0 cos(s(7))            -sin(s(7));
                    0 sin(s(7))/cos(s(8))  cos(s(7))/cos(s(8))];
            
            sp(1:3,1) = s(4:6,1);
            sp(4:6,1) = -[0 0 obj.g]'+ 1/obj.m*(R*[0 0 thrust]'+fw);
            sp(7:9,1) = T*s(10:12,1);
            sp(10:12,1) = obj.I\( Mc+tw-cross(s(10:12,1),obj.I*s(10:12,1)) );
%           sp(10,1) =  1/obj.Ixx*( Mx-( obj.Izz-obj.Iyy )*s(11)*s(12) ) ;
%           sp(11,1) =  1/obj.Iyy*( My-( obj.Ixx-obj.Izz )*s(10)*s(12) ) ;
%           sp(12,1) =  1/obj.Izz*( Mz-( obj.Iyy-obj.Ixx )*s(10)*s(11) ) ;
            
        end
        
        %         function sp = varInput(obj,t,s,omega1,omega2,omega3,omega4,omegaT)
        %
        %             thrust = obj.ct*(omega1.^2+omega2.^2+omega3.^2+omega4.^2);
        %
        %             thrustI = interp1(omegaT,thrust,t);
        %
        %
        %             Mx = obj.d*obj.ct/sqrt(2)*(-omega1.^2-omega2.^2+omega3.^2+omega4.^2);
        %             My = obj.d*obj.ct/sqrt(2)*(-omega1.^2+omega2.^2+omega3.^2-omega4.^2);
        %             Mz = obj.cq*(-omega1.^2+omega2.^2-omega3.^2+omega4.^2);
        %
        %             MxI = interp1(omegaT,Mx,t);
        %             MyI = interp1(omegaT,My,t);
        %             MzI = interp1(omegaT,Mz,t);
        %
        %             Mb = [MxI; MyI; MzI];
        %
        %             R = eul2rotm([ s(9) s(8) s(7)],'ZYX');
        %             T = [    1 sin(s(7))*tan(s(8))  cos(s(7))*tan(s(8));
        %                         0 cos(s(7))             -sin(s(7));
        %                         0 sin(s(7))/cos(s(8))  cos(s(7))/cos(s(8))];
        %
        %             sp(1:3,1) = s(4:6,1);
        %             sp(4:6,1) = -[0 0 obj.g]'+ 1/obj.m*R*([0 0 thrustI]');
        %             sp(7:9,1) = T*s(10:12,1);
        %             sp(10:12,1) = obj.I\( Mb-cross(s(10:12,1),obj.I*s(10:12,1)) );
        %
        %
        %         end
        
    end
   
end

