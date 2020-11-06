classdef SubSysController < handle
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (GetAccess = public, SetAccess = private)
        
        Ts;
        
        m;
        Ixx; Iyy; Izz;
        g=9.81;
        
        xPrefilter; xPrefilterState;
        yPrefilter; yPrefilterState;
        zPrefilter; zPrefilterState;
        psiPrefilter; psiPrefilterState;
        
        longSys; Kx; deltaMy=[]; Gx;
        latSys; Ky; deltaMx=[]; Gy;
        altSys; Kz; deltaFt=[]; Gz;
        dirSys; Kpsi; deltaMz=[]; Gpsi;
        
    
        p=[-1 -2 -3 -4];

        xRef=0;
        yRef=0;
        zRef=0;
        psiRef=0;
        
        inputs=[];
        
    end
    
    methods
        
        function obj = SubSysController(mass, inertia, Ts)
            obj.Ts = Ts;
            obj.m = mass;
            obj.Ixx = inertia(1,1);
            obj.Iyy = inertia(2,2);
            obj.Izz = inertia(3,3);
            
            Ay = [0 1 0 0; 0 0 -obj.g 0; 0 0 0 1; 0 0 0 0]; 
            By = [0; 0; 0; 1/obj.Ixx];
            Cy = [1 0 0 0];
            obj.latSys = ss(Ay,By,eye(4),zeros(4,1));
            %obj.Ky = lqrd(Ay,By,obj.Q,obj.R,Ts);
            obj.Ky=place(Ay,By,obj.p*2);
            obj.Gy=inv(Cy*inv(-Ay+By*obj.Ky)*By);
            obj.yPrefilter = zpk([],[-7 -7],49);
            obj.yPrefilter = ss(obj.yPrefilter);
            obj.yPrefilter = c2d(obj.yPrefilter,Ts);
            obj.yPrefilterState = zeros(2,1);
            
            Ax = [0 1 0 0; 0 0 obj.g 0; 0 0 0 1; 0 0 0 0]; 
            Bx = [0; 0; 0; 1/obj.Iyy];
            Cx=[1 0 0 0];
            obj.longSys = ss(Ax,Bx,eye(4),zeros(4,1));
            %obj.Kx = lqrd(Ax,Bx,obj.Q,obj.R,Ts);
            obj.Kx=place(Ax,Bx,obj.p*2);
            obj.Gx=inv(Cx*inv(-Ax+Bx*obj.Kx)*Bx);
            obj.xPrefilter = obj.yPrefilter;
            obj.xPrefilterState = obj.yPrefilterState;
            
            Az = [0 1; 0 0]; 
            Bz = [0; 1/obj.m];
            Cz = [1 0];
            obj.altSys = ss(Az,Bz,eye(2),zeros(2,1));
            %obj.Kz = lqrd(Az,Bz,eye(2),0.5,Ts);
            obj.Kz=place(Az,Bz,obj.p(1:2));
            obj.Gz=inv(Cz*inv(-Az+Bz*obj.Kz)*Bz);
            obj.zPrefilter = obj.yPrefilter;
            obj.zPrefilterState = obj.yPrefilterState;
            
            Apsi = [0 1; 0 0]; 
            Bpsi = [0; 1/obj.Izz];
            Cpsi = [1 0];
            obj.altSys = ss(Apsi,Bpsi,eye(2),zeros(2,1));
            %obj.Kpsi = lqrd(Apsi,Bpsi,eye(2),obj.R,Ts);
            obj.Kpsi=place(Apsi,Bpsi,obj.p(1:2)*3);
            obj.Gpsi=inv(Cpsi*inv(-Apsi+Bpsi*obj.Kpsi)*Bpsi);
            obj.psiPrefilter = obj.yPrefilter;
            obj.psiPrefilterState = obj.yPrefilterState;
            
        end
        
        function setRef(obj, posRef, psiRef)
            
            if size(posRef,1)~=3 || size(posRef,2)~=1 || ~isreal(posRef)
                error('posRef vector must be a real 3x1 vector');
            end
            if psiRef<-2*pi || psiRef>2*pi
                %error('psiRef must be in the interval [-2*pi,2*pi]')
                % returns the remainder of the division with the same sign
                % of the dividend: rem(-3*pi,2*pi)=-pi
                psiRef=rem(psiRef,2*pi);
            end
            
            obj.xRef = posRef(1);
            obj.yRef = posRef(2);
            obj.zRef = posRef(3);
            obj.psiRef = psiRef;
        end
        
        function commands = update(obj,deltaState)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            if size(deltaState,1)~=12 || size(deltaState,2)~=1 || ~isreal(deltaState)
                error('deltaState vector must be a real 12x1 vector');
            end
            
            obj.inputs(:,end+1)=deltaState;
            
            obj.deltaFt(:,end+1) = obj.verticalCtr(deltaState([3 6]));
            obj.deltaMx(:,end+1) = obj.lateralCtr(deltaState([2 5 7 10]));
            obj.deltaMy(:,end+1) = obj.longitudinalCtr(deltaState([1 4 8 11]));
            obj.deltaMz(:,end+1) = obj.directionalCtr(deltaState([9 12]));
            
            commands=[obj.deltaFt(:,end) obj.deltaMx(:,end) obj.deltaMy(:,end) obj.deltaMz(:,end)]';
        end
        
        function fig = plotOutputs(obj)
        
            fig=figure();
            fig.Name='controller outputs';
            times = (1:length(obj.deltaFt))*obj.Ts;
            
            subplot(2,2,1)
            stairs(times,obj.deltaFt); grid on;
            title('thrust')
            xlabel('s'); ylabel('N');
            
            subplot(2,2,2)
            stairs(times,obj.deltaMx); grid on;
            title('Mx')
            xlabel('s'); ylabel('Nm');
            
            subplot(2,2,3)
            stairs(times,obj.deltaMy); grid on;
            title('My')
            xlabel('s'); ylabel('Nm');
            
            subplot(2,2,4)
            stairs(times,obj.deltaMz); grid on;
            title('Mz')
            xlabel('s'); ylabel('Nm');
            
        end

         function fig = plotInputs(obj)

            fig=figure();
            fig.Name = 'controller inputs';
            times = (1:size(obj.inputs,2))*obj.Ts;

            subplot(2,2,1)
            stairs(times,obj.inputs(1,:)); grid on; hold on;
            stairs(times,obj.inputs(2,:));
            stairs(times,obj.inputs(3,:));
            title('positions')
            xlabel('s'); ylabel('m');
            legend('x','y','z');

            subplot(2,2,2)
            stairs(times,obj.inputs(4,:)); grid on; hold on;
            stairs(times,obj.inputs(5,:));
            stairs(times,obj.inputs(6,:));
            title('velocities')
            xlabel('s'); ylabel('m/s');
            legend('xdot','ydot','zdot');

            subplot(2,2,3)
            stairs(times,rad2deg(obj.inputs(7,:))); grid on; hold on;
            stairs(times,rad2deg(obj.inputs(8,:)));
            stairs(times,rad2deg(obj.inputs(9,:)));
            title('euler angles')
            xlabel('s'); ylabel('deg');
            legend('phi','theta','psi');

            subplot(2,2,4)
            stairs(times,rad2deg(obj.inputs(10,:))); grid on; hold on;
            stairs(times,rad2deg(obj.inputs(11,:)));
            stairs(times,rad2deg(obj.inputs(12,:)));
            title('angular velocities vector')
            xlabel('s'); ylabel('deg/s');
            legend('p','q','r');

        end
        
    end
    
    methods(Access=private)
        
        function deltaFt = verticalCtr(obj,delta)
            ref = obj.zPrefilter.C*obj.zPrefilterState(:,end) + obj.zPrefilter.D*obj.zRef;
            obj.zPrefilterState(:,end+1) = obj.zPrefilter.A*obj.zPrefilterState(:,end)... 
                                            + obj.zPrefilter.B*obj.zRef;
                                        
            %deltaFt = obj.Kz*( [obj.zRef; 0] - delta );
            %deltaFt = obj.Kz*( [ref; 0] - delta );
            %deltaFt = obj.Gz*obj.zRef -obj.Kz*delta;
            deltaFt = obj.Gz*ref -obj.Kz*delta;
        end
        
        function deltaMx = lateralCtr(obj,delta)
            
            ref = obj.yPrefilter.C*obj.yPrefilterState(:,end) + obj.yPrefilter.D*obj.yRef;
            obj.yPrefilterState(:,end+1) = obj.yPrefilter.A*obj.yPrefilterState(:,end)... 
                                            + obj.yPrefilter.B*obj.yRef;
                                        
            %deltaMx= obj.Ky*([obj.yRef; 0; 0; 0]-delta);
            %deltaMx= obj.Ky*([ref; 0; 0; 0]-delta);
            %deltaMx = obj.Gy*obj.yRef -obj.Ky*delta;
            deltaMx = obj.Gy*ref -obj.Ky*delta;
            
        end
        
        function deltaMy = longitudinalCtr(obj,delta)
            
            ref = obj.xPrefilter.C*obj.xPrefilterState(:,end) + obj.xPrefilter.D*obj.xRef;
            obj.xPrefilterState(:,end+1) = obj.xPrefilter.A*obj.xPrefilterState(:,end)... 
                                            + obj.xPrefilter.B*obj.xRef;
                                        
            %deltaMy= obj.Kx*([obj.xRef; 0; 0; 0]-delta);
            %deltaMy= obj.Kx*([ref; 0; 0; 0]-delta);
            %deltaMy = obj.Gx*obj.xRef -obj.Kx*delta;
            deltaMy = obj.Gx*ref -obj.Kx*delta;
            
        end
        
        function deltaMz = directionalCtr(obj,delta)
            
            ref = obj.psiPrefilter.C*obj.psiPrefilterState(:,end) + obj.psiPrefilter.D*obj.psiRef;
            obj.psiPrefilterState(:,end+1) = obj.psiPrefilter.A*obj.psiPrefilterState(:,end)... 
                                            + obj.psiPrefilter.B*obj.psiRef;
                                        
            %deltaMz= obj.Kpsi*([obj.psiRef; 0]- delta);
            %deltaMz= obj.Kpsi*([ref; 0]- delta);
            %deltaMz = obj.Gpsi*obj.psiRef -obj.Kpsi*delta;
            deltaMz = obj.Gpsi*ref -obj.Kpsi*delta;
        end
        
    end
end
