classdef AdmittanceCtrlComplete < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(GetAccess = public, SetAccess = private)
        x; xout; f2xout; f2xs; f3xout; f3xs; mx; cx; Gx; 
        y; yout; f2yout; f2ys; f3yout; f3ys; my; cy; Gy; 
        z; zout; f2zout; f2zs; f3zout; f3zs; mz; cz; Gz; 
        yaw; yawout; f2yawout; f2yaws; f3yawout; f3yaws; myaw; cyaw; Gyaw; 
           
        Ts
        K=1;
        T=0.3;
        filter;
    end
    
    methods
        function obj = AdmittanceCtrlComplete( initCond, masses,...
                dampings, samplingTime)
            if  size(initCond,1)~=2 || size(initCond,2)~=4
                error('initCond is a 2x4 matrix. Each column is the initial position and speed for that axis')
            end
            
            
            obj.x(:,1) = initCond(:,1); % a column with [xPos; xVel]
            obj.y(:,1) = initCond(:,2);
            obj.z(:,1) = initCond(:,3);
            obj.yaw(:,1) = initCond(:,4);
            
            obj.mx = masses(1);
            obj.my = masses(2);
            obj.mz = masses(3);
            obj.myaw = masses(4);
            
            obj.cx=dampings(1);
            obj.cy=dampings(2);
            obj.cz=dampings(3);
            obj.cyaw=dampings(4);
            
            obj.Ts=samplingTime;
            
            obj.filter = ss( 1-obj.Ts/obj.T, obj.Ts/obj.T, -obj.K/obj.T, obj.K/obj.T, obj.Ts);
            
            A = [0 1; 0 -obj.cx/obj.mx];
            B = [0; 1/obj.mx];
            C = [1 0; 0 1; A(2,:)];
            D = [0; 0; B(2)];
            tmp = ss(A,B,C,D);
            obj.Gx = c2d(tmp,obj.Ts,'foh');
            obj.xout(:,1) = obj.Gx.C*obj.x(:,1);
            obj.f2xs=0; obj.f2xout=obj.filter.C*obj.f2xs+obj.filter.D*obj.xout(2,end);
            obj.f3xs=0; obj.f3xout=obj.filter.C*obj.f3xs+obj.filter.D*obj.f2xout(end);
    
            
         
            A = [0 1; 0 -obj.cy/obj.my];
            B = [0; 1/obj.my];
            C = [1 0; 0 1; A(2,:)];
            D = [0;0; B(2)];
            tmp = ss(A,B,C,D);
            obj.Gy = c2d(tmp,obj.Ts,'foh');
            obj.yout(:,1) = obj.Gy.C*obj.y(:,1);
            obj.f2ys=0; obj.f2yout=obj.filter.C*obj.f2ys+obj.filter.D*obj.yout(2,end);
            obj.f3ys=0; obj.f3yout=obj.filter.C*obj.f3ys+obj.filter.D*obj.f2yout(end);
    
            
            
            A = [0 1; 0 -obj.cz/obj.mz];
            B = [0; 1/obj.mz];
            C = [1 0; 0 1; A(2,:)];
            D = [0;0; B(2)];
            tmp = ss(A,B,C,D);
            obj.Gz = c2d(tmp,obj.Ts,'foh');
            obj.zout(:,1) = obj.Gz.C*obj.z(:,1);
            obj.f2zs=0; obj.f2zout=obj.filter.C*obj.f2zs+obj.filter.D*obj.zout(2,end);
            obj.f3zs=0; obj.f3zout=obj.filter.C*obj.f3zs+obj.filter.D*obj.f2zout(end);
            
            
            A = [0 1; 0 -obj.cyaw/obj.myaw];
            B = [0; 1/obj.myaw];
            C = [1 0; 0 1; A(2,:)];
            D = [0;0; B(2)];
            tmp = ss(A,B,C,D);
            obj.Gyaw = c2d(tmp,obj.Ts,'foh');
            obj.yawout(:,1) = obj.Gyaw.C*obj.yaw(:,1);
            obj.f2yaws=0; obj.f2yawout=obj.filter.C*obj.f2yaws+obj.filter.D*obj.yawout(2,end);
            obj.f3yaws=0; obj.f3yawout=obj.filter.C*obj.f3yaws+obj.filter.D*obj.f2yawout(end);
            
        end
        
        function update(obj,forces)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            if  size(forces,1)~=4 || size(forces,2)~=1
                error('forces must be a 4x1 real vector')
            end
            
            %x axis
            obj.xout(:,end+1) = obj.Gx.C*obj.x(:,end) + obj.Gx.D*forces(1);
            obj.x(:,end+1) = obj.Gx.A*obj.x(:,end) + obj.Gx.B*forces(1);
            % x second der
            obj.f2xout(end+1) = obj.filter.C*obj.f2xs(end)+obj.filter.D*obj.xout(2,end);
            obj.f2xs(end+1) = obj.filter.A*obj.f2xs(end)+obj.filter.B*obj.xout(2,end);
            % x third der
            obj.f3xout(1,end+1) = obj.filter.C*obj.f3xs(end)+obj.filter.D*obj.f2xout(end);
            obj.f3xs(1,end+1) = obj.filter.A*obj.f3xs(end)+obj.filter.B*obj.f2xout(end);
            
            % y axis
            obj.yout(:,end+1) = obj.Gy.C*obj.y(:,end) + obj.Gy.D*forces(2);
            obj.y(:,end+1) = obj.Gy.A*obj.y(:,end) + obj.Gy.B*forces(2);
            % y second der
            obj.f2yout(end+1) = obj.filter.C*obj.f2ys(end)+obj.filter.D*obj.yout(2,end);
            obj.f2ys(end+1) = obj.filter.A*obj.f2ys(end)+obj.filter.B*obj.yout(2,end);
            % y third der
            obj.f3yout(1,end+1) = obj.filter.C*obj.f3ys(end)+obj.filter.D*obj.f2yout(end);
            obj.f3ys(1,end+1) = obj.filter.A*obj.f3ys(end)+obj.filter.B*obj.f2yout(end);
            
            % z axis
            obj.zout(:,end+1) = obj.Gz.C*obj.z(:,end) + obj.Gz.D*forces(3);
            obj.z(:,end+1) = obj.Gz.A*obj.z(:,end) + obj.Gz.B*forces(3);  
            % z second der
            obj.f2zout(end+1) = obj.filter.C*obj.f2zs(end)+obj.filter.D*obj.zout(2,end);
            obj.f2zs(end+1) = obj.filter.A*obj.f2zs(end)+obj.filter.B*obj.zout(2,end);
            % z third der
            obj.f3zout(1,end+1) = obj.filter.C*obj.f3zs(end)+obj.filter.D*obj.f2zout(end);
            obj.f3zs(1,end+1) = obj.filter.A*obj.f3zs(end)+obj.filter.B*obj.f2zout(end);
            
            % yaw axis
            obj.yawout(:,end+1) = obj.Gyaw.C*obj.yaw(:,end) + obj.Gyaw.D*forces(4);
            obj.yaw(:,end+1) = obj.Gyaw.A*obj.yaw(:,end) + obj.Gyaw.B*forces(4);  
            % yaw second der
            obj.f2yawout(end+1) = obj.filter.C*obj.f2yaws(end)+obj.filter.D*obj.yawout(2,end);
            obj.f2yaws(end+1) = obj.filter.A*obj.f2yaws(end)+obj.filter.B*obj.yawout(2,end);
            % yaw third der
            obj.f3yawout(1,end+1) = obj.filter.C*obj.f3yaws(end)+obj.filter.D*obj.f2yawout(end);
            obj.f3yaws(1,end+1) = obj.filter.A*obj.f3yaws(end)+obj.filter.B*obj.f2yawout(end);
            
            
        end
        
        function ref = getRef(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            ref(1,1) = obj.xout(1,end);
            ref(2,1) = obj.yout(1,end);
            ref(3,1) = obj.zout(1,end);
            ref(4,1) = obj.yawout(1,end);
            
        end
        
          function refd = getRefd(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            refd(1,1) = obj.xout(2,end);
            refd(2,1) = obj.yout(2,end);
            refd(3,1) = obj.zout(2,end);
            refd(4,1) = obj.yawout(2,end);
            
          end
        
          function refdd = getRefdd(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            refdd(1,1) = obj.xout(3,end);
            refdd(2,1) = obj.yout(3,end);
            refdd(3,1) = obj.zout(3,end);
            refdd(4,1) = obj.yawout(3,end);
            
          end  
          
          function ref2dotFilter = getRefddFilter(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            ref2dotFilter(1,1) = obj.f2xout(end);
            ref2dotFilter(2,1) = obj.f2yout(end);
            ref2dotFilter(3,1) = obj.f2zout(end);
            ref2dotFilter(4,1) = obj.f2yawout(end);
          end  
          
           function ref3dotFilter = getRefdddFilter(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            ref3dotFilter(1,1) = obj.f3xout(end);
            ref3dotFilter(2,1) = obj.f3yout(end);
            ref3dotFilter(3,1) = obj.f3zout(end);
            ref3dotFilter(4,1) = obj.f3yawout(end);
            
           end
          
           function times = getTimes(obj)
            times = obj.Ts*( 0:size(obj.xout,2)-1 );
           end
           
           function pos = getPositions(obj)
                pos=[ obj.xout(1,:); obj.yout(1,:); obj.zout(1,:);  obj.yawout(1,:) ];
           end
           
           function der1 = getDer1(obj)
                der1=[ obj.xout(2,:); obj.yout(2,:); obj.zout(2,:);  obj.yawout(2,:) ];
           end
           
           function der2 = getDer2(obj)
                der2=[ obj.xout(3,:); obj.yout(3,:); obj.zout(3,:);   obj.yawout(3,:)];
           end
           
           function der2filterd = getDer2Filtered(obj)
                der2filterd =[ obj.f2xout; obj.f2yout; obj.f2zout; obj.f2yawout  ];
           end
    
           function der3filterd = getDer3Filtered(obj)
                der3filterd =[ obj.f3xout; obj.f3yout; obj.f3zout; obj.f3yawout ];
           end
           
          function fig = plotOutputs(obj)
          
              times = obj.Ts*(0:1:size(obj.xout,2)-1);
              fig=figure();
              fig.Name='Admittance Controller';
              subplot(1,4,1);
              stairs(times,obj.xout(1,:));grid on; hold on;
              stairs(times,obj.xout(2,:));
              stairs(times,obj.xout(3,:));
              stairs(times,obj.f2xout(:));
              stairs(times,obj.f3xout(:));
              legend('x','xdot','xddot','x2dotFilter','x3dotFilter');
              
              subplot(1,4,2);
              stairs(times,obj.yout(1,:));grid on; hold on;
              stairs(times,obj.yout(2,:));
              stairs(times,obj.yout(3,:));
              stairs(times,obj.f2yout(:));
              stairs(times,obj.f3yout(:));
              legend('y','ydot','yddot','y2dotFilter','y3dotFilter');
              
              subplot(1,4,3);
              stairs(times,obj.zout(1,:));grid on; hold on;
              stairs(times,obj.zout(2,:));
              stairs(times,obj.zout(3,:));
              stairs(times,obj.f2zout(:));
              stairs(times,obj.f3zout(:));
              legend('z','zdot','zddot','z2dotFilter','z3dotFilter');
              
              subplot(1,4,4);
              stairs(times,obj.yawout(1,:));grid on; hold on;
              stairs(times,obj.yawout(2,:));
              stairs(times,obj.yawout(3,:));
              stairs(times,obj.f2yawout(:));
              stairs(times,obj.f3yawout(:));
              legend('yaw','yawdot','yawddot','yaw2dotFilter','yaw3dotFilter');
          end 
          
    end 

end

