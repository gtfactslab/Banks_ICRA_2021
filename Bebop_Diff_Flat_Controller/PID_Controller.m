classdef PID_Controller < handle
   % pid controller for diff flat control of bebop 
   properties
       K_p = 0;
       K_i = 0;
       K_d = 0; 
       T_f = 0;
       prev_error = 0;
       min_integrator = -500;
       max_integrator = 500;
       integral = 0;
       prev_time = 0;
       prev_output = 0;
             
   end
   
   methods
       function [obj] = PID_Controller(val_1, val_2, val_3, val_4)
           disp("constructor")
          switch nargin
              case 3
               obj.K_p = val_1;
               obj.K_i = val_2;
               obj.K_d = val_3;
              case 4
               obj.K_p = val_1;
               obj.K_i = val_2;
               obj.K_d = val_3;
               obj.T_f = val_4;
          end
           
       end
       
       
       function obj = reset(obj)
           obj.prev_error = 0;
           obj.integral = 0;
           obj.prev_time = rostime("now");           
              
       end
       
       function [output, obj] = update(obj, target_val, val)
           t = rostime("now");
           if obj.prev_time == 0
               obj.prev_time = t;
               disp("update time")
           end
           dt = seconds(t - obj.prev_time);
           error = target_val - val;
           prop = obj.K_p*error;
           der = 0;
           inter = obj.integral + error*dt;
           inter_obj = max(min(inter, obj.max_integrator), obj.min_integrator);
           inter = obj.K_i*inter_obj;
           if dt > 0
               der = obj.K_d * (error - obj.prev_error) / dt;
           end
           
           
           output = prop + inter + der;
           %filtered output
           if obj.T_f > 0
            output = obj.prev_output + (dt / obj.T_f)*(output - obj.prev_output);
           end
           obj.prev_error = error;
           obj.integral = inter_obj;
           obj.prev_time = t;   
           obj.prev_output = output;
       end
       
       
   end
    
    
    
end