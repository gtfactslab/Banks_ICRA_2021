classdef PD_Controller
   % pd controller for diff flat control of bebop 
   properties
       K_p = 1000;
       K_d = 100; 
       prev_error = 0;
       prev_time  = rostime("now");
             
   end
   
   methods
       function obj = reset(obj)
           obj.prev_error = 0;
           obj.prev_time = 0;                     
       end
       
       function [output, obj] = update(obj, target_val, val)
           t = rostime("now");
           dt = t - obj.prev_time;
           error = target_val - val;
           prop = obj.K_p*error;
           der = 0;
           if dt > 0
               der = obj.K_d * (error - obj.prev_error) / dt;
           end
           
           output = prop + der;
           obj.prev_error = error;
           obj.prev_time = t;   
       end
       
   end
   
end