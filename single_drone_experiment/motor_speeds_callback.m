function [motor_speed_vect] = motor_speeds_callback(speeds)
    motor_rpms = speeds.LatestMessage;
    
    motor_speed_vect = [motor_rpms.M1, motor_rpms.M2, motor_rpms.M3, ...
        motor_rpms.M4];
end
