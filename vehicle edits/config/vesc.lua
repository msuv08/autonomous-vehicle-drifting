serial_port = "/dev/ttyACM0";
-- erpm (electrical rpm) = speed_to_erpm_gain * speed (meters / second) +
--   speed_to_erpm_offset
-- for offset=0. speed_to_erpm_gain =
--   num_motor_poles*60/circumference_wheel_in_meters
speed_to_erpm_gain = 5356; -- arrma 3800: 17T pinion gear + P48 T83 spur gear
speed_to_erpm_offset = 180.0; -- should be between 160-200
-- servo value (0 to 1) =  steering_angle_to_servo_gain * steering angle
--    (radians) + steering_angle_to_servo_offset
steering_angle_to_servo_gain = -0.9015;
steering_angle_to_servo_offset = 0.57; -- should be between 0.4-0.6
-- erpm_speed_limit = 14000; -- 3250
erpm_speed_limit = 22000; -- 3250
servo_min = 0.05;
servo_max = 0.95;

-- Taken from the manufacturer: 
-- https://traxxas.com/products/models/electric/ford-fiesta-st-rally?t=specs
wheelbase = 0.324;

max_acceleration = 6.0; -- m/s^2
max_deceleration = 6.0; -- m/s^2

joystick_normal_speed = 1.0; -- m/s
joystick_turbo_speed = 5.0; -- m/s

