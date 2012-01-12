function accel = raw2accel(raw)
% function to convert the raw accelerometer data into m/s^2
%   in the robot frame

global CONFIG

x = (raw(1,:) - CONFIG.imu.accel.bias(1)) * (CONFIG.imu.accel.vref(1)/1023) / CONFIG.imu.accel.sensitivity(1);
y = (raw(2,:) - CONFIG.imu.accel.bias(2)) * (CONFIG.imu.accel.vref(2)/1023) / CONFIG.imu.accel.sensitivity(2);
z = (raw(3,:) - CONFIG.imu.accel.bias(3)) * (CONFIG.imu.accel.vref(3)/1023) / CONFIG.imu.accel.sensitivity(3);

accel = [x;y;-z];


