function gyro = raw2gyro(raw)
% function to convert the raw gyroscope data into rad/s
%   in the robot frame

global CONFIG

x = (raw(1,:) - CONFIG.imu.gyro.bias(1)) * (CONFIG.imu.gyro.vref(1)/1023) / CONFIG.imu.gyro.sensitivity(1);
y = (raw(2,:) - CONFIG.imu.gyro.bias(2)) * (CONFIG.imu.gyro.vref(2)/1023) / CONFIG.imu.gyro.sensitivity(2);
z = (raw(3,:) - CONFIG.imu.gyro.bias(3)) * (CONFIG.imu.gyro.vref(3)/1023) / CONFIG.imu.gyro.sensitivity(3);

gyro = [x;y;-z];

% zero out any values below thres
vi = abs(gyro) < .25;
gyro(vi) = gyro(vi) / 2;

