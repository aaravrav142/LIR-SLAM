% main script to test run the kalman filter
config
global CONFIG
global KINECTLOG

% load log data
imu = load(sprintf('imuRaw%d', CONFIG.lognum));
vicon = load(sprintf('viconRot%d', CONFIG.lognum));

sts = size(imu.ts);

% ukf pose filter
ukf = ukf_quaternion([0.01; 0.05], [.002; 0.001]);

% create low pass filters for accelerometer and gyroscope
accelFilter = iir_filter(.05, raw2accel(imu.vals(1:3,1)));
gyroFilter = iir_filter(.05, raw2gyro(imu.vals([5, 6, 4],1)));

% vicon log index
vi = 1;
R_gt = eye(3);

% pose/raw data gui
g = filter_gui();

t0 = time();
for i = 2:sts(2)
  tu = time();
  dt = imu.ts(i) - imu.ts(i-1);

  % update accel and gyro readings
  accel = accelFilter.update(raw2accel(imu.vals(1:3,i)));
  gyro = gyroFilter.update(raw2gyro(imu.vals([5, 6, 4],i)));

  % update robot pose based on imu
  q = ukf.update(accel, gyro, dt);
  R_est = quat2rot(q);

  % get ground truth
  if vicon.ts(vi) <= imu.ts(i)
    R_gt = vicon.rots(:,:,vi);
    vi = vi + 1;
  end


  % update pose/data gui
  g.update(accel, gyro, R_est, R_gt);

  pause(dt - (time() - tu));
end

t1 = time();

