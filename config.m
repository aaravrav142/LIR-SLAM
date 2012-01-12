global CONFIG

CONFIG.lognum = 20;

CONFIG.map.xmin = -40;
CONFIG.map.xmax =  40;
CONFIG.map.ymin = -40;
CONFIG.map.ymax =  40;
CONFIG.map.res = 0.05;
CONFIG.map.dm = 1;
CONFIG.map.dg = -2;

% lidar properties for horizontal Hokuyo (from data sheet)
CONFIG.lidar.angles = [-135:0.25:135]*pi/180;
CONFIG.lidar.rmin = 0.20;
CONFIG.lidar.rmax = 60.0;

% robot wheel properties
CONFIG.robot.robotRadius = 0.25692;
CONFIG.robot.robotRadiusFudge = 1.50;
CONFIG.robot.wheel.r = .254/2.0;

% wheel encoder values
CONFIG.encoders.ticksPerRev = 360;

% GUI display 
CONFIG.disp.naccel = 200;
CONFIG.disp.ngyro = 200;

% accelerometer properties
CONFIG.imu.accel.vref = [3300; 3300; 3300];
CONFIG.imu.accel.bias = [511; 501; 501];
CONFIG.imu.accel.sensitivity = [342; 401; 342] .* (1/9.8);

% gyro properties
CONFIG.imu.gyro.vref = [3300; 3300; 3300];
CONFIG.imu.gyro.bias = [373; 374; 369];
CONFIG.imu.gyro.sensitivity = [3.43; 3.53; 3.30] * (180.0/pi);

% ukf pose filter parameters
CONFIG.ukf.eaccel = [.005; 0.001];
CONFIG.ukf.egyro = [0.01; 0.02];

% flag indicating if the kinect data should be used
%   the kinect data is used to create a floor map
%   and height map
CONFIG.use_kinect = false;
