function h = robot()

global CONFIG
global KINECTLOG

%% configuration parameters
h.robotRadius = CONFIG.robot.robotRadius;
h.robotRadiusFudge = CONFIG.robot.robotRadiusFudge;
h.wheel.r = CONFIG.robot.wheel.r;

h.encoders.ticksPerRev = CONFIG.encoders.ticksPerRev;
h.encoders.metersPerTic = 2*pi * h.wheel.r / h.encoders.ticksPerRev; 

%% robot pose 
h.pose.ukf = ukf_quaternion(CONFIG.ukf.egyro, CONFIG.ukf.eaccel);
% position
h.pose.p = [0; 0; 0];
% orientation (quaternion)
h.pose.q = [1; 0; 0; 0];

%% lidar processing struct
h.lidar = lidar(CONFIG.lidar.angles, CONFIG.lidar.rmin, CONFIG.lidar.rmax);

%% kinect processing struct
if CONFIG.use_kinect
  h.kinect = kinect(KINECTLOG);
end

%% transformations
% lidar to body transform tranform (static transform)
h.Tlidar = trans([0.1, 0, 0])*rotz(0)*roty(0)*rotx(0);
% kinect to body transform tranform (static transform)
h.Tkinect = h.Tlidar * trans([0.18, 0.005, 0.36])*rotz(0.021)*roty(0.36)*rotx(0);

%% function definitions
h.lidar2body = @lidar2body;
h.body2world = @body2world;
h.filter_lidar = @filter_lidar;
h.adjust_pose = @adjust_pose;
h.motion_model = @motion_model;
h.update_enc = @update_enc;
h.update_imu = @update_imu;
h.get_pose = @get_pose;
h.get = @get;
h.set = @set;

  function [Y ivalid] = lidar2body(r)
  % filters the lidar data and returns the cartesian 
  % coordinates in the 2D body frame

    % to cartesian x,y in lidar frame
    [x y] = h.lidar.raw2cart(r);
    ivalid = h.lidar.valid_ranges(r);
    ivalid = ivalid(:)';

    % x,y,z in lidar frame
    X = [x(:)'; y(:)'; zeros(1,length(x))];

    % to body frame
    Y = h.Tlidar * [X; ones(1,length(x))];
  end

  function ivalid = filter_lidar(X, thres)
  % finds the indices of valid lidar points
  %   X - lidar readings in body frame
    % augment X if needed
    if size(X,1) == 3
      X = [X;ones(1,size(X,2))];
    end 
    % use roll and pitch to transform to 'world'
    rpy = quat2rpy(h.pose.q);
    Y = roty(rpy(2))*rotx(rpy(1))*X;
    ivalid = abs(Y(3,:)) < thres;
  end

  function Y = body2world(X)
  % converts base points X to the world frame
    % augment X if needed
    if size(X,1) == 3
      X = [X;ones(1,size(X,2))];
    end
    rpy = quat2rpy(h.pose.q);
    T = trans(h.pose.p)*rotz(rpy(3))*roty(rpy(2))*rotx(rpy(1));
    Y = T*X;
  end

  function [dp dyaw sdp] = motion_model(rr, rl)
  % motion dynamics based on the encoder readings

    rc = rr * h.encoders.metersPerTic;
    lc = rl * h.encoders.metersPerTic;
    rpy = quat2rpy(h.pose.q);
    yaw = rpy(3);

    vdt = mean([rc, lc]);

    %the fudge factor scales the angular change due to slippage
    %TODO: this will also affect vdt!!
    wdt = (rc - lc)/(2 * h.robotRadius * h.robotRadiusFudge);

    %calculate the change in position
    if (abs(wdt) > 0.001)
      dx = -vdt/wdt * sin(yaw) + vdt/wdt * sin(yaw + wdt);
      dy = vdt/wdt * cos(yaw) - vdt/wdt * cos(yaw + wdt);
      dyaw = wdt;
    else
      dx = vdt * cos(yaw);
      dy = vdt * sin(yaw);
      dyaw = wdt;
    end

    % TODO: scale based on robot pose (estimating a 2D world)
    rpy = quat2rpy(h.pose.q);
    %R = rpy2rot([rpy(1:2); 0]);
    R = roty(rpy(2))*rotx(rpy(1));
    nxy = R * [dx;dy;0;1];

    dp = [dx;dy];
    sdp = [nxy(1);nxy(2);];
    dyaw = dyaw;
  end

  function adjust_pose(dx, dy, dyaw)
    rpy = quat2rpy(h.pose.q);
    rpy(3) = rpy(3) + dyaw;
    h.pose.q = rpy2quat(rpy);

    h.pose.p(1) = h.pose.p(1) + dx;
    h.pose.p(2) = h.pose.p(2) + dy;

    h.pose.ukf.set_state(h.pose.q);
  end

  function [dp dyaw sdp] = update_enc(rr, rl)
  % update the robot pose from the motion model
    [dp dyaw sdp] = h.motion_model(rr, rl);
    %fprintf('proj move:\ndp: %0.2f, %0.2f\nsp: %0.2f, %0.2f\n', dp(1), dp(2), sdp(1), sdp(2));

    % position
    h.pose.p(1:2) = h.pose.p(1:2) + sdp;

    % orientation 
    rpy = quat2rpy(h.pose.q);
    rpy(3) = rpy(3) + dyaw;
    h.pose.q = rpy2quat(rpy);
  end

  function q = update_imu(accel, gyro, dt)
    q = h.pose.ukf.update(accel, gyro, dt);
    % do not use gyro yaw, use encoder
    rpy0 = quat2rpy(h.pose.q);
    rpy1 = quat2rpy(q);
    h.pose.q = rpy2quat([rpy1(1);rpy1(2);rpy0(3)]);
  end

  function [p q] = get_pose()
  % returns the current pose of the robot
  %   p - position of the robot
  %   q - quaternion orientation
    p = h.pose.p;
    q = h.pose.q;
  end

  function ret = get(field)
    ret = h.(field);
  end

  function set(field, val)
    h.(field) = val;
  end

end
