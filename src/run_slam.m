% main script to run the slam algorithm
config
global CONFIG
global KINECTLOG

% load log data
imu = load(sprintf('imuRaw%d', CONFIG.lognum));
lidar = load(sprintf('Hokuyo%d', CONFIG.lognum));
lidar = lidar.Hokuyo0;
enc = load(sprintf('Encoders%d', CONFIG.lognum));
enc = enc.Encoders;
if CONFIG.use_kinect
  KINECTLOG = load(sprintf('kinect%d', CONFIG.lognum));
end

sts = size(imu.ts);

% lidar data
d = lidar.ranges(:,1);

% create low pass filters for accelerometer and gyroscope
accelFilter = iir_filter(.01, raw2accel(imu.vals(1:3,1)));
gyroFilter = iir_filter(.05, raw2gyro(imu.vals([5, 6, 4],1)));

% robot struct
r = robot();
rstate = ramp_state();


% map struct
m = map(CONFIG.map.xmin, CONFIG.map.xmax, CONFIG.map.ymin, CONFIG.map.ymax, CONFIG.map.res);

% pose/raw data gui
g = data_gui();

% occupancy grid map gui
init_figure('Occupancy Grid', false, true, 'w');
mg = map_plot(m, gca);
if CONFIG.use_kinect
  % height map gui
  init_figure('Height Map', false, true, 'w');
  mhg = map_plot(m, gca);
  % floor map gui
  init_figure('Floor Map', false, true, 'w');
  mfg = map_plot(m, gca);
end


% initialize log indices
ei = 1;
ki = 1;
li = 1;

t0 = time();
for i = 2:sts(2)
  tu = time();
  dt = imu.ts(i) - imu.ts(i-1);

  % update accel and gyro readings
  accel = accelFilter.update(raw2accel(imu.vals(1:3,i)));
  gyro = gyroFilter.update(raw2gyro(imu.vals([5, 6, 4],i)));

  % update robot pose based on imu
  x_k = r.update_imu(accel, gyro, dt);

  % encoder update 
  if imu.ts(i) > enc.ts(ei)
    [dp dyaw sdp] = r.update_enc(enc.counts(3, ei), enc.counts(4, ei));
    ei = ei + 1;
  end

  % lidar update
  if enc.ts(ei) > lidar.ts(li)
    d = lidar.ranges(:, li);
    li = li + 1;

    [p q] = r.get_pose();
    rpy = quat2rpy(q);

    % lidar to 2d
    [Y ivalid] = r.lidar2body(d);
    % filter out points based on z-coord
    ivalid = ivalid & r.filter_lidar(Y, .2);

    % find map correlation
    pose = r.get('pose');
    [dx dy dyaw score Y_] = m.max_correlation(Y(1:3,ivalid), pose.p, pose.q);
    %fprintf('adjusting:\ndx: %0.3f\ndy: %0.3f\nscore: %0.3f\n', dx, dy, score);

    if rstate.update(-rpy(2)) == 0
      r.adjust_pose(dx, dy, dyaw);

      % update map
      m.update(Y_, CONFIG.map.dm);
    else
      % only update yaw
      r.adjust_pose(0.0, 0.0, dyaw);
    end
      
    if rem(ei, 10) == 0
      % update map gui
      mg.update_map(m.get_map());

      pose = r.get('pose');
      mg.update_robot(pose);
    end
  end

  % kinect update
  if CONFIG.use_kinect && enc.ts(ei) > r.kinect.depth_ts(ki) 

    if rstate.get_state() == 0

      % transform to body
      disparity = r.kinect.disparity(ki);
      [kY kivalid] = r.kinect.depth(disparity);

      % update floor estimate
      [A ifloor] = r.kinect.update_floor(disparity);
      fivalid = kivalid & ifloor' & (kY(1,:).^2+kY(2,:).^2 < 16);
      
      % transform to world
      [p q] = r.get_pose();
      rpy = quat2rpy(q);

      T = trans([p(1) p(2) 0])*rotz(rpy(3));
      fxyz = T*kY(:,fivalid);
      m.update(fxyz(1:3,:), CONFIG.map.dg);

      % floor color
      drgb = r.kinect.merge(disparity, r.kinect.rgb(r.kinect.rgb_ind(ki)));
      drgbr = drgb(:,:,1);
      drgbr = drgbr(:)';
      drgbg = drgb(:,:,2);
      drgbg = drgbg(:)';
      drgbb = drgb(:,:,3);
      drgbb = drgbb(:)';

      rgbivalid = fivalid & (drgbr ~= 0 | drgbg ~= 0 | drgbb ~= 0); 
      fxyzrgb = T*kY(:,rgbivalid);
      m.update_floor(fxyzrgb(1:3,:), drgbr(rgbivalid), drgbg(rgbivalid), drgbb(rgbivalid));

      hivalid = kivalid & (kY(1,:).^2+kY(2,:).^2 < 9);
      hxyz = T*kY(:,hivalid);
      m.update_height(hxyz(1:3,:));

      % update cost map based on height
      hmap = m.get_height_map();
      hmap(hmap==-1) = -0.5;
      rdmap = diff(hmap);
      cdmap = diff(hmap')';
      dmap = abs(rdmap(1:1600, 1:1600) + cdmap(1:1600,1:1600));

      if rem(ki, 10) == 0
        pose = r.get('pose');
        mhg.update_robot(pose);
        mhg.update_map(m.get_height_map());
        mfg.update_robot(pose);
        mfg.update_map_ndt(m.get_floor_map());
      end
    end

    ki = ki + 1;
  end

  % update pose/data gui
  [p q] = r.get_pose();
  R = quat2rot(q);
  g.update(accel, gyro, R, d);

  pause(dt - (time() - tu));
end

t1 = time();

