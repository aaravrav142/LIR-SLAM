function h = filter_gui(varargin)

if nargin < 1
  fig = init_figure('Filter Display', false, true, 'w');
end

h.fig = fig;

h.init = @init;
h.update = @update;

init();


  function init()

    % pose display
    h.accel = accel_plot('Accelerometer', subplot(2,2,1));
    h.gyro = gyro_plot('Gyroscope', subplot(2,2,2));
    h.pose = pose_plot('UKF Estimate', subplot(2,2,3));
    h.vicon = pose_plot('Ground Truth', subplot(2,2,4));

  end

  function update(accel, angvel, R_est, R_gt)
    h.accel.update(accel); 
    h.gyro.update(angvel);
    h.pose.update(R_est); 
    h.vicon.update(R_gt); 
  end

end

