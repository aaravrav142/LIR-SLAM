function h = data_gui(varargin)
% graphical visualization of the program

if nargin < 1
  fig = init_figure('Data Display', false, true, 'w');
end

h.fig = fig;

h.init = @init;
h.update = @update;

init();


  function init()

    % pose display
    h.accel = accel_plot('Accelerometer', subplot(2,2,1));
    h.gyro = gyro_plot('Gyroscope', subplot(2,2,2));
    h.pose = pose_plot('Robot Pose', subplot(2,2,3));
    h.lidar = lidar_plot('Lidar', [-135:.25:135]'*pi/180, subplot(2,2,4));

  end

  function update(accel, angvel, R, d)
    h.accel.update(accel); 
    h.gyro.update(angvel);
    h.pose.update(R); 
    h.lidar.update(d); 
  end

end

