function h = accel_plot(varargin)
% graphical visualization of the program

if nargin < 1
  t = 'Accelerometer Readings';
elseif nargin < 2
  t = varargin{1};
elseif nargin < 3
  t = varargin{1};
  axes(varargin{2});
end

h.init = @init;
h.update = @update;
h.plot_pose = @plot_pose;

h.n = 300;

init(t);


  function init(t)
    
    h.pose = [];

    % create axes
    % acceleration display
    h.accelAxes = gca;
    title(t);
    xlabel('t [time step]');
    ylabel('accel [m/s^2]');
    axis([0, h.n+1, 0, 1000]); 
    axis([0, h.n+1, -11, 11]); 
    grid on;
    hold on;

    % initialize accel plot
    h.data = zeros(3,h.n); 
    h.xPlot = plot([1:h.n], h.data(1,:), 'r-');
    h.yPlot = plot([1:h.n], h.data(2,:), 'g-');
    h.zPlot = plot([1:h.n], h.data(3,:), 'b-');
    
    %{
    % gyro display
    GUIDATA.gyroAxes = subplot(2,2,2);
    title('Raw Gyro');
    xlabel('t [time step]');
    ylabel('Y [rad/s]');
    axis([0, CONFIG.disp.ngyro+1, -pi, pi]); 
    grid on;
    hold on;

    % initialize gyro plot
    GUIDATA.gyro.data = zeros(3,CONFIG.disp.ngyro); 
    GUIDATA.gyro.xPlot = plot([1:CONFIG.disp.ngyro], GUIDATA.gyro.data(1,:), 'r-');
    GUIDATA.gyro.yPlot = plot([1:CONFIG.disp.ngyro], GUIDATA.gyro.data(2,:), 'g-');
    GUIDATA.gyro.zPlot = plot([1:CONFIG.disp.ngyro], GUIDATA.gyro.data(3,:), 'b-');
    %}

  end

  function update(xyz)
    h.data(:,1:end-1) = h.data(:,2:end);
    h.data(:,end) = xyz(:);

    set(h.xPlot, 'YData', h.data(1,:));
    set(h.yPlot, 'YData', h.data(2,:));
    set(h.zPlot, 'YData', h.data(3,:));
  end

end

