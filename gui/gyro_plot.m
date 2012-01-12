function h = accel_plot(varargin)
% graphical visualization of the program

if nargin < 1
  t = 'Gyroscope Readings';
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

    % gyro display
    h.Axes = gca;
    title(t);
    xlabel('t [time step]');
    ylabel('angular velocity [rad/s]');
    axis([0, h.n+1, -pi, pi]); 
    grid on;
    hold on;

    % initialize gyro plot
    h.data = zeros(3,h.n); 
    h.xPlot = plot([1:h.n], h.data(1,:), 'r-');
    h.yPlot = plot([1:h.n], h.data(2,:), 'g-');
    h.zPlot = plot([1:h.n], h.data(3,:), 'b-');

  end

  function update(xyz)
    h.data(:,1:end-1) = h.data(:,2:end);
    h.data(:,end) = xyz(:);

    set(h.xPlot, 'YData', h.data(1,:));
    set(h.yPlot, 'YData', h.data(2,:));
    set(h.zPlot, 'YData', h.data(3,:));
  end

end

