function h = lidar_plot(varargin)

if nargin < 1
  t = 'lidar';
  a = ([-135:.25:135]') * pi/180;
elseif nargin < 2
  t = varargin{1};
  a = [-135:.25:135]' * pi/180;
elseif nargin < 3
  t = varargin{1};
  a = varargin{2};
elseif nargin < 4
  t = varargin{1};
  a = varargin{2};
  axes(varargin{3});
end

h.angles = a;
h.ca = cos(a);
h.sa = sin(a);
h.init = @init;
h.update = @update;

init(t);


  function init(t)
    h.lidar = [];

    % lidar display
    h.lidarAxes = gca;
    title(t);
    xlabel('');
    ylabel('');
    axis([-6, 12, -10, 10]);
    set(h.lidarAxes, 'CameraUpVector', [1, 0, 0]);
    grid on;
    hold on;

    % initialize lidar plot
    h.lidar.pp = polar(h.angles, zeros(size(h.angles)), '.b');
    [x, y] = pol2cart([-135; 0; 135] * pi/180, [10; 0; 10]);
    h.lidar.fovPlot = plot(x, y, '--k');
    h.lidar.robotPlot = plot(0, 0, '^g', 'MarkerSize', 3, 'LineWidth', 4);

  end

  function update(d)
    x = d .* h.ca;
    y = d .* h.sa;
    set(h.lidar.pp, 'XData', x, 'YData', y);
  end

end

