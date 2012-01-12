function h = pose_plot(varargin)
% graphical visualization of the program

if nargin < 1
  t = 'Pose';
elseif nargin < 2
  t = varargin{1};
elseif nargin < 3
  t = varargin{1};
  axes(varargin{2});
end

h.init = @init;
h.update = @update;

init(t);


  function init(t)
    h.pose = [];

    % pose display
    h.poseAxes = gca;
    title(t);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    axis([-2, 2, -2, 2, -2, 2]);
    grid on;
    hold on;

    % pose display box
    lx = 1.50;
    ly = 0.75;
    lz = 0.50;
    h.dispBox.lx = lx;
    h.dispBox.ly = ly;
    h.dispBox.lz = lz;
    h.dispBox.x = [ +lx -lx +lx -lx +lx -lx +lx -lx; 
                    +ly +ly -ly -ly +ly +ly -ly -ly; 
                    +lz +lz +lz +lz -lz -lz -lz -lz];
    h.dispBox.ifront =  [1 3 7 5 1];
    h.dispBox.iback =   [2 4 8 6 2];
    h.dispBox.itop =    [1 2 4 3 1];
    h.dispBox.ibottom = [5 6 8 7 5];

    % initialize pose plot
    xp = eye(3)*h.dispBox.x;
    h.box = plot3(xp(1,h.dispBox.itop), xp(2,h.dispBox.itop), xp(3,h.dispBox.itop), 'k-', ...
                    xp(1,h.dispBox.ibottom), xp(2,h.dispBox.ibottom), xp(3,h.dispBox.ibottom), 'k-');
    h.front = patch(xp(1,h.dispBox.ifront), xp(2,h.dispBox.ifront), xp(3,h.dispBox.ifront), 'b');
    h.back = patch(xp(1,h.dispBox.iback), xp(2,h.dispBox.iback), xp(3,h.dispBox.iback), 'r');

  end

  function update(R)
    xp = R*h.dispBox.x;

    set(h.box(1), 'XData', xp(1,h.dispBox.itop), 'YDATA', xp(2,h.dispBox.itop), 'ZDATA', xp(3,h.dispBox.itop));
    set(h.box(2), 'XData', xp(1,h.dispBox.ibottom), 'YDATA', xp(2,h.dispBox.ibottom), 'ZDATA', xp(3,h.dispBox.ibottom));
    
    set(h.front, 'XData', xp(1,h.dispBox.ifront), 'YDATA', xp(2,h.dispBox.ifront), 'ZDATA', xp(3,h.dispBox.ifront));
    set(h.back, 'XData', xp(1,h.dispBox.iback), 'YDATA', xp(2,h.dispBox.iback), 'ZDATA', xp(3,h.dispBox.iback));
  end

end

