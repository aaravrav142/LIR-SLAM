function h = depth_plot(varargin)
% graphical visualization of the program

h.init = @init;
h.update = @update;

init();


  function init()
    h.pose = [];

    % pose display
    h.poseAxes = gca;
    title('Depth Plot');
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    axis([-20, 20, -10, 10, -1, 10]);
    grid on;
    hold on;

    % pose display box
    lx = 0.6/2.0;
    ly = 0.4/2.0;
    lz = 0.2/2.0;
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

    % initialize depth plot
    h.n = 1081;
    zrs = zeros(1, h.n);
    h.depth = plot3(zrs, zrs, zrs, '.b');

    % filtered points
    h.n = 1081;
    h.z3 = zeros(3, h.n);
    h.filtered = plot3(zrs, zrs, zrs, '.r');
  end

  function update(R, X, ivalid)
    xp = R*h.dispBox.x;

    set(h.box(1), 'XData', xp(1,h.dispBox.itop), 'YDATA', xp(2,h.dispBox.itop), 'ZDATA', xp(3,h.dispBox.itop));
    set(h.box(2), 'XData', xp(1,h.dispBox.ibottom), 'YDATA', xp(2,h.dispBox.ibottom), 'ZDATA', xp(3,h.dispBox.ibottom));
    
    set(h.front, 'XData', xp(1,h.dispBox.ifront), 'YDATA', xp(2,h.dispBox.ifront), 'ZDATA', xp(3,h.dispBox.ifront));
    set(h.back, 'XData', xp(1,h.dispBox.iback), 'YDATA', xp(2,h.dispBox.iback), 'ZDATA', xp(3,h.dispBox.iback));

    set(h.depth, 'XData', X(1,:), 'YData', X(2,:), 'ZData', X(3,:)); 
    fpts = h.z3;
    fpts(:,ivalid) = X(:,ivalid);
    set(h.filtered, 'XData', fpts(1,:), 'YData', fpts(2,:), 'ZData', fpts(3,:)); 
  end

end

