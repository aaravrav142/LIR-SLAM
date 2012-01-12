function h = map_plot(m, ax)

h.m = m;
h.init = @init;
h.update_map = @update_map;
h.update_map_ndt = @update_map_ndt;
h.update_robot = @update_robot;

init();

  function init()
    % map display
    h.mapAxes = gca;
    title('Map');
    xlabel('');
    ylabel('');

    % initialize map plot
    h.img = imagesc(h.m.map);
    set(h.mapAxes, 'YDir', 'normal');
    set(h.mapAxes, 'xtick', []);
    set(h.mapAxes, 'ytick', []);
    hold on;

    % initialize robot plot
    h.robot.T = eye(4);
    h.robot.tri = [ -.2,  .5,  -.2, -.2;
                    .2, 0, -.2, .2;
                     0,   0,   0,   0;
                     1,   1,   1,   1];
    Y = h.robot.T * h.robot.tri;
    [xi yi] = h.m.meters2cells(Y(1,:), Y(2,:));
    h.robotPlot = patch(xi, yi, '-w');
  end

  function update_robot(pose)
    h.robot.T(1:3,1:3) = quat2rot(pose.q);
    h.robot.T(1:2,4) = [pose.p(1); pose.p(2)];
    Y = h.robot.T * h.robot.tri;
    [xi yi] = h.m.meters2cells(Y(1,:), Y(2,:));
    set(h.robotPlot, 'XData', xi, 'YData', yi);
  end

  function update_map(map)
    set(h.img, 'CData', map');
  end

  function update_map_ndt(m)
    m(:,:,1) = m(:,:,1)';
    m(:,:,2) = m(:,:,2)';
    m(:,:,3) = m(:,:,3)';
    set(h.img, 'CData', double(m)./255.0);
  end
end

