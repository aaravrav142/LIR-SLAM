function h = map(xmin, xmax, ymin, ymax, res)
% map class to handle creation and updating of the display

h.xmin = xmin;
h.xmax = xmax;
h.ymin = ymin;
h.ymax = ymax;
h.res = res;

h.sx = ceil((h.xmax - h.xmin) / h.res + 1);
h.sy = ceil((h.ymax - h.ymin) / h.res + 1);
h.map = zeros(h.sx, h.sy, 'int8');
h.height = zeros(h.sx, h.sy)-1;

h.flr.r = zeros(h.sx, h.sy);
h.flr.g = zeros(h.sx, h.sy);
h.flr.b = zeros(h.sx, h.sy);
h.flr.rgb = zeros(h.sx, h.sy, 3);

% x,y positions of each pixel in the map
h.xim = h.xmin:h.res:h.xmax;
h.yim = h.ymin:h.res:h.ymax;

% range to correlate over
h.xrres = 0.01;
h.yrres = 0.01;
h.xrange = [-0.05:h.xrres:0.05];
h.yrange = [-0.05:h.yrres:0.05];
h.yawrres = 0.2;
h.yawrange = [-1.0:h.yawrres:1.0]*pi/180;

% a priori weight
lxr = length(h.xrange);
lyr = length(h.yrange);
lyawr = length(h.yawrange);
xyw = gausswin(lxr*4) * gausswin(lyr*4)';
yaww = gausswin(lyawr*10);
h.xyweight = xyw(size(xyw,1)/2-floor(lxr/2):size(xyw,1)/2+floor(lxr/2), ...
                 size(xyw,2)/2-floor(lyr/2):size(xyw,2)/2+floor(lyr/2));
h.yawweight = yaww(size(yaww,1)/2-floor(lyawr/2):size(yaww,1)/2+floor(lyawr/2));

h.yawweight = ones(size(h.yawrange));

h.pmax = 0.7311;
h.mscale = 100;
h.mmax = h.mscale;
h.mmin = -h.mscale;


h.update = @update;
h.direct_update = @direct_update;
h.update_height = @update_height;
h.update_floor = @update_floor;
h.meters2cells = @meters2cells;
h.valid_indices = @valid_indices;
h.correlation = @correlation;
h.max_correlation = @max_correlation;
h.decay = @decay;
h.get_prob_map = @get_prob_map;
h.get_map = @get_map;
h.get_height_map = @get_height_map;
h.get_floor_map = @get_floor_map;
h.get = @get;
h.set = @set;

  function [xi, yi] = meters2cells(x, y)
  % converts meters to map cell indices
    xi = ceil((x - h.xmin) ./ h.res);
    yi = ceil((y - h.ymin) ./ h.res);
  end

  function [inds ivalid] = valid_indices(xi, yi)
  % finds the indices within the maps boundaries
    ivalid = (xi > 1) & (yi > 1) & (xi < h.sx) & (yi < h.sy);
    inds = sub2ind(size(h.map), xi(ivalid), yi(ivalid));
  end

  function c = correlation(Y)
  % computes the map correlation
  %   Y - 3xN point array
    c = map_correlation(h.map, h.xim, h.yim, Y, h.xrange, h.yrange);
  end

  function [dx dy dyaw maxScore Y_] = max_correlation(X, p, q)
  % finds the maximum correlation across the yaw ranges
    maxScore = 0;
    dx = 0.0;
    dy = 0.0;
    dyaw = 0.0;

    % augment points if needed
    sx = size(X);
    if sx(1) == 3
      X = [X; ones(1,sx(2))]; 
    end
    Y_ = X;

    t = trans(p);
    rpy = quat2rpy(q);
    for i = 1:length(h.yawrange)
      % rotate points with given yaw
      yaw = h.yawrange(i);
      T = t * rotz(rpy(3)+yaw);
      Y = T * X;

      c = h.correlation(Y(1:3,:));

      % find max
      [score ind] = max(c(:));

      if score > maxScore
        maxScore = score;
        [dx, dy] = ind2sub(size(c), ind);
        dx = (dx .* h.xrres) + h.xrange(1); 
        dy = (dy .* h.yrres) + h.yrange(1); 
        dyaw = yaw;
        Y_ = Y;
      end
    end
  end

  function decay()
  % decay map over time
    h.map = h.map - (sign(h.map) .* int8(abs(h.map) < 80));
  end

  function update(Y, dm)
    [xi, yi] = h.meters2cells(Y(1,:), Y(2,:));
    [inds ivalid] = h.valid_indices(xi, yi);
    h.map(inds) = min(h.mmax, max(h.mmin, h.map(inds) + int8(dm)));
  end

  function direct_update(mm)
    h.map(1:1600, 1:1600) = min(h.mmax, max(h.mmin, h.map(1:1600, 1:1600) + int8(mm)));
  end

  function update_height(Y)
    [xi, yi] = h.meters2cells(Y(1,:), Y(2,:));
    [inds ivalid] = h.valid_indices(xi, yi);
    h.height(inds) = max(h.height(inds), Y(3,ivalid));
  end

  function update_floor(Y, r, g, b)
    [xi, yi] = h.meters2cells(Y(1,:), Y(2,:));
    [inds ivalid] = h.valid_indices(xi, yi);
    h.flr.r(inds) = (h.flr.r(inds) + r(ivalid))/2;
    h.flr.g(inds) = (h.flr.g(inds) + g(ivalid))/2;
    h.flr.b(inds) = (h.flr.b(inds) + b(ivalid))/2;
  end

  function ret = get_prob_map()
    ex = exp(double(h.map)./h.mscale);
    ret = ex ./ (1 + ex);
  end

  function ret = get_map()
    ret = h.map;
  end

  function ret = get_height_map()
    ret = h.height;
  end

  function ret = get_floor_map()
    h.flr.rgb(:,:,1) = h.flr.r;
    h.flr.rgb(:,:,2) = h.flr.g;
    h.flr.rgb(:,:,3) = h.flr.b;
    ret = h.flr.rgb;
  end

  function ret = get(field)
    ret = h.(field);
  end

  function set(field, val)
    h.(field) = val;
  end

end

