function h = kinect(klog)
% matlab class for handling the kinect data
%   extracting images, rgb + depth merging, etc

if isa(klog, 'char')
  k = load(klog);
else
  k = klog;
end

% calibration params
h.fx = 585.05108211;
h.fy = 585.05108211;
h.cx = 315.83800193;
h.cy = 242.94140713;

h.sim = [480, 640];
h.i = repmat([1:h.sim(2)], [h.sim(1), 1]);
h.j = repmat([1:h.sim(1)]', [1, h.sim(2)]);
[h.yi h.xi] = ind2sub(h.sim, [1:prod(h.sim)]);
h.xc = h.xi - h.cx;
h.yc = h.yi - h.cy;
h.X = [h.xc; h.yc; ones(1, size(h.xc,2))];

% reduced image
h.ri = h.i(1:4:end, 1:4:end);
h.rj = h.j(1:4:end, 1:4:end);
h.rind = sub2ind(h.sim, h.rj(:), h.ri(:));
h.rxi = h.xi(h.rind);
h.ryi = h.yi(h.rind);
h.rxc = h.rxi - h.cx;
h.ryc = h.ryi - h.cy;
h.rX = [h.rxc; h.ryc; ones(1, size(h.rxc,2))];

% generate and store arrays of indices
[yis zis] = meshgrid([1:640], [1:480]);
h.yis = yis(:)';
h.zis = zis(:)';

% parameterized plane equation of the floor in the disparity image
h.flr.A = [0.0035; -1.0749; 852.0400]; % default value
h.flr.merr = 10;


h.zdepth = k.zdepth;
h.jrgb = k.jrgb;
h.depth_ts = k.depth_ts;
h.rgb_ts = k.rgb_ts;
h.disparity = @disparity;
h.update_floor = @update_floor;
h.floor_indices = @floor_indices;
h.rgb = @rgb;
h.depth = @depth;
h.merge = @merge;
h.irls = @irls;
h.irls_weights = @irls_weights;
h.rirls = @rirls;
h.rirls_weights = @rirls_weights;
h.init = @init;
h.get = @get;
h.set = @set;

h.init();

  function init()
    % match depth and rgb timestamps
    h.rgb_ind = zeros(size(h.depth_ts));
    for i = 1:length(h.depth_ts)
      [m ind] = min(abs(h.rgb_ts(:) - h.depth_ts(i)));
      h.rgb_ind(i) = ind;
    end
  end

  function d = disparity(ind)
    d = reshape(typecast(zlibUncompress(h.zdepth{ind}), 'uint16'), [640, 480])';
  end

  function im = rgb(ind)
    im = djpeg(h.jrgb{ind});
  end

  function [Y ivalid] = depth(d)
    d = double(d(:))';

    %get the x values (forward distance)
    %slightly different from ROS values
    xs = 1.03 ./ (-0.00304 .* d + 3.31);

    %calculate y and z values
    ys = -(h.yis-h.cx) ./ h.fx .* xs;
    zs = -(h.zis-h.cy) ./ h.fy .* xs;

    %extract good values
    ivalid = xs > 0;

    % transform based on sensor pitch
    T = roty(0.36);
    Y = T*[xs; ys; zs; ones(size(xs))];
  end

  function drgb  = merge(d, im)
  % merge the depth and rgb
    global dd ii jj kk rgbi rgbj ivalid dr dg db ir ig ib
    
    dd = (-0.00304 .* double(d) + 3.31);
    ii = (h.i .* 526.37) + dd * (-4.5 * 1750.46) + 19276.0;
    jj = h.j .* 526.37 + 16662.0;
    kk = 585.051;
  
    % get the index in the rgb image
    rgbi = fix(ii./kk);
    rgbj = fix(jj./kk);

    % make sure we are not out of bounds
    ivalid = (rgbi < h.sim(2)+1) & (rgbi > 0) & (rgbj < h.sim(1)+1) & (rgbj > 0);

    dr = zeros(h.sim);
    dg = zeros(h.sim);
    db = zeros(h.sim);
    ir = im(:,:,1);
    ig = im(:,:,2);
    ib = im(:,:,3);

    ind = sub2ind(h.sim, rgbj(ivalid), rgbi(ivalid));
    dr(ivalid) = ir(ind);
    dg(ivalid) = ig(ind);
    db(ivalid) = ib(ind);

    drgb = zeros([h.sim, 3]);
    drgb(:,:,1) = dr;
    drgb(:,:,2) = dg;
    drgb(:,:,3) = db;
  end

  function [A ifloor] = update_floor(d)
  % do irls to find ground plane and return floor indices
    A = h.rirls(h.flr.A, d);

    ifloor = h.floor_indices(A, d);

    h.flr.A = A;
  end

  function ifloor = floor_indices(A, d)
  % return indices of the floor points
    d = double(d(:));
   
    err = h.X'*A - d;

    % points within range are floor
    ifloor = abs(err) < h.flr.merr;
  end

  function A = irls(A, d)
  % find ground plane using iteratively reweighted least squares
  %   d - dispartity
  %   A - current model
    d = double(d(:));
    
    % loop until threshold is met
    % TODO: determine this thres
    for i = 1:10
   
      % compute weights
      w = h.irls_weights(A, d);

      % determine new model
      A = (repmat(w, [1,3]) .* h.X')\(w .* d);
    end
  end

  function w = irls_weights(A, d)
    % TODO: better weighting
    w = exp(-abs(h.X'*A - d));
  end

  function A = rirls(A, d)
  % find ground plane using iteratively reweighted least squares
  %   d - dispartity
  %   A - current model
    d = double(d(:));
    
    % loop until threshold is met
    % TODO: determine this thres
    for i = 1:4
   
      % compute weights
      w = h.rirls_weights(A, d);

      % determine new model
      A = (repmat(w, [1,3]) .* h.rX')\(w .* d(h.rind));
    end
  end

  function w = rirls_weights(A, d)
    % TODO: better weighting?
    w = exp(-abs(h.rX'*A - d(h.rind)));
  end

  function ret = get(field)
    ret = h.(field);
  end

  function set(field, val)
    h.(field) = val;
  end

end

