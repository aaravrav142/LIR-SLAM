function h = lidar(angles, rmin, rmax)
% lidar class for processing the raw lidar data

h.rmin = rmin;
h.rmax = rmax;
h.angles = angles(:);

h.ca = cos(h.angles);
h.sa = sin(h.angles);

h.raw2cart = @raw2cart;
h.valid_ranges = @valid_ranges;

  function [x y] = raw2cart(r)
  % convert lidar points to cartesian coordinates
    x = r .* h.ca;
    y = r .* h.sa;
  end
  
  function ivalid = valid_ranges(r)
  % filter out any bad readings
    ivalid = (r > h.rmin) & (r < h.rmax);
  end

end

