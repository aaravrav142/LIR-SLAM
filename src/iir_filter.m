function h = iir_filter(a, v)
% iir filter
% v = a * y_i + (1-a) * v;
%   a - weight
%   v - initial value

if nargin < 2
  v = 0.0;
end

h.a = a;
h.v = v;

h.update = @update;

  function v = update(y)
    h.v = h.a * y + (1-h.a) * h.v;
    v = h.v; 
  end

end

