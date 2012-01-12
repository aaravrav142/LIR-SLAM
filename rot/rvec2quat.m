function q = rvec2quat(rv)
  
a = sqrt(sum(rv.^2));
if a ~= 0
  x = rv ./ a;
else
  x = rv(:);
end

q = [cos(a/2); x(:)*sin(a/2)];
q = quatnorm(q);

