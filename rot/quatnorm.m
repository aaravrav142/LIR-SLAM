function q = quatnorm(q)

n = sqrt(sum(q.^2));
s = sign(q(1));
if n == 0
  n = 1;
end

ns = n * s;

q = q / ns;

