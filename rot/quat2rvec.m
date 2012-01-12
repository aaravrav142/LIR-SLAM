function rv = quat2rvec(q)

a = 2*acos(q(1));

if a == 0
  rv = [0;0;0];
else
  rv = q(2:4)./sin(a/2);
  rv = rv .* a;
end


