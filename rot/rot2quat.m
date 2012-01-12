function q = rot2quat(rot)

  w = (1/2) * sqrt(rot(1,1) + rot(2,2) + rot(3,3) + 1);
  x = (rot(3,2) - rot(2,3))/(4*w);
  y = (rot(1,3) - rot(3,1))/(4*w);
  z = (rot(2,1) - rot(1,2))/(4*w);
  q = quatnorm([w; x; y; z]);
  %q = [w; x; y; z];


