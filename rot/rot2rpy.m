function rpy = rot2rpy(rot)

  y = atan2(rot(2,1), rot(1,1)); 
  p = atan2(-rot(3,1), sqrt(rot(3,2)^2 + rot(3,3)^2));
  r = atan2(rot(3,2), rot(3,3));
  rpy = [r; p; y];

