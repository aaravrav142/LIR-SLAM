function q = rpy2quat(rpy)

  rot = rpy2rot(rpy);
  q = rot2quat(rot); 

