function rpy = quat2rpy(q)
  
  rot = quat2rot(q);
  rpy = rot2rpy(rot);
          
