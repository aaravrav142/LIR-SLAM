function q = angvel2quat(w, dt)

  mag = norm(w);
  a = mag * dt;
  e_v = w(:) ./ (mag + 0.0001);

  q = [ cos(a/2.0);
        e_v .* sin(a/2.0)];
  q = quatnorm(q);

