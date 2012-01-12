function rv = angvel2rvec(w, dt)

  mag = norm(w);
  a = mag * dt;
  if mag == 0
    mag = 1;
  end

  e_v = w(:) ./ mag;

  q = [ cos(a/2.0);
        e_v .* sin(a/2.0)];

