function qc = quatcong(q)

  qc = q(:);
  qc(2:4) = -qc(2:4);

