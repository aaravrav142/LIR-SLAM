function [qt ei] = quatmean(q, qt)
% compute the mean of the set of quaternions 
%   q - 4xN quaternion vectors 
%   qt - optional seed for the mean calculation

if nargin < 2
  qt = q(:,1);
end

n = size(q, 2);
% init error vectors
ei = zeros(3, n);

% while the adjustment vector is > 0 (thres)
count = 0;
de = 1;
while de > 0.001
  
  % compute the rotation between the current mean estimate and each quaternion
  for i = 1:n
    dq = quatmul(q(:,i), quatcong(qt));
    ei(:,i) = quat2rvec(dq);
  end

  % compute average rotation
  e_vec = (1/n) * sum(ei, 2);
  de = sum(e_vec);
  e = rvec2quat(e_vec);

  % update mean estimate
  qt = quatmul(e, qt);

  count = count + 1;
end

