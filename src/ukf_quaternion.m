function h = ukf_quaternion(Q_coef, R_coef)
% UKF Quaternion Pose filter class

h.Q_coef = Q_coef;
h.R_coef = R_coef;

h.update = @update;
h.fix_yaw = @fix_yaw;
h.adjust_yaw = @adjust_yaw;
h.disturbance = @disturbance;
h.sigma_points = @sigma_points;
h.process_model = @process_model;
h.accel_measurement_model = @accel_measurement_model;
h.set_state = @set_state;
h.get = @get;
h.set = @set;


% initialize state 
h.x_k = [ 1;
          0;
          0;
          0 ];

% state dimension
h.n = 3;

% initialize covariance 
h.P_k = zeros(h.n);

  function set_state(q)
    h.x_k = q;
  end

  function W = disturbance(P)
    % P - covariance matrix
    A = chol(P);
    
    p = sqrt(2*h.n) * A;
    n = -sqrt(2*h.n) * A;

    W = [p, n];
  end

  function X = sigma_points(x_k, W_i)
    qk = x_k;
    n = size(W_i, 2);
    X = zeros(4, n);

    for i = 1:n
      qw = rvec2quat(W_i(:, i));
      X(:,i) = quatmul(qk, qw);
    end
  end

  function X = process_model(X, w_k, dt)
    qw = angvel2quat(w_k, dt);
    n = size(X, 2);

    for i = 1:n
      X(:, i) = quatmul(X(:, i), qw);
    end
  end

  function Z = accel_measurement_model(Y)
    n = size(Y, 2);
    Z = zeros(3, n);
    g = [0; 0; 0; -9.8];

    for i = 1:n
      q = Y(:,i);
      %z = quatmul(quatmul(q, g), quatcong(q));
      z = quatmul(quatcong(q), quatmul(g, q));
      Z(:,i) = z(2:4);
    end
  end

  function x_k = update(accel, angvel, dt)
  % update step of the ukf

    % generate noise covariances
    Q = diag(h.Q_coef(1) + h.Q_coef(2) * abs(angvel));
    R = diag(h.R_coef(1) + h.R_coef(2) * abs(angvel));


    % generate {W_i} from P_k-1 + Q
    h.W = h.disturbance(h.P_k + Q);

    
    % Transform {W_i} to {X_i} (7xN state vectors) using x_k-1 (aka sigma points)
    h.X = h.sigma_points(h.x_k, h.W);


    % update state vectors using the process model A(). {X_i} -> {Y_i}
    h.Y = h.process_model(h.X, angvel, dt);


    % compute a priori x_k as mean of the {Y_i} state vectors
    [h.q_k_ h.ei] = quatmean(h.Y);
    h.x_k_ = [h.q_k_];


    % transform {Y_i} -> {W_i'} by removing x_k and converting the quaternion to a rotation matrix
    %   this is already calculated in finding the mean
    h.W_i_ = [h.ei];


    % a priori covariance P_k^- is computed from {W_i'}
    h.P_k_ = (1/(2*h.n)) * (h.W_i_ * h.W_i_');

    
    % Use acceleration data to project sigma points to 3xN measurement space {Z_i}
    h.Z_accel = h.accel_measurement_model(h.Y);


    % compute mean of {Z_i} to get measurement estimate z_k^-
    %   compare it to the actually measured value z_k
    %   their difference is v_k
    h.z_k_accel_ = (1/(2*h.n)) * sum(h.Z_accel, 2);
    h.v_k_accel = accel - h.z_k_accel_; 
  

    % the innovation covariance P_vv is determined by adding the measurement noise R to the covariance P_zz of the set {Z_i}
    h.Z_m = h.Z_accel - repmat(h.z_k_accel_, [1, size(h.Z_accel, 2)]);
    h.P_zz = (1/(2*h.n)) * (h.Z_m * h.Z_m');
    h.P_vv = h.P_zz + R;


    % the cross correlation matrix P_xz is computed from the sets {W_i'} and {Z_i}
    h.P_xz = (1/(2*h.n)) * (h.W_i_ * h.Z_m');

  
    % the Kalman gain K_k is first computed from P_xz and P_vv 
    h.K_k = h.P_xz * h.P_vv^-1; 


    % use the Kalman gain to calculate a posteriori estimate x_k and its estimate error covariance P_k
    h.Kv = (h.K_k * h.v_k_accel);
    Kv_q = rvec2quat(h.Kv);

    % update state 
    %x_k = quatmul(h.x_k_, Kv_q);
    x_k = quatmul(Kv_q, h.x_k_);
    h.x_k = [x_k];
    
    % update covariance
    h.P_k = h.P_k_ - (h.K_k * h.P_vv * h.K_k');

    % reduce overall covariance
    h.P_k = h.P_k .* .01;
  end

  function ret = get(field)
    ret = h.(field);
  end

  function set(field, val)
    h.(field) = val;
  end

end
