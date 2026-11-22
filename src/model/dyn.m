function dX = dyn(X, w, drone)
% Rigid-body dynamics (12 states), rotor speeds w (4 x M) as input.
% Element-wise: works for a single column (simulation) and for M nodes at once (NLP,
% numeric / symbolic / optimization expressions).
    g = 9.81; % Acceleration of gravity

    vx = X(4,:);   vy = X(5,:);     vz = X(6,:);
    phi = X(7,:);  th = X(8,:);     psi = X(9,:);
    p = X(10,:);   q = X(11,:);     r = X(12,:);

    cphi = cos(phi);  sphi = sin(phi);
    cth  = cos(th);   sth  = sin(th);   tth = sth./cth;
    cpsi = cos(psi);  spsi = sin(psi);

    w2   = w.^2;
    T    = drone.k_T*(w2(1,:) + w2(2,:) + w2(3,:) + w2(4,:));      % total thrust
    Gphi = drone.l*drone.k_T*(w2(4,:) - w2(2,:));                  % torques
    Gth  = drone.l*drone.k_T*(w2(3,:) - w2(1,:));
    Gpsi = drone.k_Q*(w2(1,:) - w2(2,:) + w2(3,:) - w2(4,:));

    % Translation: (R*[0;0;T] - [0;0;m g])/m, third column of R
    ax = (cpsi.*sth.*cphi + spsi.*sphi).*T/drone.m;
    ay = (spsi.*sth.*cphi - cpsi.*sphi).*T/drone.m;
    az = (cth.*cphi).*T/drone.m - g;

    % Euler-angle kinematics: deta = W(eta)*[p;q;r]
    dphi = p + sphi.*tth.*q + cphi.*tth.*r;
    dth  = cphi.*q - sphi.*r;
    dpsi = (sphi.*q + cphi.*r)./cth;

    % Euler's equation (diagonal inertia): J*dom = -om x (J om) + G
    dp = (-(drone.Jzz - drone.Jyy).*q.*r + Gphi)/drone.Jxx;
    dq = (-(drone.Jxx - drone.Jzz).*r.*p + Gth )/drone.Jyy;
    dr = (-(drone.Jyy - drone.Jxx).*p.*q + Gpsi)/drone.Jzz;

    dX = [vx; vy; vz; ax; ay; az; dphi; dth; dpsi; dp; dq; dr];
end