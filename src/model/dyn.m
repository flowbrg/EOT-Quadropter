function dX = dyn(X, w, drone, wd)
% Rigid-body dynamics (12 states), rotor speeds w (4 x M) as input.
% Element-wise: works for a single column (simulation) and for M nodes at once
% (NLP, numeric / symbolic / optimization expressions).
%
% Inputs
%   X     : 12 x M  [x y z vx vy vz phi th psi p q r]'
%   w     : 4  x M  rotor speeds [rad/s]
%   drone : struct, requires fields m, l, k_T, k_Q, Jxx, Jyy, Jzz, Jr (rotor inertia, kg.m^2)
%   wd    : (optional) 4 x M rotor accelerations [rad/s^2]; if omitted or empty,
%           the yaw reaction torque -Jr*d(wG)/dt is neglected (README Sec. 1.4)
% Output
%   dX    : 12 x M state derivative
%
% Spin convention: rotors 1,3 clockwise, 2,4 counter-clockwise (seen from above, z_b up)
    if nargin < 4, wd = []; end
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

    % Net rotor angular speed along z_b (linear in w): wG = sum(s_i w_i), s = (-1,+1,-1,+1)
    wG = -w(1,:) + w(2,:) - w(3,:) + w(4,:);

    % Yaw reaction torque from rotor acceleration: -Jr * d(wG)/dt (optional)
    if ~isempty(wd)
        wGd  = -wd(1,:) + wd(2,:) - wd(3,:) + wd(4,:);
        Gpsi = Gpsi - drone.Jr*wGd;
    end

    % Translation: (R*[0;0;T] - [0;0;m g])/m, third column of R
    ax = (cpsi.*sth.*cphi + spsi.*sphi).*T/drone.m;
    ay = (spsi.*sth.*cphi - cpsi.*sphi).*T/drone.m;
    az = (cth.*cphi).*T/drone.m - g;

    % Euler-angle kinematics: deta = W(eta)*[p;q;r]
    dphi = p + sphi.*tth.*q + cphi.*tth.*r;
    dth  = cphi.*q - sphi.*r;
    dpsi = (sphi.*q + cphi.*r)./cth;

    % Euler's equation (diagonal inertia): J*dom = -om x (J om) + G + G_gyro
    % G_gyro = Jr*wG*[-q; p; 0]
    dp = (-(drone.Jzz - drone.Jyy).*q.*r - drone.Jr.*wG.*q + Gphi)/drone.Jxx;
    dq = (-(drone.Jxx - drone.Jzz).*r.*p + drone.Jr.*wG.*p + Gth )/drone.Jyy;
    dr = (-(drone.Jyy - drone.Jxx).*p.*q + Gpsi)/drone.Jzz;

    dX = [vx; vy; vz; ax; ay; az; dphi; dth; dpsi; dp; dq; dr];
end