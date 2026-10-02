function dXdt=dyn(X,U,params)
    % Etats
    vx      = X(4);
    vy      = X(5);
    vz      = X(6);
    phi     = X(7);
    theta   = X(8);
    psi     = X(9);
    p       = X(10);
    q       = X(11);
    r       = X(12);
    
    cphi    = cos(phi);
    sphi    = sin(phi);
    ctheta  = cos(theta);
    stheta  = sin(theta);
    ttheta  = stheta/ctheta;
    cpsi    = cos(psi);
    spsi    = sin(psi);
    
    % Commandes
    w1      = U(1);
    w2      = U(2);
    w3      = U(3);
    w4      = U(4);
    
    % Paramètres
    m       = params.m; % Masse
    k_T     = params.k_T; % Coefficient de poussée hélice
    k_Q     = params.k_Q; % Coefficient de couple de réaction hélice
    l       = params.l; % distance des hélices au centre de gravité
    Jxx     = params.Jxx;
    Jyy     = params.Jyy;
    Jzz     = params.Jzz;
    J       = diag([Jxx Jyy Jzz]); % Matrice d'inertie
    %Jr      = params.Jr;
    g       = 9.81;
    
    % Matrice de transition du repère du quadroptere dans le repere
    % terrestre
    R = [cpsi*ctheta,  cpsi*stheta*sphi - spsi*cphi,  cpsi*stheta*cphi + spsi*sphi;
         spsi*ctheta,  spsi*stheta*sphi + cpsi*cphi,  spsi*stheta*cphi - cpsi*sphi;
         -stheta,      ctheta*sphi,                   ctheta*cphi];
    
    % Matrice de dérivation des angles d'euler
    W = [1  sphi*ttheta cphi*ttheta;
        0  cphi        -sphi;
        0  sphi/ctheta cphi/ctheta];
    
    w = [p; q; r];
    % eta = [phi; theta; psi]
    deta = W*w;
    
    T       = k_T*(w1^2 + w2^2 + w3^2 + w4^2);
    Gphi    = l*k_T*(w4^2 - w2^2);
    Gtheta  = l*k_T*(w3^2 - w1^2);
    Gpsi    = k_Q*(w1^2 - w2^2 + w3^2 - w4^2);
    
    a       = (1/m)*(R*[0;0;T]-[0;0;m*g]);
    %dw      = J \ (-cross(w, J*w) - Jr*cross(w, [0;0;w1-w2+w3-w4]) + [Gphi; Gtheta; Gpsi]);
    dw      = J \ (-cross(w, J*w) + [Gphi; Gtheta; Gpsi]);


    dXdt    = [vx;
               vy;
               vz;
               a(1);
               a(2);
               a(3);
               deta(1);
               deta(2);
               deta(3);
               dw(1);
               dw(2);
               dw(3)];
end