function dstates=dyn(states,commands,params)
    % Etats
    x       = states(1);
    y       = states(2);
    z       = states(3);
    vx      = states(4);
    vy      = states(5);
    vz      = states(6);
    phi     = states(7);
    theta   = states(8);
    psi     = states(9);
    p       = states(10);
    q       = states(11);
    r       = states(12);
    
    cphi    = cos(phi);
    sphi    = sin(phi);
    ctheta  = cos(theta);
    stheta  = sin(theta);
    ttheta  = stheta/ctheta;
    cpsi    = cos(psi);
    spsi    = sin(psi);
    
    % Commandes
    w1      = commands(1);
    w2      = commands(2);
    w3      = commands(3);
    w4      = commands(4);
    
    % Paramètres
    m       = params(1); % Masse
    k_T     = params(2); % Coefficient de poussée hélice
    k_Q     = params(3); % Coefficient de couple de réaction hélice
    l       = params(4); % distance des hélices au centre de gravité
    Jxx     = params(5);
    Jyy     = params(6);
    Jzz     = params(7);
    J       = diag([Jxx Jyy Jzz]); % Matrice d'inertie
    Jr      = params(8)
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
    dw      = J \ (-cross(w, J*w) - Jr*cross(w, [0;0;w1-w2+w3-w4]) + [Gphi; Gtheta; Gpsi]);


    dstates = [vx;
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