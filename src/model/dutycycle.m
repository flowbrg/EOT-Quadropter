function [Dc, dz] = dutycycle(w_ref, w, z, batt, mot)
% Feedforward + PI feedback
% Input :   w_ref:  [4x1]   reference angular speed
%           w:      [4x1]   measured angular speed
%
% Output :  Dc:     [4x1]   Duty-cycle [0;1]
%           dz:     [4x1]   PI error state

% Feedforward
Dcff = (mot.R*mot.d)/(mot.KE*batt.E_ocv).*w_ref.^2 + (mot.KE/batt.E_ocv).*w_ref;

% PI feedback
e = w_ref-w;
uPI = mot.Kp*e + mot.Ki.*z;

% Duty cycle
Dc = Dcff + uPI;
Dc = min(1,max(Dc,0));

% Debug
% fprintf('Dc1 = %.4f | Dc2 = %.4f | Dc3 = %.4f | ...
%     Dc4 = %.4f\n', Dc(1), Dc(2), Dc(3), Dc(4));


% Update PI error state
dz = e;
end