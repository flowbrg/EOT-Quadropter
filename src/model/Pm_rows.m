function P = Pm_rows(W, dW, drone, mot)
% PM_ROWS  Electrical power of motors [W] per node (sum over the 4 rotors).
%
% Inputs: W, dW  4 x M  speeds [rad/s] and accelerations [rad/s^2]
%         drone  struct (k_Q)
%         motor  struct (Jr, R, KE)
% Output: P      1 x M
    a  = mot.Jr*dW + drone.k_Q*W.^2;
    Pr = (mot.R/mot.KE^2)*a.^2 + W.*a;
    P  = Pr(1,:) + Pr(2,:) + Pr(3,:) + Pr(4,:);
end
