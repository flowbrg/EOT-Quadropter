function [Dc, valid] = ff(wr)

% Motor and battery parameters
k_Q   = 7.5e-7;
KE    = 1e-2;
R     = 0.1;
R_int = 5e-2;
E_ocv = 11.1;

% Coefficient C (single motor)
C = (k_Q/KE) .* wr.^2 .* ...
    (KE .* wr + R .* (k_Q/KE) .* wr.^2);

% Discriminant
Delta = E_ocv^2 - 4*R_int.*C;

% Initialize outputs
Dc = NaN(size(wr));

% Valid discriminant
valid = Delta >= 0;

% Terminal voltage
Vterm = NaN(size(wr));
Vterm(valid) = ...
    (E_ocv + sqrt(Delta(valid))) / 2;

% Duty-cycle
Dc(valid) = ...
    (KE.*wr(valid) + R.*(k_Q/KE).*wr(valid).^2) ...
    ./ Vterm(valid);

end
