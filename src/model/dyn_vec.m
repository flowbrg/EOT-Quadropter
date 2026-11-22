function F = dyn_vec(X, U, drone)
% NLP dynamics (16 states): rotor speeds are states, rotor accelerations U are the inputs.
F = [dyn(X(1:12,:), X(13:16,:), drone); U];
end