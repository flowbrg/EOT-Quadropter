clear; clc; close all;

init_params

N = 51;
soc = linspace(0,1,N);
v = polyval(batt.Pol_ocv,soc) + log(soc).*soc;

figure;
plot(soc,v,'b-');
xlabel("State of charge");
ylabel("Open circuit voltage (V)");