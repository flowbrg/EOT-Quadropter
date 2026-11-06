load("params.mat")

N = 51;
soc = linspace(0,1,N);
v = polyval(batt.Pol_ocv,soc) + log(soc).*soc;

figure(1);
plot(soc,v,'b-');
xlabel("State of charge");
ylabel("Open circuit voltage (V)");