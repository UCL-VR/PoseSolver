% position random walk is the double integral of the Bias Instability for
% one second

N = 8.9e-5;

Fs = 125;

p = 0;
v = 0;
dt = 1/125;

for i = 0:Fs
    v = v + N * dt * 9.8;
    p = p + v * dt;
end

disp(p * 1000);