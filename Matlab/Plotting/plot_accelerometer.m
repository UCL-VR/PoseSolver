function plot_accelerometer(c)
%PLOT_ACCELEROMETER Summary of this function goes here
%   Detailed explanation goes here
v = c.accelerometer;
clf;
hold all;
plot(v(:,1),v(:,2) * 9.8);
plot(v(:,1),v(:,3) * 9.8);
plot(v(:,1),v(:,4) * 9.8);

% plot the norm

plot(v(:,1),vecnorm(v(:,2:4)') * 9.8);

end

