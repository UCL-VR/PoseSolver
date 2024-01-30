function [outputArg1,outputArg2] = plotKt(lb, ub, s)
%PLOTKT Summary of this function goes here
%   Detailed explanation goes here

angles = -60:0.1:60;
angles = -2*pi:0.01:pi*2;

Y = [];

for a = angles
   Y = [Y; kt(a, lb, ub, s)];
end

plot(angles,Y);

end

