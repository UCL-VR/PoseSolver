function [y] = kt(x, lb, ub, s)
%KT Summary of this function goes here
%   Detailed explanation goes here

% https://dinodini.wordpress.com/2010/04/05/normalized-tunable-sigmoid-functions/


% multiply everything to put it in the range 0-100 or so...

x = x * s;
lb = lb * s;
ub = ub * s;

t_ub = x - ub - 6;
t_lb = lb - x - 6;

y_ub = 1 / (1 + exp(-t_ub));
y_lb = 1 / (1 + exp(-t_lb));

y = y_ub + y_lb;

end

