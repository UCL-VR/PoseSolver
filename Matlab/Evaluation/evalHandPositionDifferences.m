function Ranges = evalHandPositionDifferences(h1,h2)
%EVALHANDPOSITIONDIFFERENCES Summary of this function goes here
%   Detailed explanation goes here

time = min(size(h1,3), size(h2,3));

h1 = h1(:,:,1:time);
h2 = h2(:,:,1:time);

d = h1 - h2;

% get difference per bone per frame

d = vecnorm(d);

Ranges = squeeze([max(d)]) * 1000; % put in mm

end

