function [R] = print_stream_correlations(M)
%PLOT_STREAM_CORRELATIONS Summary of this function goes here
%   Detailed explanation goes here

R = [];
ids = unique(M(:,8))';
for a = ids
    for b = ids 
        A = M(M(:,8) == a,:);
        B = M(M(:,8) == b,:);
    
        timesA = A(:,4);
        timesB = B(:,4);
        [~,iA] = unique(timesA);
        [~,iB] = unique(timesB);

        xq = max(max(timesA),max(timesB));

        xq = 0:0.001:xq';

        A = A(iA,:);
        B = B(iB,:);

        Ma = interp1(A(:,4),A(:,9),xq,'nearest','extrap');
        Mb = interp1(B(:,4),B(:,9),xq,'nearest','extrap');

        c = corrcoef(Ma,Mb);
        R(a,b) = c(1,2);
    end
end

end

