function [] = plot_stream(A)
%PLOT_STREAM Summary of this function goes here
%   Detailed explanation goes here

ids = unique(A(:,8))';
for id = ids
    rows = A(A(:,8)==id,:);
    
    time = rows(:,4);
    vectors = rows(:,5:7);

    if( rows(1,1) == 2 )
        vectors = vectors / 1000; % put the optical coordinate system into m
    end

    subplot(length(ids),1,id);
    plot(time,vectors);

end

end

