% This function reads a binary stream from the TrackerManager, which will
% contain a number of measurments by a number of Trackers.

function [A] = importTrackerStream(filename, type)

    if nargin < 1
        filename = "ImuLongCapture.bin";
    end

    f = fopen(filename);
    A = fread(f,'float32');
    fclose(f);
    A = A(1:length(A) - mod(length(A),6));
    A = reshape(A,6,[])';

    % add stream Ids
    ids = A(:,1:2);
    ids = unique(ids,'rows');
    stream_id = 1;
    
    for id = ids'
        A(A(:,1) == id(1) &  A(:,2) == id(2), 7) = stream_id;
        stream_id = stream_id + 1;
    end
   
    % add magnitudes
    A(:,8) = vecnorm(A(:,4:6)');    

    if nargin > 1
        A = A(A(:,1)==type,:);
    end

    A = array2table(A);    
    A.Properties.VariableNames = {'Type','Marker','Time','x','y','z','Id','Magnitude'};
end

