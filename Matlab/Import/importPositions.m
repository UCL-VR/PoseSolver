function P = importPositions(filename)
%IMPORTPOSITIONS Imports a stream of N position measurements over time
% Expects x,y,z x N x Frame
    if nargin < 1
        filename = "hand1.positions";
    end

    N = 16;

    f = fopen(filename);
    P = fread(f,"single");
    fclose(f);

    P = reshape(P,3,N,[]);
    
end

