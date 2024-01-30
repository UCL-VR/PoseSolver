% Imports a Capture of a stream written by an ImuMarker by the
% ImuBiasEstimator class in Unity.

% The ImuBiasEstimator stream contains Imu measurements, along with
% observations about the frame to which it is attached, taken from the hand
% pose estimator. This is the same set of data that a calibration or
% integration algorithm would have access to.

function E = importImuMarkerStream(filename)

    if nargin < 1
        filename = "biasEstimates.bin";
    end
    
    f = fopen(filename);
    E = fread(f,"single");
    fclose(f);

    p = 7 * 3;

    E = E(1:floor(length(E)/p)*p);
    E = reshape(E,p,[])';
    filt = find(E(:,1));
    E = E(filt,:);
    E = E(2:end,:);

    accelerometer = E(:,1:3);
    gyro = E(:,4:6);
    position = E(:,7:9);
    acceleration = E(:,10:12);
    angularVelocity = E(:,13:15);
    rotation = E(:,16:18);
    time = E(:,19);
    E = table(accelerometer, gyro, position, acceleration, angularVelocity, rotation, time);

end