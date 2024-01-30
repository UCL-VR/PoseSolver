% This script puts Measurements from an Imu in the form output by the
% importImuMarkerStream function. 

% This method assumes the Imu remains stationary at all times (i.e. it is
% flat on a table).

function E = makeBiasEstimatesFromTableCapture(C)

% Filter only the Imu data
C = C( C.Type == 1, :);

% Convert from g's to m/s^2

C.x = C.x * 9.81;
C.y = C.y * 9.81;
C.z = C.z * 9.81;

% Transform into the Unity Coordinate System. This is the same operation
% that takes place in ImuMarker.cs.

X = -C.y;
Y = -C.z;
Z = -C.x;

C.x = X;
C.y = Y;
C.z = Z;

% Make a fake E

E = zeros(height(C),7 * 3);

E(:,1:3) = [C.x C.y C.z];
E(:,19) = [0; diff(C.Time)];

% And finally perform the same operations as at the end of importImuMarkerStream

accelerometer = E(:,1:3);
gyro = E(:,4:6);
position = E(:,7:9);
acceleration = E(:,10:12);
angularVelocity = E(:,13:15);
rotation = E(:,16:18);
time = E(:,19);

E = table(accelerometer, gyro, position, acceleration, angularVelocity, rotation, time);

end