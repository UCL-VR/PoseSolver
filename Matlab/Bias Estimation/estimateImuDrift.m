% This function emulates the behaviour of the calibration and prediction
% methods that may go into the Hand Tracker.

% It operates on an Imu Table (E) - a recording of the Imu measurements as
% well as observations made about the Imu frame based on the output of the
% hand solver in Unity.

% All the measurments in this table have consistent units and coordinate
% system.

% The current version of this method attempts to compute 1D accelerometer
% and gyro biases based on averaging the estimated errors by subtracting
% the measurements from observations made of the same frame for a fixed
% period. Immediately after this those same biases are used in an
% integration, during which the predicated positions are compared with the
% observed ones.

function estimateImuDrift(E)

start = 1000; % wait for the hand estimator etc to get set up
biasCalibrationSamples = 250;
g = 9.81;

% Estimate the biases of the Imu given a short previous history

% Accelerometer Errors
accelerometerErrors = [];

for i = start:start+biasCalibrationSamples

    % true state at step i

    orientation = E.rotation(i,:);
    observedAcceleration = E.acceleration(i,:);
    
    % estimated g given observed rotation

    m = rotvec2mat3d(deg2rad(orientation));
    expectedGravity = (m * [0 -9.81 0 ]')';

    % observed acceleration in the local frame of the Imu

    expectedMeasuredAcceleration = (inv(m) * observedAcceleration')';

    % measurements

    measuredAcceleration = E.accelerometer(i,:);

    % accelerometer error

    accelerometerError = measuredAcceleration - expectedMeasuredAcceleration - expectedGravity;

    accelerometerErrors = [accelerometerErrors; accelerometerError];

end

% Estimate bias as mean of errors

accelerometerBias = mean(accelerometerErrors);
gyroscopeBias = [ 0 0 0 ];


% Starting conditions for Integration

start = start+biasCalibrationSamples;
time = 0;
position = E.position(start,:);
rotation = E.rotation(start,:);
velocity = [0 0 0];

% Error metrics

positionErrors = [];
times = [];
as = [];

for i = start:height(E)
    
    % Inertial measuremnents, corrected for biases

    dt = E.time(i);
    
    am = E.accelerometer(i,:) - accelerometerBias; % acceleration corrected for bias
    dps = E.gyro(i,:) - gyroscopeBias;

    % Predict gravity based on current orientation estimate
    
    m = rotvec2mat3d(deg2rad(rotation));
    g = m * [0 -9.81 0 ]';

    % Subtract gravity from current estimate

    a = am - g'; % acceleration corrected for gravity

    as = [as; a]; % record a history of the corrected acceleration measurements used

    % Integrate acceleration

    velocity = velocity + (a * dt);
    position = position + (velocity * dt);

    time = time + dt;

    % Integrate rotation

    rotation = rotation + dps * dt;

    % Compare position with measured positions

    positionErrors = [positionErrors; (position - E.position(i,:))];
    times = [times; time];

end

clf;
plot(times,positionErrors);
ylim([0,0.02]);

end