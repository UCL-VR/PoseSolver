
% Computes and displays the Allan Variance for an IMU. 

% C should be a Capture containing 1 Imu Stream.

function evalAllanVariance(C)

    C = C( C.Type == 1, :);
    accelerometer_samples = [C.x C.y C.z];
    
    [avar,tau]= allanvar(accelerometer_samples,'octave',125);
    loglog(tau,avar)
    xlabel('\tau')
    ylabel('\sigma^2(\tau)')
    title('Allan Variance')
    grid on
    legend({"x","y","z"})

end