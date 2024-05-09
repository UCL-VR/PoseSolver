#include "pch.h"
#include "Imu.h"
#include "Export.h"
#include "Unity.h"
#include "Transforms.h"

class ImuScene
{
public:
    ceres::Problem problem;
};

ImuScene* scene;

EXPORT void imu_initialise()
{
    scene = new ImuScene();
}

// For now this sets the preintegration velocity to zero
EXPORT imu::PreIntegrationFactor* imu_addPreIntegrationFactor(transforms::Transformd* start, transforms::Transformd* end)
{
    auto factor = new imu::PreIntegrationFactor(start, end);
    factor->addToProblem(scene->problem);
    scene->problem.SetParameterBlockConstant(factor->velocity->data());
    return factor;
}

/// <summary>
/// Adds a sample from the IMU to the pre-integration factor. ax, ay & az are
/// the accelerometer readings. gx, gy & gz are the gyroscope readings. Though
/// in reality they will be read at slightly different times, the pre-integration
/// factor requires that they have a single timestamp between them.
/// The units for the accelerometer are m/s^2, and the gyroscope in radians-per
/// second.
/// </summary>
EXPORT void imu_addImuMeasurement(imu::PreIntegrationFactor* factor, float ax, float ay, float az, float gx, float gy, float gz, float dt)
{
    factor->addSample(Eigen::Vector3d(ax, ay, az), Eigen::Vector3d(gx, gy, gz), dt);
}

EXPORT imu::ImuOrientationFactor* imu_addImuOrientationFactor(transforms::Transformd* reference, transforms::Transformd* start)
{
    auto factor = new imu::ImuOrientationFactor(reference, start);
    factor->addToProblem(scene->problem);
    return factor;
}

EXPORT void imu_addImuOrientationMeasurement(imu::ImuOrientationFactor* factor, float ax, float ay, float az, float gx, float gy, float gz, float dt)
{
    factor->addSample(Eigen::Vector3d(ax, ay, az), Eigen::Vector3d(gx, gy, gz), dt);
}

/// <summary>
/// Creates an ImuCalibration2 Parameter Block and returns a pointer to it. For
/// use with the ImuCalibrationFactor2 factor.
/// </summary>
EXPORT imu::ImuBiasParameters* imu_addBiasParameters()
{
    auto parameterBlock = new imu::ImuBiasParameters();
    scene->problem.AddParameterBlock((double*)parameterBlock->accelerometerBias.data(), 3);
    scene->problem.AddParameterBlock((double*)parameterBlock->gyroscopeBias.data(), 3);
    return parameterBlock;
}

EXPORT unity::ImuBiasParameters imu_getBiasParameters(imu::ImuBiasParameters biasParameters)
{
    return biasParameters;
}

/// <summary>
/// Creates an ImuCalibrationFactor2 and adds it to the problem. This overload
/// takes the necessary observations directly from the function parameters.
/// </summary>
/// <returns></returns>
EXPORT imu::ImuBiasFactor* imu_addBiasFactor(imu::ImuBiasParameters* biasParameters, unity::MotionFrame referenceFrame, unity::Vector3 accelerometer, unity::Vector3 gyroscope)
{
    return new imu::ImuBiasFactor(scene->problem, biasParameters, referenceFrame, accelerometer, gyroscope);
}

EXPORT void imu_removeBiasFactor(imu::ImuBiasFactor* factor)
{
    scene->problem.RemoveResidualBlock(factor->id);
}