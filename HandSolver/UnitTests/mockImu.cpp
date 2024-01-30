#include "pch.h"

#include "MockImu.h"

#include <vector>

#pragma optimize("",off)

// The include directive is equivalent to copying the contents of the file into
// the source, initialising the array with the string contained in the CSVs.
// (As a pre-processing step, replace all the new-line characters with commas.)

std::vector<double> const trajectory1 = std::vector<double>({
	#include "trajectories/trajectory1.csv"
});



Trajectory::Trajectory(std::vector<double> data)
{
	auto stride = 7;
	auto num = data.size() / stride;
	for (size_t i = 0; i < num; i++)
	{
		auto f = *(Frame*)&data[i * stride]; // This works given the current implementation of Eigen - if this changes update the conversion
		frames.push_back(f); 
	}
}

Trajectory::Trajectory(Frame* data, int count)
{
	for (size_t i = 0; i < count; i++)
	{
		frames.push_back(data[i]);
	}
}

MockImu::MockImu(Trajectory trajectory, Settings settings, Eigen::Vector3d localPosition, Eigen::Quaterniond localRotation, double interval)
{
	for (size_t i = 2; i < trajectory.frames.size(); i++)
	{
		Frame f;

		auto v1 = (trajectory.frames[i].position - trajectory.frames[i - 1].position) / interval;
		auto v0 = (trajectory.frames[i - 1].position - trajectory.frames[i - 2].position) / interval;

		f.reference.position = trajectory.frames[i].position;
		f.reference.rotation = trajectory.frames[i].rotation;
		f.reference.linearVelocity = v1;
		f.reference.linearAcceleration = (v1 - v0) / interval;
		f.reference.angularVelocity = (trajectory.frames[i].rotation * trajectory.frames[i - 1].rotation.inverse()); // TODO make use interval
		f.reference.angularVelocityDps = f.reference.angularVelocity.toRotationMatrix().eulerAngles(0, 1, 2);

		// These are the true pose of the Imu and will usually not be directly observable
		f.imu.position = (f.reference.rotation * localPosition) + f.reference.position;
		f.imu.rotation = (f.reference.rotation * localRotation);

		f.imu.angularVelocity = (f.reference.rotation * f.reference.angularVelocity);
		f.imu.linearAcceleration = (localRotation.inverse() * f.reference.linearAcceleration);
		f.imu.linearVelocity = (localRotation.inverse() * f.reference.linearVelocity);
		f.imu.angularVelocityDps = f.imu.angularVelocity.toRotationMatrix().eulerAngles(0, 1, 2);

		// Add biases and gravity

		f.imu.linearAcceleration += f.imu.rotation.inverse() * settings.gravity;

		frames.push_back(f);
	}
}

void CheckVector(Eigen::Vector3d v1, Eigen::Vector3d v2, double tol);

TEST(MOCKIMU, Simple)
{
	// Test whether the basic conversion of the time frames works

	using namespace Eigen;

	Trajectory::Frame* frames = new Trajectory::Frame[3];

	frames[0].position = Vector3d(0, 0, 0);
	frames[1].position = Vector3d(0, 0, 0.5);
	frames[2].position = Vector3d(0, 0, 1.5);

	MockImu::Settings settings;
	settings.gravity = Eigen::Vector3d::Zero();

	MockImu imu(Trajectory(frames, 3), settings, Vector3d::Zero(), Quaterniond::Identity(), 1.0);

	EXPECT_EQ(imu.frames.size(), 1);
	CheckVector(imu.frames[0].reference.position, frames[2].position, 0);
	CheckVector(imu.frames[0].reference.linearVelocity, Vector3d(0, 0, 1), 0);
	CheckVector(imu.frames[0].reference.linearAcceleration, Vector3d(0, 0, 0.5), 0);
}

TEST(MOCKIMU, LocalRotation)
{
	// A 45 deg rotation, but no position offset. 

	using namespace Eigen;

	Trajectory::Frame* frames = new Trajectory::Frame[3];

	frames[0].position = Vector3d(0, 0, 0);
	frames[1].position = Vector3d(0, 0, 0.5);
	frames[2].position = Vector3d(0, 0, 1.5);

	MockImu::Settings settings;
	settings.gravity = Eigen::Vector3d::Zero();

	MockImu imu(Trajectory(frames, 3), settings, Vector3d::Zero(), Quaterniond(AngleAxisd(1.5708, Vector3d::UnitX())), 1.0);

	EXPECT_EQ(imu.frames.size(), 1);
	CheckVector(imu.frames[0].reference.position, frames[2].position, 0);
	CheckVector(imu.frames[0].reference.linearVelocity, Vector3d(0, 0, 1), 0);
	CheckVector(imu.frames[0].reference.linearAcceleration, Vector3d(0, 0, 0.5), 0);

	CheckVector(imu.frames[0].imu.position, frames[2].position, 0);

	// The rotation effectively swaps the x and y axis

	CheckVector(imu.frames[0].imu.linearVelocity, Vector3d(0, 1, 0), 0.001);
	CheckVector(imu.frames[0].imu.linearAcceleration, Vector3d(0, 0.5, 0), 0.001);

}

// Add test for local offset


// Add test for rotational velocity, and dps integrtation