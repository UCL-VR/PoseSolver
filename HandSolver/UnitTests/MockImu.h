#pragma once

// This file contains tools for mocking up an imu

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// The Mock Imu cam make use of *trajectories*. These are recorded six dof
// transforms (expressed as CSV sequences). The trajectories store only one
// body - the Mock IMU will generate synthetic inertial data for transforms
// offset from the body described by the trajectories.

// These trajectories can come from any source, such as animations in Blender,
// or VR controllers in Unity.
// The format should always be as follows, however...
// [x y z q_x q_y q_z q_w ...]

// It is assumed samples are regular. The unit tests may speed up or slow down
// sequences accordingly.

// Trajectory 1 is a simple, artificial six-dof trajectory created in Blender
extern const std::vector<double> trajectory1;


class Trajectory {
public:

#pragma pack(push, 1)
	struct Frame
	{
		Eigen::Vector3d position;
		Eigen::Quaterniond rotation;
	};
#pragma pack(pop)

	Trajectory(std::vector<double> data);
	Trajectory(Frame* data, int count);

	std::vector<Frame> frames;
};

// Represents a fake IMU

class MockImu {
public:
	struct State {
		Eigen::Vector3d position;
		Eigen::Quaterniond rotation;
		Eigen::Vector3d linearVelocity;
		Eigen::Vector3d linearAcceleration;
		Eigen::Quaterniond angularVelocity;
		Eigen::Vector3d angularVelocityDps;

		// Writes the position and rotation as space-separated vectors
		void ToString1(std::ostream& s)
		{
			s << position.transpose() << " " << rotation.x() << " " << rotation.y() << " " << rotation.z() << " " << rotation.w();
		}
	};

	struct Frame {
		State reference;
		State imu;
	};

	struct Settings {
		Eigen::Vector3d gravity;

		Settings()
		{
			gravity = Eigen::Vector3d(0, -9.8, 0);
		}
	};

	MockImu(Trajectory trajectory, Settings settings, Eigen::Vector3d localPosition, Eigen::Quaterniond localRotation, double interval);

	Settings settings;
	std::vector<Frame> frames;
};