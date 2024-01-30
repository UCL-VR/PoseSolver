#include "pch.h"

#define GLOG_NO_ABBREVIATED_SEVERITIES
#define NOMINMAX // Stop minwindef.h overriding std::max, if it gets included somehow

#include <ceres/ceres.h>
#include <ceres/gradient_checker.h>
#include <glog/logging.h>
#include <Imu.h>

using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using namespace hs;
using namespace Eigen;

TEST(HAND, Hand1)
{
	// Tests the ImuCalibrationFactor2 using the solved hand positions




}