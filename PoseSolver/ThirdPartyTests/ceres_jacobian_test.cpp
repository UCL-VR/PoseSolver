#include <iostream>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Core>

struct RotationCostFunctor {
    RotationCostFunctor(const Eigen::Vector3d& observed_point, const Eigen::Vector3d& target_point)
        : observed_point_(observed_point), target_point_(target_point) {}

    template <typename T>
    bool operator()(const T* const rotation, T* residuals) const {
        // Rotate the observed point using the rotation quaternion
        T observed_point[3] = { T(observed_point_[0]), T(observed_point_[1]), T(observed_point_[2]) };
        T rotated_point[3];
        ceres::QuaternionRotatePoint(rotation, observed_point, rotated_point);

        // Compute the residuals (difference between the rotated point and the target point)
        residuals[0] = rotated_point[0] - T(target_point_[0]);
        residuals[1] = rotated_point[1] - T(target_point_[1]);
        residuals[2] = rotated_point[2] - T(target_point_[2]);

        return true;
    }

    const Eigen::Vector3d observed_point_;
    const Eigen::Vector3d target_point_;
};

int main() {
    // Observed points (e.g., from a sensor)
    std::vector<Eigen::Vector3d> observed_points = {
        {1.0, 2.0, 3.0},
        {4.0, 5.0, 6.0},
        {7.0, 8.0, 9.0}
    };

    // Target points (e.g., from a model)
    std::vector<Eigen::Vector3d> target_points = {
        {1.1, 2.1, 3.1},
        {4.1, 5.1, 6.1},
        {7.1, 8.1, 9.1}
    };

    // Initial rotation estimate (identity quaternion)
    double rotation[4] = {1.0, 0.0, 0.0, 0.0};

    // Build the problem
    ceres::Problem problem;

    for (size_t i = 0; i < observed_points.size(); ++i) {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<RotationCostFunctor, 3, 4>(
                new RotationCostFunctor(observed_points[i], target_points[i])),
            nullptr,
            rotation
        );
    }

    // Configure the solver
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    // Solve the problem
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // Output the results
    std::cout << summary.FullReport() << std::endl;
    std::cout << "Estimated rotation (quaternion): [" 
              << rotation[0] << ", " << rotation[1] << ", " << rotation[2] << ", " << rotation[3] << "]" << std::endl;

    // Evaluate the Jacobian
    std::vector<double> residuals;
    std::vector<double*> parameter_blocks = { rotation };
    std::vector<double> jacobian(12); // 3 residuals * 4 parameters

    ceres::Problem::EvaluateOptions eval_options;
    eval_options.parameter_blocks = parameter_blocks;

    double cost;
    problem.Evaluate(eval_options, &cost, &residuals, &jacobian, nullptr);

    std::cout << "Residuals: ";
    for (const auto& residual : residuals) {
        std::cout << residual << " ";
    }
    std::cout << std::endl;

    std::cout << "Jacobian: " << std::endl;
    for (size_t i = 0; i < 3; ++i) { // 3 residuals
        for (size_t j = 0; j < 4; ++j) { // 4 parameters
            std::cout << jacobian[i * 4 + j] << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}
