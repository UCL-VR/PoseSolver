add_executable(ceres_jacobian_test ceres_jacobian_test.cpp)

target_link_libraries(
  ceres_jacobian_test
  Ceres::ceres
)