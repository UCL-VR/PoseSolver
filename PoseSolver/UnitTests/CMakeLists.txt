
add_executable(UnitTests
./rodrigues.cpp
./test.cpp
./mockImu.cpp
./imu.cpp
./pch.cpp
./pose.cpp
./denavitHartenberg.cpp
)

target_link_libraries(
  UnitTests
  GTest::gtest_main
  Ceres::ceres
)