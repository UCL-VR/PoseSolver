cmake_minimum_required(VERSION 3.14) # Adjust minimum version as necessary
project(TrackerLib VERSION 1.0.0)

# Optionally set C++ standard globally
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

find_package(Ceres REQUIRED)
include(GoogleTest)


include_directories(${PROJECT_SOURCE_DIR}/PoseSolver)

# Third party dependencies
add_subdirectory(UnitTests)
