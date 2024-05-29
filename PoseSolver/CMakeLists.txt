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

# Check if std::format is supported. Thanks GPT!

# Create a test source file
file(WRITE ${CMAKE_BINARY_DIR}/check_format.cpp
"#include <format>
int main() {
    std::string s = std::format(\"{}\", 42);
    return 0;
}
")

# Try to compile the test source file
try_compile(HAS_STD_FORMAT
    ${CMAKE_BINARY_DIR}
    ${CMAKE_BINARY_DIR}/check_format.cpp
)

if(HAS_STD_FORMAT)
    message(STATUS "std::format is supported.")
    add_compile_definitions(HAS_STD_FORMAT)
else()
    message(STATUS "std::format is not supported.")
endif()

find_package(Ceres REQUIRED)
include(GoogleTest)


include_directories(${PROJECT_SOURCE_DIR}/PoseSolver)

add_subdirectory(UnitTests)
add_subdirectory(ThirdPartyTests)