cmake_minimum_required(VERSION 3.14)
project(RobotLegProject CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find the Eigen library
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Add moteus library from GitHub using FetchContent
include(FetchContent)
FetchContent_Declare(
  moteus
  GIT_REPOSITORY https://github.com/mjbots/moteus.git
  GIT_TAG        ee4f6ac9c6391523876836fcf6f276411b7f3670
)
FetchContent_MakeAvailable(moteus)

# Add project include directory
include_directories(include)

# --- Define application executables ---

# Test 1: Passive state reporting
add_executable(
    passive_test_app
    projects/01_passive_test/main.cpp
    src/kinematics.cpp
    src/virtual_leg_controller.cpp
)

# Test 2: Virtual leg controller test
add_executable(
    virtual_leg_test_app
    projects/02_virtual_leg_test/main.cpp
    src/kinematics.cpp
    src/virtual_leg_controller.cpp
)

# App 3: Hopping controller
add_executable(
    hopping_app
    projects/03_hopping/main.cpp
    src/kinematics.cpp
    src/virtual_leg_controller.cpp
)


# --- Link libraries to all targets ---
foreach(app
    passive_test_app
    virtual_leg_test_app
    hopping_app
)
    target_link_libraries(${app} PRIVATE
        Eigen3::Eigen
        moteus
    )
    target_include_directories(${app} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
endforeach()