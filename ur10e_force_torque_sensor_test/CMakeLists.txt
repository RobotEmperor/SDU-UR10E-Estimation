CMAKE_MINIMUM_REQUIRED ( VERSION 2.8 )

PROJECT ( "SDU-UR10E-Pose-Estimation" )
SET ( PROJECT_VERSION_MAJOR 0 )
SET ( PROJECT_VERSION_MINOR 1 )

SET ( CMAKE_C_COMPILER "gcc" )


INCLUDE_DIRECTORIES (ur10e_force_torque_sensor_test/include)

add_compile_options(-std=c++11)
 
find_package(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES (${EIGEN3_INCLUDE_DIR})

find_package(ur_rtde REQUIRED)
find_package(ur10e_force_torque_sensor REQUIRED)

set( SOURCE_FILES 
  ur10e_force_torque_sensor_test/src/ur10e_force_torque_sensor_test.cpp
)

add_executable(ur10e_force_torque_sensor_test ${SOURCE_FILES})
target_link_libraries(ur10e_force_torque_sensor_test ur_rtde::rtde ur10e_force_torque_sensor sdu_math sensor_filter rt)
