CMAKE_MINIMUM_REQUIRED ( VERSION 2.8 )

PROJECT ( "SDU-UR10E-Pose-Estimation" )
SET ( PROJECT_VERSION_MAJOR 0 )
SET ( PROJECT_VERSION_MINOR 1 )

SET ( CMAKE_C_COMPILER "gcc" )

find_package(Eigen3 REQUIRED)
find_package(ur_rtde REQUIRED)


INCLUDE_DIRECTORIES (${CMAKE_SOURCE_DIR}/ur10e_force_torque_sensor_test/include
                     /opt/ros/melodic/include
                     ${EIGEN3_INCLUDE_DIR}
                     )

add_compile_options(-std=c++11)


set( SOURCE_FILES 
  ur10e_force_torque_sensor_test/src/ur10e_force_torque_sensor_test.cpp
)

link_directories(/usr/local/lib
                 /opt/ros/melodic/lib)
add_executable(ur10e_force_torque_sensor_test ${SOURCE_FILES})
target_link_libraries(ur10e_force_torque_sensor_test ur_rtde::rtde ur10e_force_torque_sensor sdu_math rt roscpp rosconsole roscpp_serialization)

