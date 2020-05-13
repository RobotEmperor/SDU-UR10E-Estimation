CMAKE_MINIMUM_REQUIRED ( VERSION 2.8 )

PROJECT ( "SDU-UR10E-Estimation" )

set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)

set(RW_ROOT "$ENV{RW_ROOT}")
find_package(RobWork REQUIRED PATHS "${RW_ROOT}")



INCLUDE_DIRECTORIES (${CMAKE_SOURCE_DIR}/ur10e_force_torque_sensor_test/include
                     /opt/ros/melodic/include
                     /usr/xenomai/include
                     ${EIGEN3_INCLUDE_DIR}
                     ${ROBWORK_INCLUDE_DIR}
                     )
                     


execute_process(COMMAND /usr/xenomai/bin/xeno-config --skin=alchemy --cflags OUTPUT_VARIABLE XENO_ALCHENMY_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND /usr/xenomai/bin/xeno-config --skin=alchemy --ldflags OUTPUT_VARIABLE XENO_ALCHENMY_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${XENO_ALCHENMY_CFLAGS} ${XENO_ALCHENMY_LDFLAGS}")


set( SOURCE_FILES 
  ur10e_force_torque_sensor_test/src/ur10e_force_torque_sensor_test.cpp
)

#INCLUDE_DIRECTORIES (${CMAKE_SOURCE_DIR}/ur10e_force_torque_sensor_test/include
#                     /opt/ros/kinetic/include
#                     ${EIGEN3_INCLUDE_DIR}
#                     )



link_directories(/usr/local/lib
                 /opt/ros/melodic/lib)

#link_directories(/usr/local/lib
#                 /opt/ros/kinetic/lib)

add_executable(ur10e_force_torque_sensor_test ${SOURCE_FILES})
target_link_libraries(ur10e_force_torque_sensor_test rtde sdu_sensor sdu_math rt roscpp rosconsole roscpp_serialization boost_system rostime ${ROBWORK_LIBRARIES})
