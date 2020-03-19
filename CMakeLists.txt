CMAKE_MINIMUM_REQUIRED ( VERSION 2.8 )

PROJECT ( "SDU-UR10E-Estimation" )
SET ( PROJECT_VERSION_MAJOR 0 )
SET ( PROJECT_VERSION_MINOR 1 )

SET ( CMAKE_C_COMPILER "gcc" )

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
#find_package(Xenomai REQUIRED)



INCLUDE_DIRECTORIES (${CMAKE_SOURCE_DIR}/ur10e_force_torque_sensor_test/include
                     /opt/ros/melodic/include
                     /usr/xenomai/include
                     ${EIGEN3_INCLUDE_DIR}
                     )
                     
#include_directories(/usr/xenomai/include/alchemy/)
#include_directories(/usr/xenomai/include/)
#include_directories(/usr/xenomai/include/cobalt/)
#include_directories(/usr/xenomai/include/boilerplate/)

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
target_link_libraries(ur10e_force_torque_sensor_test rtde sdu_sensor sdu_math rt roscpp rosconsole roscpp_serialization boost_system rostime)
