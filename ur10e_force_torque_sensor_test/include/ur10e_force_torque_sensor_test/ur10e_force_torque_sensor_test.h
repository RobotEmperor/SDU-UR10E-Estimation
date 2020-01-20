/*
 * ur10e_force_torque_sensor_test.h
 *
 *  Created on: Dec 29, 2019
 *      Author: yik
 */

#ifndef SDU_UR10E_POSE_ESTIMATION_UR10E_FORCE_TORQUE_SENSOR_TEST_INCLUDE_UR10E_FORCE_TORQUE_SENSOR_TEST_UR10E_FORCE_TORQUE_SENSOR_TEST_H_
#define SDU_UR10E_POSE_ESTIMATION_UR10E_FORCE_TORQUE_SENSOR_TEST_INCLUDE_UR10E_FORCE_TORQUE_SENSOR_TEST_UR10E_FORCE_TORQUE_SENSOR_TEST_H_


#include <Eigen/Dense>

// file load
#include <fstream>
#include <string>
#include <iostream>
#include <sstream>



using namespace std;

Ur10eFTsensor *ft_sensor;
PoseEstimation *tool_estimation;
Kinematics *ur10e_kinematics;
CollisionDetection *collision_detection;

double control_time;
double sampling_time;


bool offset_check;
double  time_count;




#endif /* SDU_UR10E_POSE_ESTIMATION_UR10E_FORCE_TORQUE_SENSOR_TEST_INCLUDE_UR10E_FORCE_TORQUE_SENSOR_TEST_UR10E_FORCE_TORQUE_SENSOR_TEST_H_ */
