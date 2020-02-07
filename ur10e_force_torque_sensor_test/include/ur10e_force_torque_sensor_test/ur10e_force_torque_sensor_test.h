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
#include <stdio.h>

//ros message system
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"



using namespace std;

Ur10eFTsensor *ft_sensor;
PoseEstimation *tool_estimation;
Kinematics *ur10e_kinematics;


double control_time;
double sampling_time;

Eigen::MatrixXd raw_force_torque_data;
Eigen::MatrixXd tool_acc_data;

ros::Publisher filtered_force_torque_data_pub;
std_msgs::Float64MultiArray filtered_force_torque_data_msg;

void RawForceTorqueDataMsgCallBack(const std_msgs::Float64MultiArray::ConstPtr& msg);
void ZeroCommandMsgCallBack(const std_msgs::Bool::ConstPtr& msg);

//for ros test

double joint[6];
bool zero_command;



#endif /* SDU_UR10E_POSE_ESTIMATION_UR10E_FORCE_TORQUE_SENSOR_TEST_INCLUDE_UR10E_FORCE_TORQUE_SENSOR_TEST_UR10E_FORCE_TORQUE_SENSOR_TEST_H_ */
