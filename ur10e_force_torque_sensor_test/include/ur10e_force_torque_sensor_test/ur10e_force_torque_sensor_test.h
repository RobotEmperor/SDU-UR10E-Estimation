/*
 * ur10e_force_torque_sensor_test.h
 *
 *  Created on: Dec 29, 2019
 *      Author: yik
 */

#ifndef UR10E_FORCE_TORQUE_SENSOR_TEST_H_
#define UR10E_FORCE_TORQUE_SENSOR_TEST_H_


#include <Eigen/Dense>

// file load
#include <fstream>
#include <string>
#include <iostream>
#include <sstream>
#include <stdio.h>

#include <ur_rtde/rtde_receive_interface.h>
#include <unistd.h>
#include <signal.h>
#include <cstdlib>
//ros message system
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"

#include "sdu_sensor/ft_filter.h"
#include "sdu_sensor/tool_estimation.h"

using namespace std;

std::shared_ptr<FTfilter> ft_filter;
std::shared_ptr<ToolEstimation> tool_estimation;
std::shared_ptr<Kinematics> ur10e_kinematics;

double control_time;
double sampling_time;

Eigen::MatrixXd raw_force_torque_data;

Eigen::MatrixXd raw_tool_acc_data;
Eigen::MatrixXd tool_acc_data;
Eigen::MatrixXd contacted_force_data;

ros::Publisher filtered_force_torque_data_pub;
std_msgs::Float64MultiArray filtered_force_torque_data_msg;

void RawForceTorqueDataMsgCallBack(const std_msgs::Float64MultiArray::ConstPtr& msg);
void ZeroCommandMsgCallBack(const std_msgs::Bool::ConstPtr& msg);

//for ros test
std::vector<double> joint_vector;
bool zero_command;

#endif /* UR10E_FORCE_TORQUE_SENSOR_TEST_H_ */
