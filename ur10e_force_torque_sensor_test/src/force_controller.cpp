/*
 * force_controller.cpp
 *
 *  Created on: May 27, 2020
 *      Author: yik
 */

#include "ur10e_force_torque_sensor_test/force_controller.h"

ForceController::ForceController()
{

}
ForceController::~ForceController()
{

}

void ForceController::initialize(double control_time_)
{
  gain_traj = std::make_shared<CalRad>();
  gain_traj->set_control_time(control_time_);

}

