/*
 * task_motion.cpp
 *
 *  Created on: May 15, 2020
 *      Author: yik
 */

#include "ur10e_force_torque_sensor_test/task_motion.h"


TaskMotion::TaskMotion()
{

}
TaskMotion::~TaskMotion()
{

}

void TaskMotion::initialize(double control_time_)
{
  ur10e_traj = std::make_shared<CalRad>();
  ur10e_traj->set_control_time(control_time_);

  number_of_point = 0;

  // initial pose load
  desired_pose_matrix(0,7) = 5;
  desired_pose_matrix(1,7) = 5;
  desired_pose_matrix(2,7) = 5;
  desired_pose_matrix(3,7) = 5;
  desired_pose_matrix(4,7) = 5;
  desired_pose_matrix(5,7) = 5;

  current_pose_vector.resize(6);

  current_pose_vector[0] = -0.5;
  current_pose_vector[1] = 0.184324;
  current_pose_vector[2] = 0.5875;
  current_pose_vector[3] = -180*DEGREE2RADIAN;
  current_pose_vector[4] = 0;
  current_pose_vector[5] = 0;

  for(int num = 0; num <6 ; num ++)
  {
    desired_pose_matrix(num,0) =  current_pose_vector[num];
    desired_pose_matrix(num,1) =  current_pose_vector[num];
    ur10e_traj->current_pose_change(num,0) = current_pose_vector[num];
  }

  ur10e_traj->cal_end_point_tra_px->current_pose = current_pose_vector[0];
  ur10e_traj->cal_end_point_tra_py->current_pose = current_pose_vector[1];
  ur10e_traj->cal_end_point_tra_pz->current_pose = current_pose_vector[2];
  ur10e_traj->cal_end_point_tra_alpha->current_pose = -180*DEGREE2RADIAN;
}


void TaskMotion::robot_initialize() // joint space
{

}
void TaskMotion::load_task_motion(std::string path_)
{
  YAML::Node doc; //
  try
  {
    // load yaml
    doc = YAML::LoadFile(path_.c_str());
  }catch(const std::exception& e)
  {
    std::cout << COLOR_RED_BOLD << "Fail to load LEG parse data yaml file!" << COLOR_RESET << std::endl;
    return;
  }

  // motion data load initialize//
  YAML::Node motion_start_time_node = doc["motion_start_time"];
  YAML::Node motion_task_node = doc["motion_task"];

  std::vector<double> temp_motion_start_time_vector;
  std::vector<double> temp_motion_task_vector;

  for (YAML::iterator it = motion_start_time_node.begin(); it != motion_start_time_node.end(); ++it)
  {
    number_of_point = it->first.as<int>();
    temp_motion_start_time_vector.push_back(it->second[0].as<double>());
    motion_start_time_vector[number_of_point] = temp_motion_start_time_vector;
    temp_motion_start_time_vector.clear();
  }
  for (YAML::iterator it = motion_task_node.begin(); it != motion_task_node.end(); ++it)
  {
    number_of_point = it->first.as<int>();
    for(int num = 0; num < 3; num++)
    {
      temp_motion_task_vector.push_back(it->second[num].as<double>());
    }
    for(int num = 3; num < 6; num++)
    {
      temp_motion_task_vector.push_back(it->second[num].as<double>()*DEGREE2RADIAN);
    }
    motion_task_vector[number_of_point] = temp_motion_task_vector;
    temp_motion_task_vector.clear();
  }
}
void TaskMotion::run_task_motion()
{
  ur10e_traj->cal_end_point_to_rad(desired_pose_matrix);
  for(int num = 0; num <6 ; num ++)
  {
    current_pose_vector[num] = ur10e_traj->get_traj_results()(num,0);
  }
}
std::vector<double> TaskMotion::get_current_pose()
{
  return current_pose_vector;
}
void TaskMotion::clear_task_motion()
{
  motion_start_time_vector.clear();
  motion_task_vector.clear();
}

