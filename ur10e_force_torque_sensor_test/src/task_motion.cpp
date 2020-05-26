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
  robot_traj = std::make_shared<CalRad>();
  robot_traj->set_control_time(control_time_);

  current_point = -1; // wanna count from 0
  all_point = -1;
  check_change = false;
  task_done = false;

  desired_pose_matrix.resize(6,8);
  desired_pose_matrix.fill(0);

  // initial pose load
  desired_pose_matrix(0,7) = 5;
  desired_pose_matrix(1,7) = 5;
  desired_pose_matrix(2,7) = 5;
  desired_pose_matrix(3,7) = 5;
  desired_pose_matrix(4,7) = 5;
  desired_pose_matrix(5,7) = 5;

  current_pose_vector.resize(6);
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
    std::cout << COLOR_RED_BOLD << "Fail to load data, yaml file!" << COLOR_RESET << std::endl;
    return;
  }

  // motion data load initialize//
  YAML::Node motion_start_time_node = doc["motion_start_time"];
  YAML::Node motion_task_node = doc["motion_task"];

  std::vector<double> temp_motion_start_time_vector;
  std::vector<double> temp_motion_task_pose_vector;
  std::vector<double> temp_motion_task_vel_vector;

  int point_numbers;
  point_numbers = 0;

  //time
  for (YAML::iterator it = motion_start_time_node.begin(); it != motion_start_time_node.end(); ++it)
  {
    point_numbers = it->first.as<int>();
    temp_motion_start_time_vector.push_back(it->second[0].as<double>());
    motion_start_time_vector[point_numbers] = temp_motion_start_time_vector;
    temp_motion_start_time_vector.clear();
  }
  //points
  for (YAML::iterator it = motion_task_node.begin(); it != motion_task_node.end(); ++it)
  {
    point_numbers = it->first.as<int>();
    for(int num = 0; num < 3; num++)
    {
      temp_motion_task_pose_vector.push_back(it->second[num].as<double>());
    }
    for(int num = 3; num < 6; num++)
    {
      temp_motion_task_pose_vector.push_back(it->second[num].as<double>()*DEGREE2RADIAN);
    }
    motion_task_pose_vector[point_numbers] = temp_motion_task_pose_vector;
    temp_motion_task_pose_vector.clear();

    all_point ++;
  }
  //velocity
  for(int num = 0; num < 6; num++)
    temp_motion_task_vel_vector.push_back(0);

  for(int num = 0; num < all_point+1 ; num++)
  {
    motion_task_init_vel_vector[num] = temp_motion_task_vel_vector;
    motion_task_final_vel_vector[num] = temp_motion_task_vel_vector;
  }
  temp_motion_task_vel_vector.clear();

  std::cout << "LOAD Cmplete" << std::endl;
}
void TaskMotion::run_task_motion()
{
  //  robot_traj->cal_end_point_to_rad(desired_pose_matrix);
  //  for(int num = 0; num <6 ; num ++)
  //  {
  //    current_pose_vector[num] = robot_traj->get_traj_results()(num,0);
  //  }
  //

  if(all_point == -1)
    return;

  if(robot_traj->is_moving_check !=true)// not during motion --> can recieve first or new point.
  {
    if(task_done)
      return;

    if(robot_traj->is_moving_check != check_change) // point change is detected.
    {
      current_point ++;
      std::cout << current_point << std::endl;
    }
    else
      current_point = 0; //or stop the robot

    if(current_point > all_point)
    {
      check_change = robot_traj->is_moving_check;
      task_done = true;
      return;
    }

    if(current_point == -1)
       return;

    calculate_init_final_velocity(current_point);

    for(int num = 0; num <6; num ++)
    {
      desired_pose_matrix(num,1) = motion_task_pose_vector[current_point][num];

      desired_pose_matrix(num,2) = motion_task_init_vel_vector[0][num];
      desired_pose_matrix(num,3) = motion_task_final_vel_vector[current_point][num];

      desired_pose_matrix(num,7) = motion_start_time_vector[current_point][0];
    }

    // change the point
  }
  else // during motion
  {

  }
  check_change = robot_traj->is_moving_check;
}
void TaskMotion::generate_trajectory()
{
  robot_traj->cal_end_point_to_rad(desired_pose_matrix);
  for(int num = 0; num <6 ; num ++)
  {
    current_pose_vector[num] = robot_traj->get_traj_results()(num,0);
  }
}
std::vector<double> TaskMotion::get_current_pose()
{
  return current_pose_vector;
}
void TaskMotion::set_initial_pose(double x, double y, double z, double roll, double pitch, double yaw)
{

  current_pose_vector[0] = x;
  current_pose_vector[1] = y;
  current_pose_vector[2] = z;
  current_pose_vector[3] = roll;
  current_pose_vector[4] = pitch;
  current_pose_vector[5] = yaw;

  for(int num = 0; num <6 ; num ++)
  {
    desired_pose_matrix(num,0) =  current_pose_vector[num];
    desired_pose_matrix(num,1) =  current_pose_vector[num];
    robot_traj->current_pose_change(num,0) = current_pose_vector[num];
  }

  robot_traj->cal_end_point_tra_px->current_pose = current_pose_vector[0];
  robot_traj->cal_end_point_tra_py->current_pose = current_pose_vector[1];
  robot_traj->cal_end_point_tra_pz->current_pose = current_pose_vector[2];
  robot_traj->cal_end_point_tra_alpha->current_pose = current_pose_vector[3];
  robot_traj->cal_end_point_tra_betta->current_pose = current_pose_vector[4];
  robot_traj->cal_end_point_tra_kamma->current_pose = current_pose_vector[5];
}
void TaskMotion::set_point(double x, double y, double z, double roll, double pitch, double yaw, double time)
{
  desired_pose_matrix(0,1) = x;
  desired_pose_matrix(1,1) = y;
  desired_pose_matrix(2,1) = z;
  desired_pose_matrix(3,1) = roll;
  desired_pose_matrix(4,1) = pitch;
  desired_pose_matrix(5,1) = yaw;

  for(int num = 0; num <6 ; num ++)
  {
    desired_pose_matrix(num,7) = time;
  }
}
void TaskMotion::clear_task_motion()
{
  task_done = false;
  all_point = -1;
  current_point = -1;
  motion_start_time_vector.clear();
  motion_task_pose_vector.clear();
}

double TaskMotion::calculate_velocity(double first_point,double second_point, double interval_time)
{
  return (second_point - first_point)/interval_time;
}
void TaskMotion::calculate_init_final_velocity(int point_number)
{
  static double first_vel = 0;
  static double second_vel = 0;

  if(point_number < 0)
    return;

  if(all_point < 1) // in case of one point
  {
    for(int var = 0; var <6 ; var++)
    {
      motion_task_init_vel_vector[0][var] = 0;
      motion_task_final_vel_vector[0][var] = 0;
    }
    return;
  }

  if(point_number == 0) // start phase
  {
    for(int var = 0; var <6 ; var++)
    {
      first_vel  = calculate_velocity(current_pose_vector[var],motion_task_pose_vector[0][var], motion_start_time_vector[0][0]);
      second_vel = calculate_velocity(motion_task_pose_vector[0][var],motion_task_pose_vector[1][var], motion_start_time_vector[1][0]);

      motion_task_init_vel_vector[0][var] = 0;
      motion_task_final_vel_vector[0][var] = calculate_next_velocity(first_vel, second_vel);
    }
    return;
  }
  if(point_number == all_point) // final phase
  {
    for(int var = 0; var <6 ; var++)
    {
      motion_task_init_vel_vector[point_number][var] = motion_task_final_vel_vector[point_number-1][var];
      motion_task_final_vel_vector[point_number][var] = 0;
    }
    return;
  }

  // medium phase
  for(int var = 0; var <6 ; var++)
  {
    first_vel  = calculate_velocity(motion_task_pose_vector[point_number-1][var],motion_task_pose_vector[point_number][var], motion_start_time_vector[point_number][0]); //
    second_vel = calculate_velocity(motion_task_pose_vector[point_number][var],motion_task_pose_vector[point_number+1][var], motion_start_time_vector[point_number+1][0]); //

    motion_task_init_vel_vector[point_number][var] = motion_task_final_vel_vector[point_number-1][var];
    motion_task_final_vel_vector[point_number][var] = calculate_next_velocity(first_vel, second_vel);
  }

}
double TaskMotion::calculate_next_velocity(double first_vel, double second_vel)
{
  if(first_vel*second_vel > 0)
    return second_vel;
  else
    return 0;
}

