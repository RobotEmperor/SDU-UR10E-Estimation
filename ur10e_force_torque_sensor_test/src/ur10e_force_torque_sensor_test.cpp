/*
 * ur10e_force_torque_sensor_test.cpp
 *
 *  Created on: Dec 29, 2019
 *      Author: yik
 */

#include "ur10e_force_torque_sensor/ur10e_force_torque_sensor.h"
#include "ur10e_force_torque_sensor_test/ur10e_force_torque_sensor_test.h"

#include <ur_rtde/rtde_receive_interface.h>
#include <unistd.h>
#include <signal.h>
#include <cstdlib>
#include <thread>
#include <time.h>


using namespace ur_rtde;
volatile sig_atomic_t stop;
void inthand(int signum) {
  stop = 1;
}

//void *PrintHello(void *threadid) {
//  long tid;
//  tid = (long)threadid;
//  cout << "Hello World! Thread ID, " << tid << endl;
//  pthread_exit(NULL);
//}

//void controlFunction(const ros::TimerEvent&)
//{
//
//  if(time_count < sampling_time)
//  {
//    time_count += control_time;
//    return;
//  }
//
//  if(offset_check == false)
//  {
//    for(int num = 0;num < 6; num ++)
//    {
//      robot_raw_force_torque_data_msg.data.push_back(raw_force_torque_data(num,0) - ft_sensor->ft_offset_data(num,0));
//      //raw_force_torque_data(num,0) = force_data[num];
//    }
//
//
//    robot_raw_force_torque_data_pub.publish(robot_raw_force_torque_data_msg);
//    robot_raw_force_torque_data_msg.data.clear();
//  }
//
//  ft_sensor->signal_processing(raw_force_torque_data);
//
//  collision_detection->collision_detection_processing(raw_force_torque_data - ft_sensor->ft_offset_data);
//
//  tool_estimation->estimation_processing(ft_sensor->ft_filtered_data);
//
////  ft_sensor->ft_filtered_data(0,0) = tool_estimation ->contacted_force(0,0);
////  ft_sensor->ft_filtered_data(1,0) = tool_estimation ->contacted_force(1,0);
////  ft_sensor->ft_filtered_data(2,0) = tool_estimation ->contacted_force(2,0);
//
//  filtered_force_torque_data_msg.data.push_back(collision_detection->fx_detection);
//  filtered_force_torque_data_msg.data.push_back(collision_detection->fy_detection);
//  filtered_force_torque_data_msg.data.push_back(collision_detection->fz_detection);
//  filtered_force_torque_data_msg.data.push_back(tool_estimation ->contacted_force_torque(0,0));
//  filtered_force_torque_data_msg.data.push_back(tool_estimation ->contacted_force_torque(1,0));
//  filtered_force_torque_data_msg.data.push_back(tool_estimation ->contacted_force_torque(2,0));
//  filtered_force_torque_data_msg.data.push_back(ft_sensor->ft_filtered_data(0,0));
//  filtered_force_torque_data_msg.data.push_back(ft_sensor->ft_filtered_data(1,0));
//  filtered_force_torque_data_msg.data.push_back(ft_sensor->ft_filtered_data(2,0));
//
//  filtered_force_torque_data_pub.publish(filtered_force_torque_data_msg);
//  filtered_force_torque_data_msg.data.clear();
//}
//void RawForceTorqueDataMsgCallBack(const std_msgs::Float64MultiArray::ConstPtr& msg)
//{
//  return;
//
//  if(msg->data.size() < 6)
//    return; // some data is missed
//  else
//  {
//    raw_force_torque_data(0,0) = msg->data[0];
//    raw_force_torque_data(1,0) = msg->data[1];
//    raw_force_torque_data(2,0) = msg->data[2];
//    raw_force_torque_data(3,0) = msg->data[3];
//    raw_force_torque_data(4,0) = msg->data[4];
//    raw_force_torque_data(5,0) = msg->data[5];
//  }
//
//}

timer_t firstTimerID;

static void timerHandler( int sig, siginfo_t *si, void *uc )
{
  timer_t *tidp;
  tidp = (void**)(si->si_value.sival_ptr);

  if(time_count < sampling_time)
  {
    time_count += control_time;
    return;
  }

  //ft_sensor->signal_processing(raw_force_torque_data);

  //collision_detection->collision_detection_processing(raw_force_torque_data - ft_sensor->ft_offset_data);

  tool_estimation->estimation_processing(ft_sensor->ft_filtered_data);

  //  ft_sensor->ft_filtered_data(0,0) = tool_estimation ->contacted_force(0,0);
  //  ft_sensor->ft_filtered_data(1,0) = tool_estimation ->contacted_force(1,0);
  //  ft_sensor->ft_filtered_data(2,0) = tool_estimation ->contacted_force(2,0);
}


static int makeTimer( timer_t *timerID, int expireMS, int intervalMS )

{

  struct sigevent te;

  struct itimerspec its;

  struct sigaction sa;

  int sigNo = SIGRTMIN;

  /* Set up signal handler. */

  sa.sa_flags = SA_SIGINFO;

  sa.sa_sigaction = timerHandler;

  sigemptyset(&sa.sa_mask);

  if (sigaction(sigNo, &sa, NULL) == -1) {

    perror("sigaction");

  }
  /* Set and enable alarm */

  te.sigev_notify = SIGEV_SIGNAL;

  te.sigev_signo = sigNo;

  te.sigev_value.sival_ptr = timerID;

  timer_create(CLOCK_REALTIME, &te, timerID);

  its.it_interval.tv_sec = 0;

  its.it_interval.tv_nsec = intervalMS * 1000000;

  its.it_value.tv_sec = 0;

  its.it_value.tv_nsec = expireMS * 1000000;

  timer_settime(*timerID, 0, &its, NULL);

  return 1;
}

int main (int argc, char **argv)
{
  printf("Force Torque Sensor Test Node Start \n");

  control_time = 0.002;
  sampling_time = 2.0;

  ft_sensor = new Ur10eFTsensor;
  tool_estimation = new PoseEstimation;
  ur10e_kinematics = new Kinematics;
  collision_detection = new CollisionDetection;


  // ft sensor variables init
  ft_sensor->control_time     = control_time;
  ft_sensor->lpf_force_cutoff_frequency  = 3;
  ft_sensor->lpf_torque_cutoff_frequency = 3;

  ft_sensor->hpf_force_cutoff_frequency  = 10;
  ft_sensor->hpf_torque_cutoff_frequency = 10;

  ft_sensor->initialize();

  //tool_estimation ->initialize();


  //collision detection variables init
  collision_detection->fx_k = 1;
  collision_detection->fx_high_limit = 12;
  collision_detection->fx_low_limit = -22;

  collision_detection->fy_k = 0;
  collision_detection->fy_high_limit = 0;
  collision_detection->fy_low_limit = 0;

  collision_detection->fz_k = 0;
  collision_detection->fz_high_limit = 0;
  collision_detection->fz_low_limit = 0;


  // offset variables init
  offset_check = true;
  time_count =0.0;

  //  RTDEReceiveInterface rtde_receive("192.168.1.129");
  //  std::vector<double> joint_positions        = rtde_receive.getActualQ();
  //  std::vector<double> force_data             = rtde_receive.getActualTCPForce();
  //  std::vector<double> tool_linear_acc_data   = rtde_receive.getActualToolAccelerometer();
  //  std::vector<double> tool_pose_data         = rtde_receive.getActualTCPPose();
  //
  //
  //
  //  tool_linear_acc_data  = rtde_receive.getActualToolAccelerometer();
  //  tool_pose_data = rtde_receive.getActualTCPPose();
  //  force_data     = rtde_receive.getActualTCPForce();
  //  joint_positions        = rtde_receive.getActualQ();

  // ur10e_kinematics->joint_positions = joint_positions;
  //
  //  for(int num = 0;num < 6; num ++)
  //  {
  //    //robot_raw_force_torque_data_msg.data.push_back(force_data[num]);
  //    //raw_force_torque_data(num,0) = force_data[num];
  //  }


  makeTimer(&firstTimerID, 2, 2); //2ms
  signal(SIGINT, inthand);

  while (!stop)
  {
    usleep(100);
    //printf("loop\n");

  }


  timer_delete(&firstTimerID);
  printf("exiting safely\n");
  system("pause");



  //  ur10e_kinematics->temp_data(0,0) = tool_linear_acc_data[0];
  //  ur10e_kinematics->temp_data(1,0) = tool_linear_acc_data[1];
  //  ur10e_kinematics->temp_data(2,0) = tool_linear_acc_data[2];
  //  ur10e_kinematics->calculate_forward_kinematics(6,ur10e_kinematics->joint_positions);
  //  ur10e_kinematics->temp_data = ur10e_kinematics->transformation_result*ur10e_kinematics->temp_data;
  //
  //  tool_estimation -> tool_linear_acc_data(0,0) = ur10e_kinematics->temp_data(0,0);
  //  tool_estimation -> tool_linear_acc_data(1,0) = ur10e_kinematics->temp_data(1,0);
  //  tool_estimation -> tool_linear_acc_data(2,0) = ur10e_kinematics->temp_data(2,0);

  //    // offset initialize
  //    if(time_count < sampling_time)
  //    {
  //      offset_check = true;
  //      ft_sensor->offset_init(raw_force_torque_data, offset_check);
  //      tool_estimation->offset_init(tool_estimation -> tool_linear_acc_data, offset_check);
  //    }
  //    if(time_count > sampling_time && offset_check == true)
  //    {
  //      offset_check = false;
  //      ft_sensor->offset_init(raw_force_torque_data, offset_check);
  //      tool_estimation->offset_init(tool_estimation -> tool_linear_acc_data, offset_check);
  //
  //      cout << tool_estimation->offset_data << "\n\n";
  //      cout << ur10e_kinematics->transformation_result << "\n\n";
  //      cout << tool_pose_data[0] << "\n\n";
  //      cout << tool_pose_data[1] << "\n\n";
  //      cout << tool_pose_data[2] << "\n\n";

  cout << "complete" << "\n\n";

  delete ft_sensor;
  delete tool_estimation;
  delete ur10e_kinematics;
  delete collision_detection;
  return 0;
}
