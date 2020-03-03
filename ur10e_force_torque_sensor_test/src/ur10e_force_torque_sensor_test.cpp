/*
 * ur10e_force_torque_sensor_test.cpp
 *
 *  Created on: Dec 29, 2019
 *      Author: yik
 */
#include "ur10e_force_torque_sensor_test/ur10e_force_torque_sensor_test.h"

ofstream out("test.csv");

void loop_task_proc(void *arg)
{
  RT_TASK *curtask;
  RT_TASK_INFO curtaskinfo;
  int iret = 0;

  RTIME tstart, now;

  curtask = rt_task_self();
  rt_task_inquire(curtask, &curtaskinfo);
  int ctr = 0;

  printf("Starting task %s with period of 2 ms ....\n", curtaskinfo.name);

  //Make the task periodic with a specified loop period
  rt_task_set_periodic(NULL, TM_NOW, LOOP_PERIOD);

  tstart = rt_timer_read();

//  RTDEReceiveInterface rtde_receive("192.168.1.129");
//  std::vector<double> joint_positions        = rtde_receive.getActualQ();
//  std::vector<double> force_data             = rtde_receive.getActualTCPForce();
//  std::vector<double> tool_linear_acc_data   = rtde_receive.getActualToolAccelerometer();
//  std::vector<double> tcp_pose_data         = rtde_receive.getActualTCPPose();
//  std::vector<double> tcp_speed_data         = rtde_receive.getActualTCPSpeed();

//  tool_linear_acc_data  = rtde_receive.getActualToolAccelerometer();
//  tcp_pose_data = rtde_receive.getActualTCPPose();
//  force_data     = rtde_receive.getActualTCPForce();

  // offset initialize function
  for(int num = 1;num < 1001; num ++)
  {
    ros::spinOnce();
    //force_data     = rtde_receive.getActualTCPForce();
    //tool_linear_acc_data  = rtde_receive.getActualToolAccelerometer();
    for(int var = 0; var < 6; var ++)
    {
      //raw_force_torque_data(var,0) = force_data[var];
    }
    //raw_tool_acc_data(0, 0) = tool_linear_acc_data[0];
    //raw_tool_acc_data(1, 0) = tool_linear_acc_data[1];
    //raw_tool_acc_data(2, 0) = tool_linear_acc_data[2];

    ft_filter -> offset_init(raw_force_torque_data, 1000);
    tool_estimation -> offset_init(raw_tool_acc_data, 1000);
    usleep(2000); // 2ms
  }
  //cout <<  tool_estimation->offset_data << "\n\n";

  cout <<"\n--------------------------\n";

  cout <<  ft_filter->get_offset_data() << "\n\n";

  cout <<"\n--------------------------\n";

  cout <<  tool_estimation->get_offset_data() << "\n\n";

  double previous_t = 0.0;

  //Start the task loop
  while(1){
    if((rt_timer_read() - tstart)/1000000.0  - previous_t > 2.15) //2ms
    {
      printf("delayed Loop time: %.5f ms\n",((rt_timer_read() - tstart)/1000000.0  - previous_t) - 2);
    }
    previous_t = (rt_timer_read() - tstart)/1000000.0;
    //    for(int var = 0; var < 6; var ++)
    //    {
    //      //raw_force_torque_data(var,0) = force_data[var];
    //    }
    //    //raw_tool_acc_data(0, 0) = tool_linear_acc_data[0];
    //    //raw_tool_acc_data(1, 0) = tool_linear_acc_data[1];
    //    //raw_tool_acc_data(2, 0) = tool_linear_acc_data[2];

    time_count += 0.002;

    tool_acc_data = raw_tool_acc_data - tool_estimation->get_offset_data();
    ur10e_kinematics->calculate_forward_kinematics(joint_vector);
    //ur10e_kinematics->calculate_forward_kinematics(joint_positions);
    tool_acc_data = ur10e_kinematics->get_tf_base_to_tool(tool_acc_data);
    tool_estimation->set_acc_input_data(tool_acc_data);
    tool_estimation->set_pose_input_data(tcp_pose);
    tool_estimation->get_angular_acc();

    ft_filter->filter_processing(raw_force_torque_data);

    //contacted_force_data = tool_estimation->get_contacted_force(raw_force_torque_data - ft_filter->get_offset_data(), tool_acc_data);

    tool_estimation->get_one_axis_inertia_tensor(ft_filter->get_filtered_data(),"x");
    tool_estimation->get_one_axis_inertia_tensor(ft_filter->get_filtered_data(),"y");
    tool_estimation->get_one_axis_inertia_tensor(ft_filter->get_filtered_data(),"z");
    //
    contacted_force_data = tool_estimation->get_contacted_force(raw_force_torque_data - ft_filter->get_offset_data(), tool_acc_data);
    //
    //    filtered_force_torque_data_msg.data.push_back(tool_acc_data(0,0));
    //    filtered_force_torque_data_msg.data.push_back(raw_force_torque_data(1,0));
    //    filtered_force_torque_data_msg.data.push_back(raw_force_torque_data(2,0));
    //    filtered_force_torque_data_msg.data.push_back(raw_force_torque_data(3,0));
    //    filtered_force_torque_data_msg.data.push_back(raw_force_torque_data(4,0));
    //    filtered_force_torque_data_msg.data.push_back(raw_force_torque_data(5,0));
    //
    //
    //    filtered_force_torque_data_msg.data.push_back(ft_filter->get_filtered_data()(0,0));
    //    filtered_force_torque_data_msg.data.push_back(ft_filter->get_filtered_data()(1,0));
    //    filtered_force_torque_data_msg.data.push_back(ft_filter->get_filtered_data()(2,0));
    //    filtered_force_torque_data_msg.data.push_back(ft_filter->get_filtered_data()(3,0));
    //    filtered_force_torque_data_msg.data.push_back(ft_filter->get_filtered_data()(4,0));
    //    filtered_force_torque_data_msg.data.push_back(ft_filter->get_filtered_data()(5,0));
    //
    //    filtered_force_torque_data_msg.data.push_back(contacted_force_data(0,0));
    //    filtered_force_torque_data_msg.data.push_back(contacted_force_data(1,0));
    //    filtered_force_torque_data_msg.data.push_back(contacted_force_data(2,0));
    //    filtered_force_torque_data_msg.data.push_back(contacted_force_data(3,0));
    //    filtered_force_torque_data_msg.data.push_back(contacted_force_data(4,0));
    //    filtered_force_torque_data_msg.data.push_back(contacted_force_data(5,0));
    //
    //    filtered_force_torque_data_msg.data.push_back(tool_estimation->get_euler_angle()(0,0));
    //    filtered_force_torque_data_msg.data.push_back(tool_estimation->get_euler_angle()(1,0));
    //    filtered_force_torque_data_msg.data.push_back(tool_estimation->get_euler_angle()(2,0));
    //
    //    filtered_force_torque_data_pub.publish(filtered_force_torque_data_msg);
    //    filtered_force_torque_data_msg.data.clear();

    ros::spinOnce();

    for(int num = 0; num<6; num++)
    {
      getActualTCPForce += " "+to_string(raw_force_torque_data(num,0));
      getFilteredForce += " "+to_string(ft_filter->get_filtered_data()(num,0));
      getContactedForceTorque += " "+to_string(contacted_force_data(num,0));
    }
    for(int num = 0; num<3; num++)
    {
      getActualToolAccelerometer += " "+to_string(tool_acc_data(num,0));
      getActualTCPPose += " "+to_string(tcp_pose(num,0));
    }
    for(int num = 0; num<3; num++)
    {
      getActualTCPPose += " "+to_string(tool_estimation->get_euler_angle()(num,0));
    }

    data_line = to_string(time_count)+getActualTCPPose+getActualTCPForce+getFilteredForce+getContactedForceTorque+getActualToolAccelerometer;

    out<<data_line<<endl;

    getActualTCPPose = "";
    getActualTCPForce = "";
    getFilteredForce = "";
    getContactedForceTorque = "";
    getActualToolAccelerometer = "";

    rt_task_wait_period(NULL);
  }
}

void RawForceTorqueDataMsgCallBack(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  if(msg->data.size() < 6)
  {
    raw_force_torque_data(0,0) = 0;
    raw_force_torque_data(1,0) = 0;
    raw_force_torque_data(2,0) = 0;
    raw_force_torque_data(3,0) = 0;
    raw_force_torque_data(4,0) = 0;
    raw_force_torque_data(5,0) = 0;
    return; // some data are missed
  }
  else
  {
    raw_force_torque_data(0,0) = msg->data[0];
    raw_force_torque_data(1,0) = msg->data[1];
    raw_force_torque_data(2,0) = msg->data[2];
    raw_force_torque_data(3,0) = msg->data[3];
    raw_force_torque_data(4,0) = msg->data[4];
    raw_force_torque_data(5,0) = msg->data[5];

    joint_vector[0] = msg->data[18];
    joint_vector[1] = msg->data[19];
    joint_vector[2] = msg->data[20];
    joint_vector[3] = msg->data[21];
    joint_vector[4] = msg->data[22];
    joint_vector[5] = msg->data[23];

    raw_tool_acc_data(0,0) = msg->data[15];
    raw_tool_acc_data(1,0) = msg->data[16];
    raw_tool_acc_data(2,0) = msg->data[17];

    tcp_pose(0,0) = msg->data[24];
    tcp_pose(1,0) = msg->data[25];
    tcp_pose(2,0) = msg->data[26];
    tcp_pose(3,0) = msg->data[27];
    tcp_pose(4,0) = msg->data[28];
    tcp_pose(5,0) = msg->data[29];
  }
}
void ZeroCommandMsgCallBack(const std_msgs::Bool::ConstPtr& msg)
{
  zero_command = true;
}
void initialize()
{
  ft_filter = std::make_shared<FTfilter>();
  tool_estimation = std::make_shared<ToolEstimation>();
  ur10e_kinematics = std::make_shared<Kinematics>();

  control_time = 0.002;
  raw_force_torque_data.resize(6,1);
  raw_force_torque_data.fill(0);

  raw_tool_acc_data.resize(4,1);
  raw_tool_acc_data.fill(0);
  raw_tool_acc_data(3,0)=1;

  tool_acc_data.resize(4,1);
  tool_acc_data.fill(0);
  tool_acc_data(3,0) = 1;

  tcp_pose.resize(6,1);
  tcp_pose.fill(0);

  joint_vector.resize(6);

  //data variables define
  getActualTCPPose = ",actual_tcp_pose_x,actual_tcp_pose_y,actual_tcp_pose_z,actual_tcp_pose_r,actual_tcp_pose_p,actual_tcp_pose_y";
  getActualTCPForce = ",actual_tcp_force_x,actual_tcp_force_y,actual_tcp_force_z,actual_tcp_force_r,actual_tcp_force_p,actual_tcp_force_y";
  getFilteredForce = ",filtered_force_x,filtered_force_y,filtered_force_z,filtered_torque_r,filtered_torque_p,filtered_torque_y";
  getContactedForceTorque = ",contacted_force_x,contacted_force_y,contacted_force_z,contacted_torque_r,contacted_torque_p,contacted_torque_y";
  getActualToolAccelerometer = ",actual_tcp_acc_x,actual_tcp_acc_y,actual_tcp_acc_z";

  data_line = "time"+getActualTCPPose+getActualTCPForce+getFilteredForce+getContactedForceTorque+getActualToolAccelerometer;

  out<<data_line<<endl;

  getActualTCPPose = "";
  getActualTCPForce = "";
  getFilteredForce = "";
  getContactedForceTorque = "";
  getActualToolAccelerometer = "";
}
int main (int argc, char **argv)
{
  printf("Force Torque Sensor Test Node Start \n");
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  filtered_force_torque_data_pub = n.advertise<std_msgs::Float64MultiArray>("/sdu/chatter", 10);

  // ros subsrcibe
  ros::Subscriber raw_force_torque_data_sub;
  ros::Subscriber zero_command_sub;

  //raw_force_torque_data_sub = n.subscribe("/sdu/ur10e/raw_force_torque_data", 10, RawForceTorqueDataMsgCallBack);
  raw_force_torque_data_sub = n.subscribe("/chatter", 10, RawForceTorqueDataMsgCallBack);
  zero_command_sub          = n.subscribe("/sdu/zero", 10, ZeroCommandMsgCallBack);

  //system conffiguration
  std::string init_data_path;
  init_data_path = "../config/init_data.yaml";

  initialize();
  ft_filter->initialize(init_data_path);
  tool_estimation->initialize();
  tool_estimation->set_parameters(control_time, 4.118);
  zero_command = false;

  printf("waiting initialize\n");
  usleep(3000000);

  //real time task
  char str[35];
  mlockall(MCL_CURRENT | MCL_FUTURE); //Lock the memory to avoid memory swapping for this program

  printf("Starting cyclic task...\n");
  sprintf(str, "real time control loop task start");
  rt_task_create(&loop_task, str, 0, 40, 0);//Create the real time task
  rt_task_start(&loop_task, &loop_task_proc, 0);//Since task starts in suspended mode, start task

  //    if(zero_command == true)
  //    {
  //      // offset initialize function
  //      for(int num = 1;num < 1001; num ++)
  //      {
  //        //ros::spinOnce();
  //        //force_data     = rtde_receive.getActualTCPForce();
  //        for(int var = 0; var < 6; var ++)
  //        {
  //          //raw_force_torque_data(var,0) = force_data[var];
  //        }
  //        ft_filter -> offset_init(raw_force_torque_data, 1000);
  //
  //        usleep(2000); // 2ms
  //      }
  //      cout <<"\n--------------------------\n";
  //
  //      cout <<  ft_filter ->get_offset_data() << "\n\n";
  //
  //      zero_command = false;
  //    }

  pause();
  rt_task_delete(&loop_task);

  usleep(3000000);
  out.close();
  cout << "complete and save" << "\n\n";
  printf("exiting safely\n");
  //system("pause");

  return 0;
}
