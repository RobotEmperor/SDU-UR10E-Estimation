/*
 * ur10e_force_torque_sensor_test.cpp
 *
 *  Created on: Dec 29, 2019
 *      Author: yik
 */
#include "ur10e_force_torque_sensor_test/ur10e_force_torque_sensor_test.h"
#include "ur10e_force_torque_sensor_test/log.h"
#define WC_FILE "/home/yik/sdu_ws/SDU-UR10E-Estimation/wc/UR10e_2018/UR10e.xml"

ofstream out("test.csv");
const WorkCell::Ptr wc = WorkCellLoader::Factory::load(WC_FILE);
const SerialDevice::Ptr device = wc->findDevice<SerialDevice>("UR10e");

void loop_task_proc(void *arg)
{
  if (wc.isNull())
    RW_THROW("WorkCell could not be loaded.");
  if (device.isNull())
    RW_THROW("UR10e device could not be found.");

  const State state = wc->getDefaultState();
  const ClosedFormIKSolverUR solver(device, state);
  double previous_t = 0.0;


  RT_TASK *curtask;
  RT_TASK_INFO curtaskinfo;

  RTIME tstart;

  curtask = rt_task_self();
  rt_task_inquire(curtask, &curtaskinfo);

  printf("Starting task %s with period of %f ms ....\n", curtaskinfo.name, control_time*1000);

  //Make the task periodic with a specified loop period
  rt_task_set_periodic(NULL, TM_NOW, LOOP_PERIOD);
  tstart = rt_timer_read();

  if(!gazebo_check)
  {
    std::cout << COLOR_GREEN_BOLD << "Real robot start " << COLOR_RESET << std::endl;
    force_data            = rtde_receive->getActualTCPForce();
    tool_linear_acc_data  = rtde_receive->getActualToolAccelerometer();
    tcp_pose_data         = rtde_receive->getActualTCPPose();

    tool_acc_data(0,0) = tool_linear_acc_data[0];
    tool_acc_data(1,0) = tool_linear_acc_data[1];
    tool_acc_data(2,0) = tool_linear_acc_data[2];

    tf_current = Transform3D<> (Vector3D<>(tcp_pose_data[0], tcp_pose_data[1], tcp_pose_data[2]), EAA<>(tcp_pose_data[3], tcp_pose_data[4], tcp_pose_data[5]).toRotation3D());
    for(int num_row = 0; num_row < 4; num_row ++)
    {
      for(int num_col = 0; num_col < 4; num_col ++)
      {
        tf_current_matrix(num_row,num_col) = tf_current(num_row,num_col);
      }
    }

    tool_estimation->set_orientation_data(tf_current_matrix);
    tool_estimation->set_gravity_input_data(tool_acc_data.block(0,0,3,1));

    cout << tool_acc_data.block(0,0,3,1) << endl;

    usleep(3000000);


    while(1){
      if((rt_timer_read() - tstart)/1000000.0  - previous_t > (control_time*1000 + 0.15)) //
      {
        printf("delayed Loop time: %.5f ms\n",((rt_timer_read() - tstart)/1000000.0  - previous_t) - control_time*1000);
      }
      previous_t = (rt_timer_read() - tstart)/1000000.0;
      force_data            = rtde_receive->getActualTCPForce();
      tool_linear_acc_data  = rtde_receive->getActualToolAccelerometer();
      tcp_pose_data         = rtde_receive->getActualTCPPose();

      // data types will be changed
      for(int var = 0; var < 6; var ++)
      {
        raw_force_torque_data(var,0) = force_data[var];
        tcp_pose(var,0) = tcp_pose_data[var];
      }
      tool_acc_data(0,0) = -tool_linear_acc_data[0];
      tool_acc_data(1,0) = -tool_linear_acc_data[1];
      tool_acc_data(2,0) = -tool_linear_acc_data[2];

      tf_current = Transform3D<> (Vector3D<>(tcp_pose_data[0], tcp_pose_data[1], tcp_pose_data[2]), EAA<>(tcp_pose_data[3], tcp_pose_data[4], tcp_pose_data[5]).toRotation3D());
      for(int num_row = 0; num_row < 4; num_row ++)
      {
        for(int num_col = 0; num_col < 4; num_col ++)
        {
          tf_current_matrix(num_row,num_col) = tf_current(num_row,num_col);
        }
      }

      tool_estimation->set_orientation_data(tf_current_matrix);
      tool_acc_data = tf_current_matrix*tool_acc_data;

      contacted_force_data = tool_estimation->get_estimated_force(raw_force_torque_data, tool_acc_data.block(0,0,3,1));

      raw_force_torque_data_msg.data.push_back(raw_force_torque_data(0,0));
      raw_force_torque_data_msg.data.push_back(raw_force_torque_data(1,0));
      raw_force_torque_data_msg.data.push_back(raw_force_torque_data(2,0));
      filtered_force_torque_data_msg.data.push_back(contacted_force_data(0,0));
      filtered_force_torque_data_msg.data.push_back(contacted_force_data(1,0));
      filtered_force_torque_data_msg.data.push_back(contacted_force_data(2,0));

      raw_force_torque_data_pub.publish(raw_force_torque_data_msg);
      filtered_force_torque_data_pub.publish(filtered_force_torque_data_msg);
      filtered_force_torque_data_msg.data.clear();
      raw_force_torque_data_msg.data.clear();

      // controller algorithm
      //ur10e_task->run_task_motion();

      // have to add force controller

      //
      //desired_pose_vector = ur10e_task->get_current_pose();

      //double x,y,z;
      //x = ur10e_kinematics->get_rotation_matrix_to_axis(ur10e_kinematics->desired_rotation_matrix_xd(desired_pose_vector[3],desired_pose_vector[4],desired_pose_vector[5]))(1,0);
      //y = ur10e_kinematics->get_rotation_matrix_to_axis(ur10e_kinematics->desired_rotation_matrix_xd(desired_pose_vector[3],desired_pose_vector[4],desired_pose_vector[5]))(2,0);
      //z = ur10e_kinematics->get_rotation_matrix_to_axis(ur10e_kinematics->desired_rotation_matrix_xd(desired_pose_vector[3],desired_pose_vector[4],desired_pose_vector[5]))(3,0);
      //
      ////
      //const Transform3D<> Tdesired(Vector3D<>(desired_pose_vector[0], desired_pose_vector[1], desired_pose_vector[2]),
      //    EAA<>(x, y, z).toRotation3D());
      //const std::vector<Q> solutions = solver.solve(Tdesired, state);

      //ur10e_kinematics->calculate_forward_kinematics(solutions[3].toStdVector());

      //for(int num= 0; num  < 6; num ++)
      //  joint_cur_value_msg.data.push_back(solutions[3].toStdVector()[num]);
      //
      //for(int num= 0; num  < 3; num ++)
      //  ee_cur_value_msg.data.push_back(ur10e_kinematics->get_tf_base_to_tool()(num,3));
      //
      //for(int num= 0; num  < 3; num ++)
      //  ee_cur_value_msg.data.push_back(ur10e_kinematics->get_axis_to_euler_angle(x,y,z)(num,0)*RADIAN2DEGREE);


      //      for(int num = 0; num<6; num++)
      //      {
      //        getActualTCPForce += " "+to_string(raw_force_torque_data(num,0));
      //        getFilteredForce += " "+to_string(contacted_force_data(num,0));
      //        getContactedForceTorque += " "+to_string(contacted_force_data(num,0));
      //        getActualQ +=" "+to_string(joint_positions[num]);
      //      }
      //      for(int num = 0; num<3; num++)
      //      {
      //        getActualToolAccelerometer += " "+to_string(tool_acc_data(num,0));
      //        getTargetTCPPose += " "+to_string(0);
      //        getActualTCPPose += " "+to_string(tcp_pose(num,0));
      //        getActualToolSpeed +=" "+to_string(0);
      //        getActualToolAcc +=" "+to_string(0);
      //
      //      }
      //      for(int num = 0; num<3; num++)
      //      {
      //        getActualTCPPose += " "+to_string(0);
      //        getTargetTCPPose += " "+to_string(0);
      //      }
      //
      //      data_line = to_string(time_count)+getTargetTCPPose+getActualTCPPose+getActualTCPForce+getFilteredForce+getContactedForceTorque+
      //          getActualToolAccelerometer+getActualToolSpeed+getActualToolAcc+getActualQ;
      //
      //      out<<data_line<<endl;
      //
      //      getTargetTCPPose = "";
      //      getActualTCPPose = "";
      //      getActualTCPForce = "";
      //      getFilteredForce = "";
      //      getContactedForceTorque = "";
      //      getActualToolAccelerometer = "";
      //      getActualToolSpeed = "";
      //      getActualToolAcc = "";
      //      getActualQ = "";
      //
      //      joint_cur_value_pub.publish(joint_cur_value_msg);
      //      ee_cur_value_pub.publish(ee_cur_value_msg);
      //
      //      joint_cur_value_msg.data.clear();
      //      ee_cur_value_msg.data.clear();

      ros::spinOnce();

      rt_task_wait_period(NULL);
    }
  }
  else
  {
    std::cout << COLOR_GREEN_BOLD << "Gazebo robot start " << COLOR_RESET << std::endl;
    //Start the task loop
    while(1){
      if((rt_timer_read() - tstart)/1000000.0  - previous_t > (control_time*1000 + 0.15)) //
      {
        printf("delayed Loop time: %.5f ms\n",((rt_timer_read() - tstart)/1000000.0  - previous_t) - control_time*1000);
      }
      previous_t = (rt_timer_read() - tstart)/1000000.0;

      // controller algorithm

      if(task_command.compare(""))
      {
        ur10e_task->run_task_motion();
      }

      // have to add force controller

      //
      ur10e_task->generate_trajectory();
      desired_pose_vector = ur10e_task->get_current_pose();
      //
      double x,y,z;
      x = ur10e_kinematics->get_rotation_matrix_to_axis(ur10e_kinematics->desired_rotation_matrix_xd(desired_pose_vector[3],desired_pose_vector[4],desired_pose_vector[5]))(1,0);
      y = ur10e_kinematics->get_rotation_matrix_to_axis(ur10e_kinematics->desired_rotation_matrix_xd(desired_pose_vector[3],desired_pose_vector[4],desired_pose_vector[5]))(2,0);
      z = ur10e_kinematics->get_rotation_matrix_to_axis(ur10e_kinematics->desired_rotation_matrix_xd(desired_pose_vector[3],desired_pose_vector[4],desired_pose_vector[5]))(3,0);


      tf_desired = Transform3D<> (Vector3D<>(desired_pose_vector[0], desired_pose_vector[1], desired_pose_vector[2]), EAA<>(x, y, z).toRotation3D());
      solutions = solver.solve(tf_desired, state);

      ur10e_kinematics->calculate_forward_kinematics(solutions[3].toStdVector());

      for(int num= 0; num  < 6; num ++)
        joint_cur_value_msg.data.push_back(solutions[3].toStdVector()[num]);

      for(int num= 0; num  < 3; num ++)
        ee_cur_value_msg.data.push_back(ur10e_kinematics->get_tf_base_to_tool()(num,3));

      for(int num= 0; num  < 3; num ++)
        ee_cur_value_msg.data.push_back(ur10e_kinematics->get_axis_to_euler_angle(x,y,z)(num,0)*RADIAN2DEGREE);

      gazebo_shoulder_pan_position_msg.data = solutions[3].toStdVector()[0];
      gazebo_shoulder_lift_position_msg.data = solutions[3].toStdVector()[1];
      gazebo_elbow_position_msg.data = solutions[3].toStdVector()[2];
      gazebo_wrist_1_position_msg.data = solutions[3].toStdVector()[3];
      gazebo_wrist_2_position_msg.data = solutions[3].toStdVector()[4];
      gazebo_wrist_3_position_msg.data = solutions[3].toStdVector()[5];

      gazebo_shoulder_pan_position_pub.publish(gazebo_shoulder_pan_position_msg);
      gazebo_shoulder_lift_position_pub.publish(gazebo_shoulder_lift_position_msg);
      gazebo_elbow_position_pub.publish(gazebo_elbow_position_msg);
      gazebo_wrist_1_position_pub.publish(gazebo_wrist_1_position_msg);
      gazebo_wrist_2_position_pub.publish(gazebo_wrist_2_position_msg);
      gazebo_wrist_3_position_pub.publish(gazebo_wrist_3_position_msg);

      joint_cur_value_pub.publish(joint_cur_value_msg);
      ee_cur_value_pub.publish(ee_cur_value_msg);

      joint_cur_value_msg.data.clear();
      ee_cur_value_msg.data.clear();

      ros::spinOnce();
      rt_task_wait_period(NULL);
    }
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
//void ZeroCommandMsgCallBack(const std_msgs::Bool::ConstPtr& msg)
//{
//  zero_command = true;
//}
void CommandDataMsgCallBack (const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  for(int num = 0; num < 6 ; num++)
  {
    desired_pose_matrix(num,1) = msg->data[num];
  }
  if(msg->data[6] <= 0)
    return;

  for(int num = 0; num < 6 ; num++)
  {
    motion_time =  msg->data[6];
  }

  task_command = "";
  ur10e_task->set_point(msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5], msg->data[6]);
}
void TaskCommandDataMsgCallBack (const std_msgs::String::ConstPtr& msg)
{
  static std::string path_ = "";
  task_command = msg->data;

  if(!task_command.compare("init"))
  {
    ur10e_task->clear_task_motion();
    return;
  }

  path_ = "/home/yik/sdu_ws/SDU-UR10E-Estimation/config/" + task_command + ".yaml";

  ur10e_task->load_task_motion(path_);
}
void initialize()
{
  //ft_filter = std::make_shared<FTfilter>();
  tool_estimation = std::make_shared<ToolEstimation>();

  ur10e_kinematics = std::make_shared<Kinematics>();
  ur10e_traj = std::make_shared<CalRad>();
  ur10e_task = std::make_shared<TaskMotion>();

  //kinematics
  ur10e_kinematics->set_dh_parameter(0.1807, -0.6127, -0.57155, 0.17415, 0.11985, 0.11655);

  // fifth order traj
  control_time = 0.002;
  ur10e_traj->set_control_time(control_time);

  ur10e_task->initialize(control_time);
  ur10e_task->set_initial_pose(-0.5, 0.184324, 0.5875, -180*DEGREE2RADIAN, 0, 0); // set to be robot initial values
  motion_time = 5;

  raw_force_torque_data.resize(6,1);
  raw_force_torque_data.fill(0);

  tool_acc_data.resize(4,1);
  tool_acc_data.fill(0);
  tool_acc_data(3,0)=1;

  tcp_pose.resize(6,1);
  tcp_pose.fill(0);

  tcp_target_pose.resize(6,1);
  tcp_target_pose.fill(0);


  desired_pose_matrix.resize(6,8);
  desired_pose_matrix.fill(0);

  tool_estimation ->set_parameters(control_time, 2.52);
  gravity.resize(4,1);
  gravity << 0.114921681582928,
      -0.076614454388618,
      9.2703,
      0;

  desired_pose_matrix(0,1) = -0.5;
  desired_pose_matrix(1,1) = 0.184324;
  desired_pose_matrix(2,1) = 0.5875;
  desired_pose_matrix(3,1) = -180*DEGREE2RADIAN;
  desired_pose_matrix(4,1) = 0;
  desired_pose_matrix(5,1) = 0;

  // initial pose load
  joint_vector.resize(6);
  desired_pose_vector.resize(6);

  // real robot
  //robot A
  //  desired_pose_vector[0] = -0.5;
  //  desired_pose_vector[1] = 0.184324;
  //  desired_pose_vector[2] = 0.5875;
  //  desired_pose_vector[3] = -180*DEGREE2RADIAN;
  //  desired_pose_vector[4] = 0;
  //  desired_pose_vector[5] = 0;

  //robot B
  desired_pose_vector[0] = -0.41361;
  desired_pose_vector[1] = 0.03022;
  desired_pose_vector[2] = 0.45822;
  desired_pose_vector[3] = 2.222;
  desired_pose_vector[4] = 2.220;
  desired_pose_vector[5] = 0.001;


  //
  tf_current_matrix.resize(4,4);
  tf_current_matrix.fill(0);

  //gazebo
  gazebo_shoulder_pan_position_msg.data = 3.1213190230795913;
  gazebo_shoulder_lift_position_msg.data = -1.2650450021126538;
  gazebo_elbow_position_msg.data = -1.9836958306426968;
  gazebo_wrist_1_position_msg.data = -1.4640438428275298;
  gazebo_wrist_2_position_msg.data = 1.5707963268419363;
  gazebo_wrist_3_position_msg.data = 1.5505226962847685;

  task_command = "";
  //data variables define
  getTargetTCPPose = " target_tcp_pose_x target_tcp_pose_y target_tcp_pose_z target_tcp_pose_r target_tcp_pose_p target_tcp_pose_y";
  getActualTCPPose = " actual_tcp_pose_x actual_tcp_pose_y actual_tcp_pose_z actual_tcp_pose_r actual_tcp_pose_p actual_tcp_pose_y";
  getActualTCPForce = " actual_tcp_force_x actual_tcp_force_y actual_tcp_force_z actual_tcp_force_r actual_tcp_force_p actual_tcp_force_y";
  getFilteredForce = " filtered_force_x filtered_force_y filtered_force_z filtered_torque_r filtered_torque_p filtered_torque_y";
  getContactedForceTorque = " contacted_force_x contacted_force_y contacted_force_z contacted_torque_r contacted_torque_p contacted_torque_y";
  getActualToolAccelerometer = " actual_tcp_acc_x actual_tcp_acc_y actual_tcp_acc_z";
  getActualToolSpeed = " actual_tcp_speed_r actual_tcp_speed_p actual_tcp_speed_y";
  getActualToolAcc = " actual_tcp_acc_r actual_tcp_acc_p actual_tcp_acc_y";
  getActualQ = " actual_q_x actual_q_y actual_q_z";

  data_line = "time"+getTargetTCPPose+getActualTCPPose+getActualTCPForce+getFilteredForce+getContactedForceTorque+
      getActualToolAccelerometer+getActualToolSpeed+getActualToolAcc+getActualQ;

  out<<data_line<<endl;

  getTargetTCPPose = "";
  getActualTCPPose = "";
  getActualTCPForce = "";
  getFilteredForce = "";
  getContactedForceTorque = "";
  getActualToolAccelerometer = "";
  getActualToolSpeed = "";
  getActualToolAcc = "";
  getActualQ = "";
}

int main (int argc, char **argv)
{
  gazebo_check = true;
  std::string silmulation_on_off;
  std::cout << COLOR_YELLOW_BOLD << "Simulation On [ yes / no ]" << COLOR_RESET << std::endl;
  cin >> silmulation_on_off;

  if(!silmulation_on_off.compare("yes") || !silmulation_on_off.compare("y"))
    std::cout << COLOR_GREEN_BOLD << "Setting up Simulation " << COLOR_RESET << std::endl;
  else
  {
    std::cout << COLOR_GREEN_BOLD << "REAL Robot, Be careful to run:" << COLOR_RESET << std::endl;
    std::cout << COLOR_GREEN_BOLD << "Are you sure ? [yes / no]" << COLOR_RESET << std::endl;
    cin >> silmulation_on_off;
    if(!silmulation_on_off.compare("no") || !silmulation_on_off.compare("n"))
    {
      std::cout << COLOR_RED_BOLD << "Terminate program" << COLOR_RESET << std::endl;
      return 0;
    }
    gazebo_check = false;
  }

  std::cout << COLOR_GREEN_BOLD << "Program Start:" << COLOR_RESET << std::endl;
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  //ros publisher
  joint_cur_value_pub = n.advertise<std_msgs::Float64MultiArray>("/sdu/ur10e/joint_cur_value", 10);
  ee_cur_value_pub = n.advertise<std_msgs::Float64MultiArray>("/sdu/ur10e/ee_cur_value", 10);
  raw_force_torque_data_pub = n.advertise<std_msgs::Float64MultiArray>("/sdu/ur10e/raw_force_torque_data", 10);
  filtered_force_torque_data_pub = n.advertise<std_msgs::Float64MultiArray>("/sdu/ur10e/filtered_force_torque_data", 10);

  // ros subsrcibe
  ros::Subscriber zero_command_sub;
  ros::Subscriber command_sub;
  ros::Subscriber task_command_sub;

  command_sub = n.subscribe("/sdu/ur10e/ee_command", 10, CommandDataMsgCallBack);
  task_command_sub = n.subscribe("/sdu/ur10e/task_command", 10, TaskCommandDataMsgCallBack);

  initialize();
  usleep(3000000);

  if(gazebo_check)
  {
    gazebo_shoulder_pan_position_pub = n.advertise<std_msgs::Float64>("/ur10e_robot/shoulder_pan_position/command", 10);
    gazebo_shoulder_lift_position_pub = n.advertise<std_msgs::Float64>("/ur10e_robot/shoulder_lift_position/command", 10);
    gazebo_elbow_position_pub = n.advertise<std_msgs::Float64>("/ur10e_robot/elbow_position/command", 10);
    gazebo_wrist_1_position_pub = n.advertise<std_msgs::Float64>("/ur10e_robot/wrist_1_position/command", 10);
    gazebo_wrist_2_position_pub = n.advertise<std_msgs::Float64>("/ur10e_robot/wrist_2_position/command", 10);
    gazebo_wrist_3_position_pub = n.advertise<std_msgs::Float64>("/ur10e_robot/wrist_3_position/command", 10);

    gazebo_shoulder_pan_position_pub.publish(gazebo_shoulder_pan_position_msg);
    gazebo_shoulder_lift_position_pub.publish(gazebo_shoulder_lift_position_msg);
    gazebo_elbow_position_pub.publish(gazebo_elbow_position_msg);
    gazebo_wrist_1_position_pub.publish(gazebo_wrist_1_position_msg);
    gazebo_wrist_2_position_pub.publish(gazebo_wrist_2_position_msg);
    gazebo_wrist_3_position_pub.publish(gazebo_wrist_3_position_msg);
  }
  else
  {
    const std::string robot_ip = "192.168.1.130"; // robot B

    rtde_receive = std::make_shared<RTDEReceiveInterface>(robot_ip);
    rtde_control = std::make_shared<RTDEControlInterface>(robot_ip);


    //rtde_control->zeroFtSensor();

    std::cout << COLOR_YELLOW_BOLD << "Robot connected to your program" << COLOR_RESET << std::endl;
    std::cout << COLOR_RED_BOLD << "Robot will move 2 seconds later" << COLOR_RESET << std::endl;
    usleep(2000000);
    rtde_control->moveL(desired_pose_vector,0.1,0.1);
    std::cout << COLOR_RED_BOLD << "Send" << COLOR_RESET << std::endl;
    usleep(2000000);
  }

  //system conffiguration
  std::string ft_init_data_path;
  ft_init_data_path = "../config/ft_init_data.yaml";


  //ft_filter->initialize(ft_init_data_path);
  //tool_estimation->initialize();
  //tool_estimation->set_parameters(control_time, 1);
  zero_command = false;
  //

  ros::spinOnce();

  usleep(3000000);

  std::cout << COLOR_GREEN << "All of things were initialized!" << COLOR_RESET << std::endl;

  //real time task
  char str[35];
  mlockall(MCL_CURRENT | MCL_FUTURE); //Lock the memory to avoid memory swapping for this program

  printf("Starting cyclic task...\n");
  sprintf(str, "real time control loop task start");
  rt_task_create(&loop_task, str, 0, 80, 0);//Create the real time task
  rt_task_start(&loop_task, &loop_task_proc, 0);//Since task starts in suspended mode, start task

  std::cout << COLOR_GREEN << "Real time task loop was created!" << COLOR_RESET << std::endl;

  pause();
  rt_task_delete(&loop_task);

  usleep(3000000);
  rtde_control->stopRobot();
  out.close();
  cout << "complete and save" << "\n\n";
  printf("exiting safely\n");
  return 0;
}
