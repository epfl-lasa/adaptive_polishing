#include "PickAndPlacePlanner.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pickAndPlace_node");

  ros::NodeHandle nh;
  double frequency = 250.0;


  std::string input_att1_desired_vel;
  std::string input_att1_target;
  std::string input_att2_desired_vel;
  std::string input_att2_target;
  std::string input_att3_desired_vel;
  std::string input_att3_target;
  std::string input_real_pos;
  std::string output_desired_vel;
  std::string output_active_node;
  int activeNode = 1;
  std::vector<Eigen::Vector3d> targets;
  std::vector<double> target1;
  std::vector<double> target2;
  std::vector<double> target3;


  if (!nh.getParam("input_att1_desired_vel_topic_name", input_att1_desired_vel))   {
    ROS_ERROR("Couldn't retrieve the topic name for the input. ");
    // return -1;
  }

  if (!nh.getParam("input_att2_desired_vel_topic_name", input_att2_desired_vel))   {
    ROS_ERROR("Couldn't retrieve the topic name for the input. ");
    // return -1;
  }

  if (!nh.getParam("input_att3_desired_vel_topic_name", input_att3_desired_vel))   {
    ROS_ERROR("Couldn't retrieve the topic name for the input. ");
    // return -1;
  }

  if (!nh.getParam("input_att1_target_topic_name", input_att1_target))   {
    ROS_ERROR("Couldn't retrieve the topic name for the input. ");
    // return -1;
  }

  if (!nh.getParam("input_att2_target_topic_name", input_att2_target))   {
    ROS_ERROR("Couldn't retrieve the topic name for the input. ");
    // return -1;
  }

  if (!nh.getParam("input_att3_target_topic_name", input_att3_target))   {
    ROS_ERROR("Couldn't retrieve the topic name for the input. ");
    // return -1;
  }

  if (!nh.getParam("input_real_pos_topic_name", input_real_pos))   {
    ROS_ERROR("Couldn't retrieve the topic name for the input. ");
    // return -1;
  }

  if (!nh.getParam("output_desired_vel_topic_name", output_desired_vel))   {
    ROS_ERROR("Couldn't retrieve the topic name for the output. ");
    // return -1;
  }

  if (!nh.getParam("output_active_node_topic_name", output_active_node))   {
    ROS_ERROR("Couldn't retrieve the topic name for the output. ");
    // return -1;
  }


  if (!nh.getParam("target1", target1))   {
    ROS_ERROR("Couldn't retrieve the target1. ");
    // return -1;
  }
  Eigen::Vector3d tmp(target1.data());
  targets.push_back(tmp);


  if (!nh.getParam("target2", target2))   {
    ROS_ERROR("Couldn't retrieve the target2. ");
    // return -1;
  }

  Eigen::Vector3d tmp2(target2.data());
  targets.push_back(tmp2);


  if (!nh.getParam("target3", target3))   {
    ROS_ERROR("Couldn't retrieve the target3. ");
    // return -1;
  }
  Eigen::Vector3d tmp3(target3.data());
  targets.push_back(tmp3); 


  PickAndPlacePlanner planner(
    nh,
    frequency,
    input_att1_desired_vel,
    input_att1_target,
    input_att2_desired_vel,
    input_att2_target,
    input_att3_desired_vel,
    input_att3_target,
    input_real_pos,
    output_desired_vel,
    output_active_node,
    activeNode,
    targets);

  planner.Run();

  return 0;
}