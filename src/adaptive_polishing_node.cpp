#include "adaptivePolishing.h"

// #include "ros/ros.h"
// #include <vector>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "adaptive_polishing_node");

  ros::NodeHandle nh;
  double frequency = 250.0;


  // Parameters
  //inputs
  std::string input_rob_pose_topic_name;
  std::string input_rob_vel_topic_name;
  std::string input_rob_acc_topic_name;
  std::string input_rob_force_topic_name;
  //outputs
  std::string output_vel_topic_name;
  std::string output_filtered_vel_topic_name;

  //trajectory parameters
  std::vector<double> CenterRotation;
  double radius;
  double major_axis_scale;
  double minor_axis_scale;
  double alpha;
  double RotationSpeed;
  double ConvergenceRate;

  std::vector<double> adaptable_parameters;

  int my_int_param;
  std::vector<double> my_vec_param;


  if (!nh.getParam("input_rob_pose_topic_name", input_rob_pose_topic_name))   {
    ROS_ERROR("Couldn't retrieve the topic name for the input. ");
    // return -1;
  }

  if (!nh.getParam("input_rob_vel_topic_name", input_rob_vel_topic_name))   {
    ROS_ERROR("Couldn't retrieve the topic name for the input. ");
    // return -1;
  }

  if (!nh.getParam("input_rob_acc_topic_name", input_rob_acc_topic_name))   {
    ROS_ERROR("Couldn't retrieve the topic name for the input. ");
    // return -1;
  }

  if (!nh.getParam("input_rob_force_topic_name", input_rob_force_topic_name))   {
    ROS_ERROR("Couldn't retrieve the topic name for the output. ");
    // return -1;
  }

  if (!nh.getParam("output_vel_topic_name", output_vel_topic_name))   {
    ROS_ERROR("Couldn't retrieve the topic name for the output. ");
    // return -1;
  }

  if (!nh.getParam("output_filtered_vel_topic_name", output_filtered_vel_topic_name))   {
    ROS_ERROR("Couldn't retrieve the topic name for the filtered output. ");
    // return -1;
  }

  if (!nh.getParam("CenterRotation", CenterRotation))   {
    ROS_ERROR("Couldn't retrieve the center of rotation. ");
    // return -1;
  }

  if (!nh.getParam("radius", radius))   {
    ROS_ERROR("Couldn't retrieve the radius rotation. ");
    // return -1;
  }

  if (!nh.getParam("major_axis_scale", major_axis_scale))   {
    ROS_ERROR("Couldn't retrieve the major axis of the rotation. ");
    // return -1;
  }

  if (!nh.getParam("minor_axis_scale", minor_axis_scale))   {
    ROS_ERROR("Couldn't retrieve the minor axis of the rotation. ");
    // return -1;
  }

  if (!nh.getParam("alpha", alpha))   {
    ROS_ERROR("Couldn't retrieve the orientation of the rotation. ");
    // return -1;
  }
  alpha = alpha/180*M_PI;

  if (!nh.getParam("RotationSpeed", RotationSpeed))  {
    ROS_ERROR("Couldn't retrieve the rotation speed.");
    // return -1;
  }

  if (!nh.getParam("ConvergenceRate", ConvergenceRate)) {
    ROS_ERROR("Couldn't retrieve the convergence speed. ");
    // return -1;
  }

  if (!nh.getParam("adaptable_parameters", adaptable_parameters))   {
    ROS_ERROR("Couldn't retrieve the orientation of the rotation. ");
    // return -1;
  }

  ROS_INFO("Starting the Motion generator... and guess what? doing nothing!");

  AdaptivePolishing my_adaptive_polishing(
    nh,
    frequency,
    input_rob_pose_topic_name,
    input_rob_vel_topic_name,
    input_rob_acc_topic_name,
    input_rob_force_topic_name,
    output_vel_topic_name,
    output_filtered_vel_topic_name,
    CenterRotation,
    radius,
    major_axis_scale,
    minor_axis_scale,
    alpha,
    adaptable_parameters,
    RotationSpeed,
    ConvergenceRate);

  my_adaptive_polishing.Run();



  return 0;
}