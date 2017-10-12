#ifndef __MOTION_GENERATION_H__
#define __MOTION_GENERATION_H__

#include "ros/ros.h"
#include <vector>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Path.h"



#include "MathLib.h"
#include "CDDynamics.h"

#include <mutex>


// #include <dynamic_reconfigure/server.h>
// #include <adaptive_polishing/polishing_paramsConfig.h>

class MotionGenerator {


protected:

	// ROS variables
	ros::NodeHandle nh_;
	ros::Rate loop_rate_;

	std::string input_rob_pose_topic_name_;
	std::string output_vel_topic_name_;
	std::string output_filtered_vel_topic_name_;
	std::string input_rob_vel_topic_name_;
	std::string input_rob_force_ee_topic_name_;


	ros::Subscriber sub_real_pose_;
	ros::Publisher pub_desired_twist_;
	ros::Publisher pub_desired_twist_filtered_;


	double dt_;


	// Filter variables: it takes x and use Wn as gains and dx ddx as limits
	std::unique_ptr<CDDynamics> filter_;
	double filter_Wn_;
	MathLib::Vector filter_dxLim_;
	MathLib::Vector filter_ddxLim_;



	ros::Publisher pub_target_;
	ros::Publisher pub_DesiredPath_;



	// geometry_msgs::Pose msg_real_pose_;
	geometry_msgs::TwistStamped msg_desired_velocity_;
	geometry_msgs::TwistStamped msg_desired_velocity_filtered_;

	nav_msgs::Path msg_DesiredPath_;
	int MAX_FRAME = 200;



	// //dynamic reconfig settig
	// dynamic_reconfigure::Server<motionGenerator::motion_paramsConfig> dyn_rec_srv_;
	// dynamic_reconfigure::Server<motionGenerator::motion_paramsConfig>::CallbackType dyn_rec_f_;


	// Class variables
	std::mutex mutex_;

	MathLib::Vector real_pose_;
	MathLib::Vector target_pose_;
	MathLib::Vector target_offset_;

	double Velocity_limit_;

	MathLib::Vector desired_velocity_;
	MathLib::Vector desired_velocity_filtered_;



public:
	MotionGenerator(ros::NodeHandle &n,
		double frequency,
		std::string input_rob_pos_topic_name,
		std::string output_vel_topic_name,
		std::string output_filtered_vel_topic_name,
		std::string input_rob_vel_topic_name,
		std::string input_rob_force_ee_topic_name
        );

	bool Init();

	void Run();

protected:

	bool InitializeROS();

	void UpdateRealPosition(const geometry_msgs::Pose::ConstPtr& msg);

	void ComputeDesiredVelocity();

	void PublishDesiredVelocity();

	void PublishFuturePath();

	virtual MathLib::Vector GetVelocityFromPose(MathLib::Vector pose) = 0;

};


#endif //__MOTION_GENERATION_H__