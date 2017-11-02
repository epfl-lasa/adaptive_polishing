#ifndef __MOTION_GENERATION_H__
#define __MOTION_GENERATION_H__

#include "ros/ros.h"
#include <vector>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Accel.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Path.h"



#include "MathLib.h"
#include "eigen3/Eigen/Dense"
#include "CDDynamics.h"

#include <mutex>
#include <pthread.h>
#include <signal.h>

// #include <dynamic_reconfigure/server.h>
// #include <adaptive_polishing/polishing_paramsConfig.h>

class MotionGenerator {


protected:

	// ROS variables
	ros::NodeHandle nh_;
	ros::Rate loop_rate_;

	ros::Subscriber sub_real_pose_;
	ros::Subscriber sub_real_vel_;
	ros::Subscriber sub_real_acc_;
	ros::Subscriber sub_robot_force_;
	ros::Publisher pub_human_action_;
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


	// Class variables
	std::mutex mutex_;

	// MathLib::Vector real_pose_;
	// MathLib::Vector real_vel_;
	// MathLib::Vector real_acc_;
	// MathLib::Vector target_pose_;
	// MathLib::Vector target_offset_;
	Eigen::Vector3d real_pose_;
	Eigen::Vector3d real_vel_;
	Eigen::Vector3d real_acc_;
	Eigen::Vector3d target_offset_;
	Eigen::Vector3d rob_sensed_force;
	int FORCE_THRESHOLD = 10;
	//create a pointer that can point to where a trajectory indicates
	//std::unique_ptr<Eigen::Vector3d> target_pose_;

	double Velocity_limit_;

	// MathLib::Vector desired_velocity_;
	// MathLib::Vector desired_velocity_filtered_;
	Eigen::Vector3d desired_velocity_;
	Eigen::Vector3d desired_velocity_filtered_;


	//boolean to make sure we have received at least one position
	bool gotFirstPosition_ = false;

	//thread to publish futur path
	pthread_t thread_;
	bool startThread_;
	//futur path variables
	nav_msgs::Path msg_DesiredPath_;
	int MAX_FRAME = 200;

	//so we can stop
	static MotionGenerator* me;
	bool stop_ = false;

public:
	MotionGenerator(ros::NodeHandle &n,
		double frequency,
		std::string input_rob_pos_topic_name,
		std::string input_rob_vel_topic_name,
		std::string input_rob_acc_topic_name,
		std::string input_rob_force_topic_name,
		std::string output_vel_topic_name,
		std::string output_filtered_vel_topic_name
        );

	bool Init();

	void Run();

private:

	void UpdateRealPosition(const geometry_msgs::Pose::ConstPtr& msg);

	void UpdateRealVelocity(const geometry_msgs::Twist::ConstPtr& msg);

	void UpdateRealAcceleration(const geometry_msgs::Accel::ConstPtr& msg);

	void UpdateRobotSensedForce(const geometry_msgs::WrenchStamped::ConstPtr& msg);

	//void UpdateRobotForce(const geometry_msgs::Pose::ConstPtr& msg);

	void ComputeDesiredVelocity();

	void PublishDesiredVelocity();

	//publish predicted path and 2 functions to set in on another thread
	void PublishFuturePath();

	static void* startPathPublishingLoop(void* ptr);

	void pathPublishingLoop();

	static void stopNode(int sig);

protected:

	virtual void AdaptTrajectoryParameters(Eigen::Vector3d pose){}

	virtual Eigen::Vector3d GetVelocityFromPose(Eigen::Vector3d pose) = 0;

};


#endif //__MOTION_GENERATION_H__