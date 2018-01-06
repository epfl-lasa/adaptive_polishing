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


class MotionGenerator {


	// ROS variables
protected:
	ros::NodeHandle nh_;
private:
	ros::Rate loop_rate_;
	// subscribers and publishers
	ros::Subscriber sub_real_pose_;
	ros::Subscriber sub_real_vel_;
	ros::Subscriber sub_real_acc_;
	ros::Subscriber sub_robot_force_;
	ros::Publisher pub_external_action_;
	ros::Publisher pub_desired_twist_;
	ros::Publisher pub_desired_twist_filtered_;
	ros::Publisher pub_target_;
	ros::Publisher pub_DesiredPath_;
	// geometry_msgs::Pose msg_real_pose_;
	geometry_msgs::TwistStamped msg_desired_velocity_;
	geometry_msgs::TwistStamped msg_desired_velocity_filtered_;


	// Class variables
protected:
	// time step of node
	double dt_;

	// Filter variables:
	std::unique_ptr<CDDynamics> filter_;
	double filter_Wn_;
	MathLib::Vector filter_dxLim_; //limits
	MathLib::Vector filter_ddxLim_;

	// Vectors to store the robot state
	Eigen::Vector3d real_pose_;
	Eigen::Vector3d real_orientation_;
	Eigen::Vector3d real_vel_;
	Eigen::Vector3d real_acc_;
	Eigen::Vector3d target_offset_;
	Eigen::Vector3d rob_sensed_force_;

	// vectors to store the computed desired velocity
	Eigen::Vector3d desired_velocity_;
	Eigen::Vector3d desired_velocity_filtered_;
	double Velocity_limit_; //speed limit


	// check if we have received the first position
	// also needed to start publishing the futur path
	bool gotFirstPosition_ = false;

	// booean to know if node is active
	bool paused_ = false;

private:
	std::mutex mutex_;
	//thread to publish futur path
	pthread_t thread_futurePath_;
	bool startThread_futurePath_;
	//futur path variables
	nav_msgs::Path msg_DesiredPath_;
	int MAX_FRAME = 200;

	// boolean for the adaptation
	bool ADAPTABLE;
	pthread_t thread_adaptation_;
	bool startThread_adaptation_;

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
		std::string output_filtered_vel_topic_name,
		bool adaptable = false
		);

	bool Init();

	void Run();

private:

	void UpdateRealPosition(const geometry_msgs::Pose::ConstPtr& msg);

	void UpdateRealVelocity(const geometry_msgs::Twist::ConstPtr& msg);

	void UpdateRealAcceleration(const geometry_msgs::Accel::ConstPtr& msg);

	void UpdateRobotSensedForce(const geometry_msgs::WrenchStamped::ConstPtr& msg);


	void ComputeDesiredVelocity();

	void PublishDesiredVelocity();



	//adaptation is set on another thread
	static void* startAdaptationLoop(void* ptr);

	void adaptationLoop();


	/*Set of 3 functions to compute the forward integral of the desired velocity
	* (the estimated path).The computation is done on another thread to prevent 
	* the node from slowing down
	*/
	void PublishFuturePath();

	static void* startPathPublishingLoop(void* ptr);

	void pathPublishingLoop();


	// Function called when the node is killed through CTRL + C
	static void stopNode(int sig);

	void pauseNode();

	void unpauseNode();

protected:

	virtual void AdaptTrajectoryParameters(Eigen::Vector3d pose){}

	virtual Eigen::Vector3d GetVelocityFromPose(Eigen::Vector3d pose) = 0;

};


#endif //__MOTION_GENERATION_H__