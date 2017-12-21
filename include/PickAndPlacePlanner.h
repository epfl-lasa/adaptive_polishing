#ifndef __PLANNER_H__
#define __PLANNER_H__


#include "ros/ros.h"
#include <vector>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Accel.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Int32.h"

#include "MathLib.h"
#include "eigen3/Eigen/Dense"
#include "CDDynamics.h"

#include <mutex>
#include <pthread.h>
#include <signal.h>


#define NUM_ATTRACTORS 3

class PickAndPlacePlanner {
private:
	ros::NodeHandle nh_;
	ros::Rate loop_rate_;
	// 2 subscribers per attractor , one for velocity and one for target
	ros::Subscriber sub_att1_desired_vel_;
	ros::Subscriber sub_att1_target_;

	ros::Subscriber sub_att2_desired_vel_;
	ros::Subscriber sub_att2_target_;

	ros::Subscriber sub_att3_desired_vel_;
	ros::Subscriber sub_att3_target_;

	// real position of the robot
	ros::Subscriber sub_real_pos_;

	//publisher for desired velocity and real message
	ros::Publisher pub_desired_vel_;
	geometry_msgs::Twist msg_desired_velocity_;

	// publisher for active node with associated msg
	ros::Publisher pub_active_node_;
	std_msgs::Int32 msg_active_node;

	std::vector<Eigen::Vector3d> targets_;
	int activeNode_;
	int activeNodeIndex_;
	Eigen::Vector3d real_pose_;

	static PickAndPlacePlanner* me;
	bool stop_ = false;

public:

	PickAndPlacePlanner(
			ros::NodeHandle &n,
			double frequency,
			std::string input_att1_desired_vel,
			std::string input_att1_target,
			std::string input_att2_desired_vel,
			std::string input_att2_target,
			std::string input_att3_desired_vel,
			std::string input_att3_target,
			std::string input_real_pos,
			std::string output_desired_vel,
			std::string output_active_node,
			int activeNode,
			std::vector<Eigen::Vector3d> targets
	);

	void Run();

private:

	void updateDesiredVelocity1(const geometry_msgs::Twist::ConstPtr& msg);

	void updateDesiredVelocity2(const geometry_msgs::Twist::ConstPtr& msg);

	void updateDesiredVelocity3(const geometry_msgs::Twist::ConstPtr& msg);

	void updateTarget1(const geometry_msgs::Pose::ConstPtr& msg);

	void updateTarget2(const geometry_msgs::Pose::ConstPtr& msg);

	void updateTarget3(const geometry_msgs::Pose::ConstPtr& msg);

	void updateRealPos(const geometry_msgs::Pose::ConstPtr& msg);

	void checkTargetReached();

	void publishActiveNode();

	static void stopNode(int sig);

};
#endif // __PLANNER_H__