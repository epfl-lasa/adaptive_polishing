#include "PickAndPlacePlanner.h"

enum Coordinate { X=0 , Y=1 , Z=2};

//create pointer to self for CTRL + C catching
PickAndPlacePlanner* PickAndPlacePlanner::me = NULL;

PickAndPlacePlanner::PickAndPlacePlanner(
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
)
	:nh_(n),loop_rate_(frequency),activeNode_(activeNode),targets_(targets),
	activeNodeIndex_(activeNode-1),real_pose_(targets[activeNode-1])
{
	// interruption set up to catch CTRL + C
	me = this;
	signal(SIGINT,PickAndPlacePlanner::stopNode);

	ROS_INFO_STREAM("PickAndPlacePlanner.CPP: planner node is created at: " <<
			nh_.getNamespace() << " with freq: " << frequency << "Hz");

	sub_att1_desired_vel_ = nh_.subscribe(input_att1_desired_vel,1000,
			&PickAndPlacePlanner::updateDesiredVelocity1, this,
			ros::TransportHints().reliable().tcpNoDelay());

	sub_att2_desired_vel_ = nh_.subscribe(input_att2_desired_vel,1000,
			&PickAndPlacePlanner::updateDesiredVelocity2, this,
			ros::TransportHints().reliable().tcpNoDelay());

	sub_att3_desired_vel_ = nh_.subscribe(input_att3_desired_vel,1000,
			&PickAndPlacePlanner::updateDesiredVelocity3, this,
			ros::TransportHints().reliable().tcpNoDelay());

	sub_att1_target_ = nh_.subscribe(input_att1_target,1000,
			&PickAndPlacePlanner::updateTarget1, this,
			ros::TransportHints().reliable().tcpNoDelay());

	sub_att2_target_ = nh_.subscribe(input_att2_target,1000,
			&PickAndPlacePlanner::updateTarget2, this,
			ros::TransportHints().reliable().tcpNoDelay());

	sub_att3_target_ = nh_.subscribe(input_att3_target,1000,
			&PickAndPlacePlanner::updateTarget3, this,
			ros::TransportHints().reliable().tcpNoDelay());

	sub_real_pos_ = nh_.subscribe(input_real_pos,1000,
			&PickAndPlacePlanner::updateRealPos, this,
			ros::TransportHints().reliable().tcpNoDelay());


	pub_active_node_ = nh_.advertise<std_msgs::Int32>(
			output_active_node, 1000, 1);

	pub_desired_vel_ = nh_.advertise<geometry_msgs::Twist>(
			output_desired_vel, 1000, 1);

	pub_pickAndPlace_param_ = nh_.advertise<adaptive_polishing::pickAndPlaceParam_msg>(
			"/DS/pickAndPlace/pickAndPlace_param", 1000, 1);

	double duration = 0.01;
	publishTimer_ = nh_.createTimer(ros::Duration(duration), 
			&PickAndPlacePlanner::PublishTargets,this);
 
}

void PickAndPlacePlanner::Run(){

	while (!stop_) {

		checkTargetReached();

		publishActiveNode();

		ros::spinOnce();

		loop_rate_.sleep();
	}
	// send a 0 velocity command to the robot and stop the node
	msg_desired_velocity_.linear.x  = 0;
	msg_desired_velocity_.linear.y  = 0;
	msg_desired_velocity_.linear.z  = 0;

	msg_desired_velocity_.angular.x = 0;
	msg_desired_velocity_.angular.y = 0;
	msg_desired_velocity_.angular.z = 0;

	pub_desired_vel_.publish(msg_desired_velocity_);

	ros::spinOnce();
	loop_rate_.sleep();

	ros::shutdown();
}

void PickAndPlacePlanner::updateDesiredVelocity1(const geometry_msgs::Twist::ConstPtr& msg){
	if (activeNode_ == 1){
		pub_desired_vel_.publish(msg);
	}

}

void PickAndPlacePlanner::updateDesiredVelocity2(const geometry_msgs::Twist::ConstPtr& msg){
	if (activeNode_ == 2){
		pub_desired_vel_.publish(msg);
	}
}

void PickAndPlacePlanner::updateDesiredVelocity3(const geometry_msgs::Twist::ConstPtr& msg){
	if (activeNode_ == 3){
		pub_desired_vel_.publish(msg);
	}
}

void PickAndPlacePlanner::updateTarget1(const geometry_msgs::Pose::ConstPtr& msg){
	targets_[0](X) = msg->position.x;
	targets_[0](Y) = msg->position.y;
	targets_[0](Z) = msg->position.z;
}

void PickAndPlacePlanner::updateTarget2(const geometry_msgs::Pose::ConstPtr& msg){
	targets_[1](X) = msg->position.x;
	targets_[1](Y) = msg->position.y;
	targets_[1](Z) = msg->position.z;
}

void PickAndPlacePlanner::updateTarget3(const geometry_msgs::Pose::ConstPtr& msg){
	targets_[2](X) = msg->position.x;
	targets_[2](Y) = msg->position.y;
	targets_[2](Z) = msg->position.z;
}

void PickAndPlacePlanner::updateRealPos(const geometry_msgs::Pose::ConstPtr& msg){
	real_pose_(X) = msg->position.x;
	real_pose_(Y) = msg->position.y;
	real_pose_(Z) = msg->position.z;
}

void PickAndPlacePlanner::checkTargetReached(){
	// ROS_INFO_STREAM("real_pose_: " << real_pose_ << 
	// 	" target: " << targets_[activeNodeIndex_] << " activeNode: " << activeNode_);
	Eigen::Vector3d error = real_pose_ - targets_[activeNodeIndex_];
	// ROS_INFO_STREAM("error is: " << error.norm());
	if(error.norm() < 0.05){
		activeNode_ = activeNode_ == 3 ? 1 : activeNode_ + 1;
		activeNodeIndex_ = activeNode_ - 1;
	}
}

void PickAndPlacePlanner::publishActiveNode(){
	msg_active_node.data = activeNode_;
	pub_active_node_.publish(msg_active_node);
}

void PickAndPlacePlanner::stopNode(int sig)
{
	ROS_INFO("Catched the ctrl C");
	me->stop_ = true;
}


void PickAndPlacePlanner::PublishTargets(const ros::TimerEvent&){
	msg_pickAndPlaceParam_.activeNode =activeNode_;
	msg_pickAndPlaceParam_.target1_x =targets_[0](X);
	msg_pickAndPlaceParam_.target1_y =targets_[0](Y);
	msg_pickAndPlaceParam_.target2_x =targets_[1](X);
	msg_pickAndPlaceParam_.target2_y =targets_[1](Y);
	msg_pickAndPlaceParam_.target3_x =targets_[2](X);
	msg_pickAndPlaceParam_.target3_y =targets_[2](Y);
	pub_pickAndPlace_param_.publish(msg_pickAndPlaceParam_);
}