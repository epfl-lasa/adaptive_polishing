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
		std::string input_rob_force_topic_name,
		std::string output_desired_vel,
		std::string output_active_node,
		int activeNode,
		std::vector<Eigen::Vector3d> targets
)
	:nh_(n),loop_rate_(frequency),activeNode_(activeNode),targets_(targets),
	activeNodeIndex_(activeNode-1),real_pose_(targets[activeNode-1]),prev_vel_(Eigen::Vector3d::Zero())
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

	sub_robot_force_ = nh_.subscribe(input_rob_force_topic_name, 1000,
			&PickAndPlacePlanner::UpdateRobotSensedForce, this,
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


	// intialize the gripper
	gripper_.reset(new RSGripperInterface(false));

  	ROS_INFO("[P&P-node] activating, wait 2 sec ...");
  	gripper_->activate();
    ros::Duration(3.0).sleep();
    gripper_->setSpeed(250);
  	gripper_->setPosition(128);
  	ros::Duration(1.0).sleep();
  	prev_activeNode_ = ((activeNode_ - 1 ) % 3) +1;

}

void PickAndPlacePlanner::Run(){

	while (!stop_) {

		checkTargetReached();

		publishActiveNode();

		ros::spinOnce();



		ROS_INFO_STREAM_THROTTLE(1,"-----------------------------------------------");
		ROS_INFO_STREAM_THROTTLE(1,"sensed force norm: " << rob_sensed_force_.norm());
		ROS_INFO_STREAM_THROTTLE(1,"desired velocity: " << msg_desired_velocity_   );
		ROS_INFO_STREAM_THROTTLE(1,"Active node: " << activeNode_ << " Previous: " << prev_activeNode_);


		loop_rate_.sleep();
	}
	ros::spinOnce();
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
		Eigen::Vector3d tmp;
		tmp << msg->linear.x,msg->linear.y,msg->linear.z;
		prev_vel_ = vel_filter_factor_*prev_vel_+(1-vel_filter_factor_)*tmp;

		msg_desired_velocity_.linear.x  = prev_vel_(X);
		msg_desired_velocity_.linear.y  = prev_vel_(Y);
		msg_desired_velocity_.linear.z  = prev_vel_(Z);

		msg_desired_velocity_.angular.x = msg->angular.x;
		msg_desired_velocity_.angular.y = msg->angular.y;
		msg_desired_velocity_.angular.z = msg->angular.z;

		pub_desired_vel_.publish(msg_desired_velocity_);
	}

}

void PickAndPlacePlanner::updateDesiredVelocity2(const geometry_msgs::Twist::ConstPtr& msg){
	if (activeNode_ == 2){

		Eigen::Vector3d tmp;
		tmp << msg->linear.x,msg->linear.y,msg->linear.z;
		prev_vel_ = vel_filter_factor_*prev_vel_+(1-vel_filter_factor_)*tmp;
		msg_desired_velocity_.linear.x  = prev_vel_(X);
		msg_desired_velocity_.linear.y  = prev_vel_(Y);
		msg_desired_velocity_.linear.z  = prev_vel_(Z);

		msg_desired_velocity_.angular.x = msg->angular.x;
		msg_desired_velocity_.angular.y = msg->angular.y;
		msg_desired_velocity_.angular.z = msg->angular.z;

		pub_desired_vel_.publish(msg_desired_velocity_);
	}
}

void PickAndPlacePlanner::updateDesiredVelocity3(const geometry_msgs::Twist::ConstPtr& msg){
	if (activeNode_ == 3){
		Eigen::Vector3d tmp;
		tmp << msg->linear.x,msg->linear.y,msg->linear.z;
		prev_vel_ = vel_filter_factor_*prev_vel_+(1-vel_filter_factor_)*tmp;
		msg_desired_velocity_.linear.x  = prev_vel_(X);
		msg_desired_velocity_.linear.y  = prev_vel_(Y);
		msg_desired_velocity_.linear.z  = prev_vel_(Z);

		msg_desired_velocity_.angular.x = msg->angular.x;
		msg_desired_velocity_.angular.y = msg->angular.y;
		msg_desired_velocity_.angular.z = msg->angular.z;

		pub_desired_vel_.publish(msg_desired_velocity_);
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

void PickAndPlacePlanner::UpdateRobotSensedForce(
		const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
	rob_sensed_force_(X) = msg->wrench.force.x;
	rob_sensed_force_(Y) = msg->wrench.force.y;
	rob_sensed_force_(Z) = msg->wrench.force.z;
}

void PickAndPlacePlanner::checkTargetReached(){
	// ROS_INFO_STREAM("real_pose_: " << real_pose_ << 
	// 	" target: " << targets_[activeNodeIndex_] << " activeNode: " << activeNode_);
	Eigen::Vector3d error = real_pose_ - targets_[activeNodeIndex_];
	// ROS_INFO_STREAM("error is: " << error.norm());
	ROS_INFO_STREAM_THROTTLE(1,"error norm: " << error.norm());
	if(error.norm() <  ( (activeNode_ == 2) ? 0.08: 0.04 )  ){	

		// if the human lets us go, we change the attractor to the next one
		if(rob_sensed_force_.norm()<7) {

			// only peforming grasping for attractor 1 and 3
		if(activeNode_ != 2) {
			ros::Duration(0.2).sleep();

			// if we reach our target, we open/close the grasp
			if(activeNode_ == 1){
  				gripper_->setPosition(250);
			}
			else if(activeNode_ == 3){
				gripper_->setPosition(0);
			}

			ros::Duration(0.8).sleep();
		}

			int tmp = activeNode_;
			activeNode_ = (activeNode_ == 3 || activeNode_ == 1) ? 2 : (prev_activeNode_ + 2)%4;
			activeNodeIndex_ = activeNode_ - 1;
			prev_activeNode_ = tmp;
		}
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