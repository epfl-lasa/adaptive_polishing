#include "MotionGenerator.h"


enum Coordinate { X=0 , Y=1 , Z=2};
enum Orientation {ROLL=0, PITCH=1, YAW=1};

#define SUB_BUFFER_SIZE 1000

//create pointer to self for CTRL + C catching
MotionGenerator* MotionGenerator::me = NULL;




//====================Initialisation and Run functions==========================

MotionGenerator::MotionGenerator(ros::NodeHandle &n,
		double frequency,
		std::string input_rob_pose_topic_name,
		std::string input_rob_vel_topic_name,
		std::string input_rob_acc_topic_name,
		std::string input_rob_force_topic_name,
		std::string output_vel_topic_name,
		std::string output_filtered_vel_topic_name,
		bool adaptable
)
	:nh_(n), loop_rate_(frequency), filter_Wn_(.411),dt_(1 / frequency),
	Velocity_limit_(1), ADAPTABLE(adaptable) 
{

	me = this;

	// interruption set up to catch CTRL + C
	signal(SIGINT,MotionGenerator::stopNode);

	//ROS Topic initialization
	sub_real_pose_ = nh_.subscribe(input_rob_pose_topic_name, SUB_BUFFER_SIZE,
			&MotionGenerator::UpdateRealPosition, this,
			ros::TransportHints().reliable().tcpNoDelay());

	sub_real_vel_ = nh_.subscribe(input_rob_vel_topic_name, SUB_BUFFER_SIZE,
			&MotionGenerator::UpdateRealVelocity, this,
			ros::TransportHints().reliable().tcpNoDelay());

	sub_real_acc_ = nh_.subscribe(input_rob_acc_topic_name, SUB_BUFFER_SIZE,
			&MotionGenerator::UpdateRealAcceleration, this,
			ros::TransportHints().reliable().tcpNoDelay());

	sub_robot_force_ = nh_.subscribe(input_rob_force_topic_name, SUB_BUFFER_SIZE,
			&MotionGenerator::UpdateRobotSensedForce, this,
			ros::TransportHints().reliable().tcpNoDelay());


	pub_desired_twist_ = nh_.advertise<geometry_msgs::Twist>(
			output_vel_topic_name, 1000, 1);

	pub_desired_twist_filtered_ = nh_.advertise<geometry_msgs::Twist>(
			output_filtered_vel_topic_name, 1);

	pub_target_ = nh_.advertise<geometry_msgs::PointStamped>("/DS/target", 1);
	pub_DesiredPath_ = nh_.advertise<nav_msgs::Path>("/DS/DesiredPath", 1);
	pub_external_action_ = nh_.advertise<geometry_msgs::WrenchStamped>(
			"/DS/human_force",1);

	msg_DesiredPath_.poses.resize(MAX_FRAME);

	if (nh_.ok()) { // Wait for poses being published
		ros::spinOnce();
		ROS_INFO("The Motion generator is ready.");
	}


	Init();
	ROS_INFO_STREAM("MotionGenerator.CPP: MotionGenerator node is created at: "
			 << nh_.getNamespace() << " with freq: " << frequency << "Hz");
}// end constructor


bool MotionGenerator::Init() {

	// initializing the filter
	filter_.reset (new CDDynamics(3, dt_, filter_Wn_));

	// we should set the size automagically
	filter_dxLim_.Resize(3);
	filter_dxLim_(X) = 1;
	filter_dxLim_(Y) = 1;
	filter_dxLim_(Z) = 1;

	filter_->SetVelocityLimits(filter_dxLim_);

	filter_ddxLim_.Resize(3);
	filter_ddxLim_(X) = .51;
	filter_ddxLim_(Y) = .51;
	filter_ddxLim_(Z) = .51;
	filter_->SetAccelLimits(filter_ddxLim_);

	MathLib::Vector initial(3);
	initial.Zero();
	filter_->SetState(initial);
	filter_->SetTarget(initial);


	target_offset_.setZero();
	rob_sensed_force_.setZero();


	//thread for future path
	startThread_futurePath_ = true;
	if(pthread_create(&thread_futurePath_, NULL,
			&MotionGenerator::startPathPublishingLoop,this))
	{
	 	throw std::runtime_error("Cannot create reception thread");  
	}

	//thread for adaptation
	if(ADAPTABLE){
		startThread_adaptation_ = true;
		if(pthread_create(&thread_adaptation_, NULL,
				&MotionGenerator::startAdaptationLoop,this))
		{
			throw std::runtime_error("Cannot create adaptation thread");  
		}
	}

	return true;
}//end Init

void MotionGenerator::Run() {

	while (!stop_) {

		if(!paused_)
		{
			ComputeDesiredVelocity();

			PublishDesiredVelocity();
		}

		ros::spinOnce();

		loop_rate_.sleep();
	}

	// send a 0 velocity command to the robot and stop the node
	desired_velocity_.setZero();
	PublishDesiredVelocity();
	ros::spinOnce();
	loop_rate_.sleep();

	startThread_futurePath_ = false;
	pthread_join(thread_futurePath_,NULL);

	if(ADAPTABLE){
		startThread_adaptation_ = false;
		pthread_join(thread_adaptation_,NULL);
	}

	ros::shutdown();

}


//====================END: Initialisation and Run functions=====================

//====================Callback functions to get robot states====================

void MotionGenerator::UpdateRealPosition(
		const geometry_msgs::Pose::ConstPtr& msg) 
{
	if(!gotFirstPosition_)
	{
		gotFirstPosition_ = true;
	}
	real_pose_(X) = msg->position.x;
	real_pose_(Y) = msg->position.y;
	real_pose_(Z) = msg->position.z;

	double qtx = msg->orientation.x;
	double qty = msg->orientation.y;
	double qtz = msg->orientation.z;
	double qtw = msg->orientation.w;

	Eigen::Quaterniond q(qtw, qtx, qty, qtz);
	Eigen::Matrix3d rot_mat = q.toRotationMatrix();
	Eigen::Vector3d  euler_angles = rot_mat.eulerAngles(ROLL,PITCH,YAW);

	real_orientation_(ROLL) = euler_angles(ROLL);
	real_orientation_(PITCH) = euler_angles(PITCH);
	real_orientation_(YAW) = euler_angles(YAW);
}


void MotionGenerator::UpdateRealVelocity(
		const geometry_msgs::Twist::ConstPtr& msg)
{
	real_vel_(X) = msg->linear.x;
	real_vel_(Y) = msg->linear.y;
	real_vel_(Z) = msg->linear.z;

}

void MotionGenerator::UpdateRealAcceleration(
		const geometry_msgs::Accel::ConstPtr& msg)
{
	real_acc_(X) = msg->linear.x;
	real_acc_(Y) = msg->linear.y;
	real_acc_(Z) = msg->linear.z;
}

void MotionGenerator::UpdateRobotSensedForce(
		const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
	rob_sensed_force_(X) = msg->wrench.force.x;
	rob_sensed_force_(Y) = msg->wrench.force.y;
	rob_sensed_force_(Z) = msg->wrench.force.z;

	geometry_msgs::WrenchStamped msg_external_force;
	msg_external_force.header = msg->header;

	msg_external_force.wrench.force.x = -rob_sensed_force_(X);
	msg_external_force.wrench.force.y = -rob_sensed_force_(Y);
	msg_external_force.wrench.force.z = -rob_sensed_force_(Z);

	pub_external_action_.publish(msg_external_force);
}

//====================END:Callback functions to get robot states================

//====================Velocity command and adaptation===========================

void MotionGenerator::ComputeDesiredVelocity() {

	mutex_.lock();
	
	Eigen::Vector3d pose = real_pose_ - target_offset_; 

	desired_velocity_ = GetVelocityFromPose(pose);

	if (std::isnan(desired_velocity_.squaredNorm())) {
		ROS_WARN_THROTTLE(1, "DS is generating NaN. Setting the output to zero.");
		desired_velocity_.setZero();

	}

	if (desired_velocity_.norm() > Velocity_limit_) {
		// set the velocity to the max norm
		desired_velocity_ = desired_velocity_ / desired_velocity_.norm();
		desired_velocity_ *= Velocity_limit_;
	}

	//shitty way of getting the filter
	MathLib::Vector tmp(3);
	tmp(X) = desired_velocity_(X);
	tmp(Y) = desired_velocity_(Y);
	tmp(Z) = desired_velocity_(Z);
	filter_->SetTarget(tmp);
	filter_->Update();
	filter_->GetState(tmp);
	desired_velocity_filtered_(X)= tmp(X);
	desired_velocity_filtered_(Y)= tmp(Y);
	desired_velocity_filtered_(Z)= tmp(Z);


	mutex_.unlock();

}


void MotionGenerator::PublishDesiredVelocity() {


	msg_desired_velocity_.header.stamp = ros::Time::now();

	msg_desired_velocity_.twist.linear.x  = desired_velocity_(X);
	msg_desired_velocity_.twist.linear.y  = desired_velocity_(Y);
	msg_desired_velocity_.twist.linear.z  = desired_velocity_(Z);

	msg_desired_velocity_.twist.angular.x = 0;
	msg_desired_velocity_.twist.angular.y = 0;
	msg_desired_velocity_.twist.angular.z = 0;

	pub_desired_twist_.publish(msg_desired_velocity_.twist);


	msg_desired_velocity_filtered_.header.stamp = ros::Time::now();

	msg_desired_velocity_filtered_.twist.linear.x  = desired_velocity_filtered_(X);
	msg_desired_velocity_filtered_.twist.linear.y  = desired_velocity_filtered_(Y);
	msg_desired_velocity_filtered_.twist.linear.z  = desired_velocity_filtered_(Z);

	msg_desired_velocity_filtered_.twist.angular.x = 0;
	msg_desired_velocity_filtered_.twist.angular.y = 0;
	msg_desired_velocity_filtered_.twist.angular.z = 0;

	pub_desired_twist_filtered_.publish(msg_desired_velocity_filtered_.twist);

}


void* MotionGenerator::startAdaptationLoop(void* ptr)
{
	reinterpret_cast<MotionGenerator *>(ptr)->adaptationLoop(); 
}


void MotionGenerator::adaptationLoop()
{
	while(startThread_adaptation_)
	{
		if(gotFirstPosition_ && !paused_)
		{
			AdaptTrajectoryParameters(real_pose_ - target_offset_);
		}
	}
	std::cerr << "END adaptation thread" << std::endl;
}

//====================END:Velocity command and adaptation=======================

//==============Forward integration and publishing of estimated Path============

void MotionGenerator::PublishFuturePath() {

	// setting the header of the path
	msg_DesiredPath_.header.stamp = ros::Time::now();
	msg_DesiredPath_.header.frame_id = "world";


	Eigen::Vector3d simulated_pose = real_pose_;
	Eigen::Vector3d simulated_vel;


	for (int frame = 0; frame < MAX_FRAME; frame++)
	{

		Eigen::Vector3d pose = simulated_pose - target_offset_;

		simulated_vel = GetVelocityFromPose(pose);

		if (simulated_vel.norm() > Velocity_limit_) {
			simulated_vel = simulated_vel / simulated_vel.norm();
			simulated_vel *= Velocity_limit_;
		}

		simulated_pose(X) +=  simulated_vel(X) * dt_ * 20;
		simulated_pose(Y) +=  simulated_vel(Y) * dt_ * 20;
		simulated_pose(Z) +=  simulated_vel(Z) * dt_ * 20;

		msg_DesiredPath_.poses[frame].header.stamp = ros::Time::now();
		msg_DesiredPath_.poses[frame].header.frame_id = "world";
		msg_DesiredPath_.poses[frame].pose.position.x = simulated_pose(X);
		msg_DesiredPath_.poses[frame].pose.position.y = simulated_pose(Y);
		msg_DesiredPath_.poses[frame].pose.position.z = simulated_pose(Z);

		pub_DesiredPath_.publish(msg_DesiredPath_);
	}
}

void* MotionGenerator::startPathPublishingLoop(void* ptr)
{
	reinterpret_cast<MotionGenerator *>(ptr)->pathPublishingLoop(); 
}


void MotionGenerator::pathPublishingLoop()
{
	while(startThread_futurePath_)
	{
		if(gotFirstPosition_ && !paused_)
		{
			PublishFuturePath();
		}
	}
	std::cerr << "END path publishing thread" << std::endl;
}

//============END:Forward integration and publishing of estimated Path==========

void MotionGenerator::stopNode(int sig)
{
	ROS_INFO("Catched the ctrl C");
	me->stop_ = true;
}


void MotionGenerator::pauseNode(){
	paused_ = true;
}

void MotionGenerator::unpauseNode(){
	paused_ = false;
}