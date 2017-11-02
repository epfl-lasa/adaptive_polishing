#include "MotionGenerator.h"


enum Coordinate { X=0 , Y=1 , Z=2 };

MotionGenerator* MotionGenerator::me = NULL;

MotionGenerator::MotionGenerator(ros::NodeHandle &n,
                                	double frequency,
                                	std::string input_rob_pose_topic_name,
                                	std::string input_rob_vel_topic_name,
                                	std::string input_rob_acc_topic_name,
                                	std::string input_rob_force_topic_name,
                                	std::string output_vel_topic_name,
                                	std::string output_filtered_vel_topic_name
                                )
:nh_(n),
loop_rate_(frequency),
filter_Wn_(.42),
dt_(1 / frequency),
Velocity_limit_(1) 
{

	me = this;

	// Allows to catch CTRL+C
	signal(SIGINT,MotionGenerator::stopNode);

	//ROS Topic initialization
	sub_real_pose_ = nh_.subscribe(input_rob_pose_topic_name , 1000,
	                               &MotionGenerator::UpdateRealPosition, this,
	                               ros::TransportHints().reliable().tcpNoDelay());
	sub_real_vel_ = nh_.subscribe(input_rob_vel_topic_name , 1000,
	                               &MotionGenerator::UpdateRealVelocity, this,
	                               ros::TransportHints().reliable().tcpNoDelay());
	sub_real_acc_ = nh_.subscribe(input_rob_acc_topic_name , 1000,
	                               &MotionGenerator::UpdateRealAcceleration, this,
	                               ros::TransportHints().reliable().tcpNoDelay());
	sub_robot_force_ = nh_.subscribe(input_rob_force_topic_name , 1000,
	                                  &MotionGenerator::UpdateRobotSensedForce, this,
	                                  ros::TransportHints().reliable().tcpNoDelay());

	pub_desired_twist_ = nh_.advertise<geometry_msgs::Twist>(output_vel_topic_name, 1000, 1);
	pub_desired_twist_filtered_ = nh_.advertise<geometry_msgs::Twist>(output_filtered_vel_topic_name, 1);



	// target and desired path publishing with max size of output
	pub_target_ = nh_.advertise<geometry_msgs::PointStamped>("DS/target", 1);
	pub_DesiredPath_ = nh_.advertise<nav_msgs::Path>("DS/DesiredPath", 1);
	pub_human_action_ = nh_.advertise<geometry_msgs::WrenchStamped>("DS/human_force",1);

	msg_DesiredPath_.poses.resize(MAX_FRAME);

	if (nh_.ok()) { // Wait for poses being published
		ros::spinOnce();
		ROS_INFO("The Motion generator is ready.");
	}
	// else {
	// 	ROS_ERROR("The ros node has a problem.");
	// }


	Init();
	ROS_INFO_STREAM("MotionGenerator.CPP: MotionGenerator node is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");
}


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

	// target_pose_.Resize(3);
	// target_pose_.Zero();
	//target_pose_.reset( new Eigen::Vector3d( Eigen::Vector3d::Zero() ) );
	// target_offset_.Resize(3);
	// target_offset_.Zero();
	target_offset_.setZero();
	rob_sensed_force.setZero();
	//real_pose_.Resize(3);
	// real_vel_.Resize(3);
	// real_acc_.Resize(3);
	// desired_velocity_.Resize(3);
	// desired_velocity_filtered_.Resize(3);

	//thread for futur path
	startThread_ = true;
	if(pthread_create(&thread_, NULL, &MotionGenerator::startPathPublishingLoop, this))
	{
		throw std::runtime_error("Cannot create reception thread");  
	}

	return true;
}

void MotionGenerator::Run() {

	while (!stop_) {

		if(gotFirstPosition_){

			ComputeDesiredVelocity();

			PublishDesiredVelocity();
		}

		// PublishFuturePath();
		//pub_desired_twist_.pub()

		ros::spinOnce();

		loop_rate_.sleep();
	}

	desired_velocity_.setZero();
	PublishDesiredVelocity();
	ros::spinOnce();
	loop_rate_.sleep();

	startThread_ = false;
	pthread_join(thread_,NULL);

	ros::shutdown();
}

void MotionGenerator::stopNode(int sig)
{
	ROS_INFO("Catched the ctrl C");
	me->stop_ = true;
}


void MotionGenerator::ComputeDesiredVelocity() {

	mutex_.lock();
	
	// MathLib::Vector pose = real_pose_ - target_pose_ - target_offset_;
	Eigen::Vector3d pose = real_pose_ - target_offset_; //*target_pose_ - target_offset_;

	desired_velocity_ = GetVelocityFromPose(pose);

	//TODO set this as a parameter
	//bool adaptable = 0;
	if(rob_sensed_force.segment(0,2).norm()>FORCE_THRESHOLD){
		AdaptTrajectoryParameters(pose);
	}

	if (std::isnan(desired_velocity_.squaredNorm())) {
		ROS_WARN_THROTTLE(1, "DS is generating NaN. Setting the output to zero.");
		desired_velocity_.setZero();

	}

	if (desired_velocity_.norm() > Velocity_limit_) {
		desired_velocity_ = desired_velocity_ / desired_velocity_.norm() * Velocity_limit_;
	}


	// msg_desired_velocity_.header.stamp = ros::Time::now();
	// msg_desired_velocity_.twist.linear.x  = desired_velocity_(0);
	// msg_desired_velocity_.twist.linear.y  = desired_velocity_(1);
	// msg_desired_velocity_.twist.linear.z  = desired_velocity_(2);

	// msg_desired_velocity_.twist.angular.x = 0;
	// msg_desired_velocity_.twist.angular.y = 0;
	// msg_desired_velocity_.twist.angular.z = 0;

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




void MotionGenerator::UpdateRealPosition(const geometry_msgs::Pose::ConstPtr& msg) {

	real_pose_(X) = msg->position.x;
	real_pose_(Y) = msg->position.y;
	real_pose_(Z) = msg->position.z;

	if(!gotFirstPosition_)
	{
		//desired_pose_ = real_pose_;
		//std:cerr << desired_pose_(2) << " " <<  real_pose_(2) << std::endl;

		gotFirstPosition_ = true;
	}
	// double qtx = msg_real_pose_.orientation.x;
	// double qty = msg_real_pose_.orientation.y;
	// double qtz = msg_real_pose_.orientation.z;
	// double qtw = msg_real_pose_.orientation.w;

	// tf::Quaternion q(qtx, qty, qtz, qtw);
	// tf::Matrix3x3 m(q);
	// double roll, pitch, yaw;
	// m.getRPY(roll, pitch, yaw);

	// real_pose_(3?) = roll;
	// real_pose_(4?) = pitch;
	// real_pose_(5?) = yaw;
}

void MotionGenerator::UpdateRealVelocity(const geometry_msgs::Twist::ConstPtr& msg){

	real_vel_(X) = msg->linear.x;
	real_vel_(Y) = msg->linear.y;
	real_vel_(Z) = msg->linear.z;

}

void MotionGenerator::UpdateRealAcceleration(const geometry_msgs::Accel::ConstPtr& msg){

	real_acc_(X) = msg->linear.x;
	real_acc_(Y) = msg->linear.y;
	real_acc_(Z) = msg->linear.z;
}

void MotionGenerator::UpdateRobotSensedForce(const geometry_msgs::WrenchStamped::ConstPtr& msg){
	rob_sensed_force(X) = msg->wrench.force.x;
	rob_sensed_force(Y) = msg->wrench.force.y;
	rob_sensed_force(Z) = msg->wrench.force.z;

	geometry_msgs::WrenchStamped msg_human_action_;
	msg_human_action_.header = msg->header;

	msg_human_action_.wrench.force.x = -rob_sensed_force(X);
	msg_human_action_.wrench.force.y = -rob_sensed_force(Y);
	msg_human_action_.wrench.force.z = -rob_sensed_force(Z);

	pub_human_action_.publish(msg_human_action_);
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


	//pub_desired_twist_filtered_.publish(msg_desired_velocity_filtered_);

}


void MotionGenerator::PublishFuturePath() {

	// geometry_msgs::PointStamped msg;

	// msg.header.frame_id = "world";
	// msg.header.stamp = ros::Time::now();
	// msg.point.x =  target_offset_(X); // + (*target_pose_)(X);
	// msg.point.y =  target_offset_(Y); // + (*target_pose_)(Y);
	// msg.point.z =  target_offset_(Z); // + (*target_pose_)(Z);

	// pub_target_.publish(msg);


	// setting the header of the path
	msg_DesiredPath_.header.stamp = ros::Time::now();
	msg_DesiredPath_.header.frame_id = "world";


	// MathLib::Vector simulated_pose = real_pose_;
	// MathLib::Vector simulated_vel;
	Eigen::Vector3d simulated_pose = real_pose_;
	Eigen::Vector3d simulated_vel;


	for (int frame = 0; frame < MAX_FRAME; frame++)
	{

		Eigen::Vector3d pose = simulated_pose - target_offset_;// - *target_pose_;

		simulated_vel = GetVelocityFromPose(pose);

		if (simulated_vel.norm() > Velocity_limit_) {
			simulated_vel = simulated_vel / simulated_vel.norm() * Velocity_limit_;
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
    while(startThread_)
    {
        if(gotFirstPosition_)
        {
            PublishFuturePath();   
        }
    }
    std::cerr << "END path publishing thread" << std::endl;
}