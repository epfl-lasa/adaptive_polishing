#include "MotionGenerator.h"






MotionGenerator::MotionGenerator(ros::NodeHandle &n,
                                       double frequency,
                                       std::string input_rob_pose_topic_name,
                                       std::string output_vel_topic_name,
                                       std::string output_filtered_vel_topic_name,
                                       std::string input_rob_vel_topic_name,
									   std::string input_rob_force_ee_topic_name
                                      )
	: nh_(n),
	  loop_rate_(frequency),
	  input_rob_pose_topic_name_(input_rob_pose_topic_name),
	  output_vel_topic_name_(output_vel_topic_name),
	  output_filtered_vel_topic_name_(output_filtered_vel_topic_name),
	  input_rob_vel_topic_name_(input_rob_vel_topic_name),
	  input_rob_force_ee_topic_name_(input_rob_force_ee_topic_name),
	  filter_Wn_(.42),
	  dt_(1 / frequency),
	  Velocity_limit_(1) 
{
	Init();
	ROS_INFO_STREAM("MotionGenerator.CPP: MotionGenerator node is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");
}


bool MotionGenerator::Init() {



	real_pose_.Resize(3);
	desired_velocity_.Resize(3);
	desired_velocity_filtered_.Resize(3);


	// initializing the filter
	filter_.reset (new CDDynamics(3, dt_, filter_Wn_));

	// we should set the size automagically
	filter_dxLim_.Resize(3);
	filter_dxLim_(0) = 1;
	filter_dxLim_(1) = 1;
	filter_dxLim_(2) = 1;

	filter_->SetVelocityLimits(filter_dxLim_);

	filter_ddxLim_.Resize(3);
	filter_ddxLim_(0) = .51;
	filter_ddxLim_(1) = .51;
	filter_ddxLim_(2) = .51;
	filter_->SetAccelLimits(filter_ddxLim_);


	target_pose_.Resize(3);
	target_pose_.Zero();
	target_offset_.Resize(3);
	target_offset_.Zero();

	MathLib::Vector initial(3);

	initial.Zero();

	filter_->SetState(initial);
	filter_->SetTarget(initial);

	if (!InitializeROS()) {
		ROS_ERROR_STREAM("ERROR intializing ROS variables");
		return false;
	}

	return true;
}



bool MotionGenerator::InitializeROS() {

	sub_real_pose_ = nh_.subscribe(input_rob_pose_topic_name_ , 1000,
	                               &MotionGenerator::UpdateRealPosition, this,
	                               ros::TransportHints().reliable().tcpNoDelay());

	pub_desired_twist_ = nh_.advertise<geometry_msgs::Twist>(output_vel_topic_name_, 1000, 1);
	pub_desired_twist_filtered_ = nh_.advertise<geometry_msgs::Twist>(output_filtered_vel_topic_name_, 1);

	// pub_desired_twist_ = nh_.advertise<geometry_msgs::TwistStamped>(output_vel_topic_name__, 1);
	// pub_desired_twist_filtered_ = nh_.advertise<geometry_msgs::TwistStamped>(output_filtered_vel_topic_name_, 1);


	// targt and desired path publishing with max size of output
	pub_target_ = nh_.advertise<geometry_msgs::PointStamped>("DS/target", 1);
	pub_DesiredPath_ = nh_.advertise<nav_msgs::Path>("DS/DesiredPath", 1);
	msg_DesiredPath_.poses.resize(MAX_FRAME);

	if (nh_.ok()) { // Wait for poses being published
		ros::spinOnce();
		ROS_INFO("The Motion generator is ready.");
		return true;
	}
	else {
		ROS_ERROR("The ros node has a problem.");
		return false;
	}

}


void MotionGenerator::Run() {

	while (nh_.ok()) {

		ComputeDesiredVelocity();

		PublishDesiredVelocity();

		PublishFuturePath();
		//pub_desired_twist_.pub()

		ros::spinOnce();

		loop_rate_.sleep();
	}
}


void MotionGenerator::ComputeDesiredVelocity() {

	mutex_.lock();
	
	MathLib::Vector pose = real_pose_ - target_pose_  - target_offset_;

	desired_velocity_ = GetVelocityFromPose(pose);

	if (std::isnan(desired_velocity_.Norm2())) {
		ROS_WARN_THROTTLE(1, "DS is generating NaN. Setting the output to zero.");
		desired_velocity_.Zero();
	}

	if (desired_velocity_.Norm() > Velocity_limit_) {
		desired_velocity_ = desired_velocity_ / desired_velocity_.Norm() * Velocity_limit_;
	}


	// msg_desired_velocity_.header.stamp = ros::Time::now();
	// msg_desired_velocity_.twist.linear.x  = desired_velocity_(0);
	// msg_desired_velocity_.twist.linear.y  = desired_velocity_(1);
	// msg_desired_velocity_.twist.linear.z  = desired_velocity_(2);

	// msg_desired_velocity_.twist.angular.x = 0;
	// msg_desired_velocity_.twist.angular.y = 0;
	// msg_desired_velocity_.twist.angular.z = 0;

	 filter_->SetTarget(desired_velocity_);
	 filter_->Update();
	 filter_->GetState(desired_velocity_filtered_);



	mutex_.unlock();

}




void MotionGenerator::UpdateRealPosition(const geometry_msgs::Pose::ConstPtr& msg) {

	// msg_real_pose_ = *msg;

	// ROS_INFO_STREAM (
	//     "x: " << msg->position.x << "\t" <<
	//     "y: " << msg->position.y << "\t" <<
	//     "z: " << msg->position.z
	// );


	real_pose_(0) = msg->position.x;
	real_pose_(1) = msg->position.y;
	real_pose_(2) = msg->position.z;

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

void MotionGenerator::PublishDesiredVelocity() {


	msg_desired_velocity_.header.stamp = ros::Time::now();

	msg_desired_velocity_.twist.linear.x  = desired_velocity_(0);
	msg_desired_velocity_.twist.linear.y  = desired_velocity_(1);
	msg_desired_velocity_.twist.linear.z  = desired_velocity_(2);

	msg_desired_velocity_.twist.angular.x = 0;
	msg_desired_velocity_.twist.angular.y = 0;
	msg_desired_velocity_.twist.angular.z = 0;

	pub_desired_twist_.publish(msg_desired_velocity_.twist);



	msg_desired_velocity_filtered_.header.stamp = ros::Time::now();

	msg_desired_velocity_filtered_.twist.linear.x  = desired_velocity_filtered_(0);
	msg_desired_velocity_filtered_.twist.linear.y  = desired_velocity_filtered_(1);
	msg_desired_velocity_filtered_.twist.linear.z  = desired_velocity_filtered_(2);

	msg_desired_velocity_filtered_.twist.angular.x = 0;
	msg_desired_velocity_filtered_.twist.angular.y = 0;
	msg_desired_velocity_filtered_.twist.angular.z = 0;

	pub_desired_twist_filtered_.publish(msg_desired_velocity_filtered_.twist);






	//pub_desired_twist_filtered_.publish(msg_desired_velocity_filtered_);

}


void MotionGenerator::PublishFuturePath() {

	geometry_msgs::PointStamped msg;

	msg.header.frame_id = "world";
	msg.header.stamp = ros::Time::now();
	msg.point.x = target_pose_[0] + target_offset_[0];
	msg.point.y = target_pose_[1] + target_offset_[1];
	msg.point.z = target_pose_[2] + target_offset_[2];

	pub_target_.publish(msg);

	// create a temporary message


	// setting the header of the path
	msg_DesiredPath_.header.stamp = ros::Time::now();
	msg_DesiredPath_.header.frame_id = "world";




	MathLib::Vector simulated_pose = real_pose_;
	MathLib::Vector simulated_vel;
	simulated_vel.Resize(3);

	for (int frame = 0; frame < MAX_FRAME; frame++)
	{

		MathLib::Vector pose = simulated_pose - target_pose_  - target_offset_;

		simulated_vel = GetVelocityFromPose(pose);



		if (simulated_vel.Norm() > Velocity_limit_) {
			simulated_vel = simulated_vel / simulated_vel.Norm() * Velocity_limit_;
		}

		simulated_pose[0] +=  simulated_vel[0] * dt_ * 20;
		simulated_pose[1] +=  simulated_vel[1] * dt_ * 20;
		simulated_pose[2] +=  simulated_vel[2] * dt_ * 20;

		msg_DesiredPath_.poses[frame].header.stamp = ros::Time::now();
		msg_DesiredPath_.poses[frame].header.frame_id = "world";
		msg_DesiredPath_.poses[frame].pose.position.x = simulated_pose[0];
		msg_DesiredPath_.poses[frame].pose.position.y = simulated_pose[1];
		msg_DesiredPath_.poses[frame].pose.position.z = simulated_pose[2];

		pub_DesiredPath_.publish(msg_DesiredPath_);
	}
}
