#include "adaptive_polishing.h"






Adaptive_polishing::Adaptive_polishing(ros::NodeHandle &n,
                                       double frequency,
                                       std::string input_topic_name,
                                       std::string output_topic_name,
                                       std::string output_filtered_topic_name,
                                       std::vector<double> CenterRotation,
        							   double radius,
        							   double RotationSpeed,
									   double ConvergenceRate
                                      )
	: nh_(n),
	  loop_rate_(frequency),
	  input_topic_name_(input_topic_name),
	  output_topic_name_(output_topic_name),
	  output_filtered_topic_name_(output_filtered_topic_name),
	  filter_Wn_(.42),
	  dt_(1 / frequency),
	  Cycle_Target_(CenterRotation),
	  Cycle_radius_(radius),
	  Cycle_radius_scale_(1),
	  Cycle_speed_(RotationSpeed),
	  Cycle_speed_offset_(0),
	  Convergence_Rate_(ConvergenceRate),
	  Convergence_Rate_scale_(1),
	  Velocity_limit_(0) 
{

	ROS_INFO_STREAM("AP.CPP: Adaptive polishing node is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");
}


bool Adaptive_polishing::Init() {



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




	// if (!InitializeDS()) {
	// 	ROS_ERROR_STREAM("ERROR intializing the DS");
	// 	return false;
	// }

	if (!InitializeROS()) {
		ROS_ERROR_STREAM("ERROR intializing ROS variables");
		return false;
	}

	return true;
}



bool Adaptive_polishing::InitializeROS() {

	sub_real_pose_ = nh_.subscribe(input_topic_name_ , 1000,
	                               &Adaptive_polishing::UpdateRealPosition, this,
	                               ros::TransportHints().reliable().tcpNoDelay());

	pub_desired_twist_ = nh_.advertise<geometry_msgs::Twist>(output_topic_name_, 1000, 1);
	pub_desired_twist_filtered_ = nh_.advertise<geometry_msgs::Twist>(output_filtered_topic_name_, 1);

	// pub_desired_twist_ = nh_.advertise<geometry_msgs::TwistStamped>(output_topic_name_, 1);
	// pub_desired_twist_filtered_ = nh_.advertise<geometry_msgs::TwistStamped>(output_filtered_topic_name_, 1);


	dyn_rec_f_ = boost::bind(&Adaptive_polishing::DynCallback, this, _1, _2);
	dyn_rec_srv_.setCallback(dyn_rec_f_);




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


void Adaptive_polishing::Run() {

	while (nh_.ok()) {

		ComputeDesiredVelocity();

		PublishDesiredVelocity();

		// PublishFuturePath();
		//pub_desired_twist_.pub()

		ros::spinOnce();

		loop_rate_.sleep();
	}
}


void Adaptive_polishing::ComputeDesiredVelocity() {

	mutex_.lock();

	
	MathLib::Vector pose = real_pose_ - target_pose_  - target_offset_;

	double x_vel = 0;
	double y_vel = 0;
	double z_vel = - Convergence_Rate_ * Convergence_Rate_scale_ * pose(2);

	double R = sqrt(pose(0) * pose(0) + pose(1) * pose(1));
	double T = atan2(pose(1), pose(0));

	double Rdot = - Convergence_Rate_ * Convergence_Rate_scale_ * (R - Cycle_radius_ * Cycle_radius_scale_);
	double Tdot = Cycle_speed_ + Cycle_speed_offset_;


	x_vel = Rdot * cos(T) - R * Tdot * sin(T);
	y_vel = Rdot * sin(T) + R * Tdot * cos(T);

	desired_velocity_(0) = x_vel;
	desired_velocity_(1) = y_vel;
	desired_velocity_(2) = z_vel;

	if (std::isnan(desired_velocity_.Norm2())) {
		ROS_WARN_THROTTLE(1, "DS is generating NaN. Setting the output to zero.");
		desired_velocity_.Zero();
	}

	if (desired_velocity_.Norm() > Velocity_limit_) {
		desired_velocity_ = desired_velocity_ / desired_velocity_.Norm() * Velocity_limit_;
	}




	// MathLib::Vector pose = real_pose_ - target_pose_  - target_offset_;


	// double R = sqrt(real_pose_(0) * real_pose_(0) + real_pose_(1) * real_pose_(1));
	// double T = atan2(real_pose_(1), real_pose_(0));

	// // speed in polar coordinates
	// double Rdot = -1 * (R - 1);
	// double Tdot = 1;


	// double x_vel = Rdot * cos(T) - R * Tdot * sin(T);
	// double y_vel = Rdot * sin(T) + R * Tdot * cos(T);
	// double z_vel = - 1 * real_pose_(2);

	// // filling in the Mathlib::vector
	// desired_velocity_(0) = x_vel;
	// desired_velocity_(1) = y_vel;
	// desired_velocity_(2) = z_vel;

	// if (std::isnan(desired_velocity_.Norm2())) {
	// 	ROS_WARN_THROTTLE(1, "DS is generating NaN. Setting the output to zero.");
	// 	desired_velocity_.Zero();
	// }

	// if (desired_velocity_.Norm() > Velocity_limit_) {
	// 	desired_velocity_ = desired_velocity_ / desired_velocity_.Norm() * Velocity_limit_;
	// }


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



void Adaptive_polishing::UpdateRealPosition(const geometry_msgs::Pose::ConstPtr& msg) {

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

void Adaptive_polishing::PublishDesiredVelocity() {

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



void Adaptive_polishing::DynCallback(adaptive_polishing::polishing_paramsConfig &config, uint32_t level) {


	ROS_INFO("Reconfigure request. Updatig the parameters ...");

	filter_Wn_ = config.Wn;
	filter_->SetWn(filter_Wn_);


	filter_dxLim_(0) = config.fil_dx_lim;
	filter_dxLim_(1) = config.fil_dx_lim;
	filter_dxLim_(2) = config.fil_dx_lim;
	filter_->SetVelocityLimits(filter_dxLim_);

	filter_ddxLim_(0) = config.fil_ddx_lim;
	filter_ddxLim_(1) = config.fil_ddx_lim;
	filter_ddxLim_(2) = config.fil_ddx_lim;
	filter_->SetAccelLimits(filter_ddxLim_);



	target_offset_(0) = config.offset_x;
	target_offset_(1) = config.offset_y;
	target_offset_(2) = config.offset_z;

	Cycle_radius_scale_ = config.radius_scale;
	Cycle_speed_offset_ = config.Speed_offset;
	Convergence_Rate_scale_ = config.ConvergenceSpeed;
	Velocity_limit_ = config.vel_trimming;

	if (Cycle_radius_scale_ < 0) {
		ROS_ERROR("RECONFIGURE: The scaling factor for radius cannot be negative!");
	}

	if (Convergence_Rate_scale_ < 0) {
		ROS_ERROR("RECONFIGURE: The scaling factor for convergence rate cannot be negative!");
	}

	if (Velocity_limit_ < 0) {
		ROS_ERROR("RECONFIGURE: The limit for velocity cannot be negative!");
	}



	// target_offset_(0) = config.offset_x;
	// target_offset_(1) = config.offset_y;
	// target_offset_(2) = config.offset_z;

	// scaling_factor_ = config.scaling;
	// ds_vel_limit_   = config.trimming;

}
