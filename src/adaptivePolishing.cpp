#include "adaptivePolishing.h"


enum Coordinate { X=0 , Y=1 , Z=2 };


AdaptivePolishing::AdaptivePolishing(ros::NodeHandle &n,
                                       double frequency,
                                       std::string input_rob_pose_topic_name,
                                       std::string output_vel_topic_name,
                                       std::string output_filtered_vel_topic_name,
                                       std::string input_rob_vel_topic_name,
									   std::string input_rob_force_ee_topic_name,
                                       std::vector<double> CenterRotation,
        							   double radius,
        							   double RotationSpeed,
									   double ConvergenceRate
                                      )
	: MotionGenerator(n, 
		frequency,
		input_rob_pose_topic_name,
		output_vel_topic_name,
		output_filtered_vel_topic_name,
		input_rob_vel_topic_name,
		input_rob_force_ee_topic_name),
	  Cycle_Target_(CenterRotation),
	  Cycle_radius_(radius),
	  Cycle_radius_scale_(1),
	  Cycle_speed_(RotationSpeed),
	  Cycle_speed_offset_(0),
	  Convergence_Rate_(ConvergenceRate),
	  Convergence_Rate_scale_(1)
{
	ROS_INFO_STREAM("AP.CPP: Adaptive polishing node is created at: " <<
						 nh_.getNamespace() << " with freq: " << frequency << "Hz");
	dyn_rec_f_ = boost::bind(&AdaptivePolishing::DynCallback, this, _1, _2);
	dyn_rec_srv_.setCallback(dyn_rec_f_);
}


MathLib::Vector AdaptivePolishing::GetVelocityFromPose(MathLib::Vector pose)
{
	MathLib::Vector output_velocity;
	output_velocity.Resize(3);

	double R = sqrt(pose(X) * pose(X) + pose(Y) * pose(Y));
	double T = atan2(pose(Y), pose(X));

	double Rdot = - Convergence_Rate_ * Convergence_Rate_scale_ * (R - Cycle_radius_ * Cycle_radius_scale_);
	double Tdot = Cycle_speed_ + Cycle_speed_offset_;

	double x_vel = Rdot * cos(T) - R * Tdot * sin(T);
	double y_vel = Rdot * sin(T) + R * Tdot * cos(T);
	double z_vel = - Convergence_Rate_ * Convergence_Rate_scale_ * pose(Z);

	output_velocity(X) = x_vel;
	output_velocity(Y) = y_vel;
	output_velocity(Z) = z_vel;

	return output_velocity;
}

void AdaptivePolishing::DynCallback(adaptive_polishing::polishing_paramsConfig &config, uint32_t level) {


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




