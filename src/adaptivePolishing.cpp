#include "adaptivePolishing.h"

#define POWER_THRESHOLD 0.4
#define FORCE_THRESHOLD 10.0
#define WORKSPACE_UP_BOUND 10
#define numParams 6

enum Coordinate { X=0 , Y=1 , Z=2 };
enum Params {SEMI_AXIS_A ,SEMI_AXIS_B,ALPHA,OFFSET_X,OFFSET_Y,OFFSET_Z};

AdaptivePolishing::AdaptivePolishing(ros::NodeHandle &n,
		double frequency,
		std::string input_rob_pose_topic_name,
		std::string input_rob_vel_topic_name,
		std::string input_rob_acc_topic_name,
		std::string input_rob_force_topic_name,
		std::string output_vel_topic_name,
		std::string output_filtered_vel_topic_name,
		std::vector<double> CenterRotation,
		double Cycle_radius,
		double major_axis_scale,
		double minor_axis_scale,
		double alpha,
		std::vector<double> adaptable_parameters,
		double RotationSpeed,
		double ConvergenceRate
)
	:MotionGenerator(n, frequency, input_rob_pose_topic_name,
	input_rob_vel_topic_name, input_rob_acc_topic_name,
	input_rob_force_topic_name, output_vel_topic_name,
	output_filtered_vel_topic_name, true), Cycle_speed_(RotationSpeed),
	Cycle_speed_offset_(0), Convergence_Rate_(ConvergenceRate),
	Convergence_Rate_scale_(1),Cycle_radius_(Cycle_radius)
{
	parameters_.resize(numParams);

	parameters_[SEMI_AXIS_A].val = minor_axis_scale;
	parameters_[SEMI_AXIS_A].adapt = adaptable_parameters[SEMI_AXIS_A];
	parameters_[SEMI_AXIS_A].scale = 100;

	parameters_[SEMI_AXIS_B].val = major_axis_scale;
	parameters_[SEMI_AXIS_B].adapt = adaptable_parameters[SEMI_AXIS_B];
	parameters_[SEMI_AXIS_A].scale = 100;

	parameters_[ALPHA].val = alpha;
	parameters_[ALPHA].adapt = adaptable_parameters[ALPHA];

	parameters_[OFFSET_X].val = CenterRotation[X];
	parameters_[OFFSET_X].adapt = adaptable_parameters[OFFSET_X];
	parameters_[OFFSET_X].scale = 1;

	parameters_[OFFSET_Y].val = CenterRotation[Y];
	parameters_[OFFSET_Y].adapt = adaptable_parameters[OFFSET_Y];
	parameters_[OFFSET_Z].scale = 1;

	parameters_[OFFSET_Z].val = CenterRotation[Z];
	parameters_[OFFSET_Z].adapt = adaptable_parameters[OFFSET_Z];
	parameters_[OFFSET_Z].scale = 1;

	ROS_INFO_STREAM("AP.CPP: Adaptive polishing node is created at: " <<
			nh_.getNamespace() << " with freq: " << frequency << "Hz");

	pub_cycle_target_ = nh_.advertise<geometry_msgs::Pose>(
			"DS/adaptivePolishing/cycle_target", 1000, 1);

	dyn_rec_f_ = boost::bind(&AdaptivePolishing::DynCallback, this, _1, _2);
	dyn_rec_srv_.setCallback(dyn_rec_f_);
}


Eigen::Vector3d AdaptivePolishing::GetVelocityFromPose(Eigen::Vector3d pose)
{
	// msg_cycle_target_.position.x = Cycle_Target_(X);
	// msg_cycle_target_.position.y = Cycle_Target_(Y);
	// msg_cycle_target_.position.z = Cycle_Target_(Z);
	// msg_cycle_target_.orientation.x = 0;
	// msg_cycle_target_.orientation.y = 0;
	// msg_cycle_target_.orientation.z = 0;
	// msg_cycle_target_.orientation.w = 0;
	// pub_cycle_target_.publish(msg_cycle_target_);

	Eigen::Vector3d Cycle_Target;
	Cycle_Target << parameters_[OFFSET_X].val,
			parameters_[OFFSET_Y].val,
			parameters_[OFFSET_Z].val;

	double alpha = parameters_[ALPHA].val;

	Eigen::Matrix3d rot;
	rot <<  cos(alpha), -sin(alpha), 0,
			sin(alpha),  cos(alpha), 0,
			0		  ,  0		   , 1;

	Eigen::Matrix3d inv_rot;
	inv_rot <<  cos(alpha), sin(alpha), 0,
				-sin(alpha),  cos(alpha), 0,
					0	  ,  0		   , 1;

	double a = parameters_[SEMI_AXIS_A].val;
	double b = parameters_[SEMI_AXIS_B].val;
	
	Eigen::Matrix3d scale;
	scale << a, 0, 0,
			 0, b, 0,
			 0, 0, 1;

	Eigen::Matrix3d inv_scale;
	inv_scale << 1/a, 0  , 0,
				 0  , 1/b, 0,
				 0  , 0  , 1;


	Eigen::Vector3d output_velocity;


	Eigen::Vector3d error_pose = inv_scale * inv_rot * (pose - Cycle_Target);

	double R = sqrt(error_pose(X) * error_pose(X) + error_pose(Y) * error_pose(Y));
	double T = atan2(error_pose(Y), error_pose(X));

	double Rdot = -Convergence_Rate_ * Convergence_Rate_scale_ * (R - Cycle_radius_);
	double Tdot = Cycle_speed_ + Cycle_speed_offset_;

	double x_vel = Rdot * cos(T) - R * Tdot * sin(T);
	double y_vel = Rdot * sin(T) + R * Tdot * cos(T);
	double z_vel = -Convergence_Rate_ * Convergence_Rate_scale_ * error_pose(Z);

	output_velocity(X) = x_vel;
	output_velocity(Y) = y_vel;
	output_velocity(Z) = z_vel;

	output_velocity = rot * scale * output_velocity;

	return output_velocity;
}

void AdaptivePolishing::DynCallback(
		adaptive_polishing::polishing_paramsConfig &config, uint32_t level) {


	ROS_INFO("Reconfigure request. Updatig the parameters ...");

	filter_Wn_ = config.Wn;

	filter_->SetWn(filter_Wn_);

	filter_dxLim_(X) = config.fil_dx_lim;
	filter_dxLim_(Y) = config.fil_dx_lim;
	filter_dxLim_(Z) = config.fil_dx_lim;

	filter_->SetVelocityLimits(filter_dxLim_);

	filter_ddxLim_(X) = config.fil_ddx_lim;
	filter_ddxLim_(Y) = config.fil_ddx_lim;
	filter_ddxLim_(Z) = config.fil_ddx_lim;
	filter_->SetAccelLimits(filter_ddxLim_);

	target_offset_(X) = config.offset_x;
	target_offset_(Y) = config.offset_y;
	target_offset_(Z) = config.offset_z;

	//Cycle_radius_scale_ = config.radius_scale;
	Cycle_speed_offset_ = config.Speed_offset;
	Convergence_Rate_scale_ = config.ConvergenceSpeed;
	Velocity_limit_ = config.vel_trimming;
	Grad_desc_epsilon_ = config.grad_descent_epsilon; 
	Grad_desc_step_ = config.grad_descent_step;

	// if (Cycle_radius_scale_ < 0) {
	// 	ROS_ERROR("RECONFIGURE: The scaling factor for radius cannot be negative!");
	// }

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

void AdaptivePolishing::AdaptTrajectoryParameters(Eigen::Vector3d pose){

	//if outside of workspace adapt if force is applied to rorobot 
	if(real_pose_(Z) > WORKSPACE_UP_BOUND){

		if(rob_sensed_force_.norm() < FORCE_THRESHOLD){
			//return;
		}
	// if inside workspace adapt if there is a power exchange
	}else{

		double power = rob_sensed_force_.dot(real_vel_);
		if(power < POWER_THRESHOLD){
			return;
		}
	}

	if(Grad_desc_step_ == 0){
		return;
	}

	//ROS_INFO("I'm adapting");
	//ROS_INFO("Detected human interaction, updatig the trajectory parameters ...");
	// get the error on the velocity
	Eigen::Vector3d error_vel = desired_velocity_ - real_vel_;

	//double cost_function_J = 1/2 * error_vel.squaredNorm();

	double grad_J;
	Eigen::Vector3d err1;
	Eigen::Vector3d err2;
	
	for(auto& param : parameters_){
		if(param.adapt)
		{
			//compute backward derivative
			param.val -= Grad_desc_step_;
			err1 = GetVelocityFromPose(pose);
			param.val += Grad_desc_step_;
			//compute forward derivative
			param.val += Grad_desc_step_;
			err2 = GetVelocityFromPose(pose);
			param.val -= Grad_desc_step_;

			//compute gradient
			grad_J = error_vel.dot((err1-err2)/(2*Grad_desc_step_));
			//modify the concerned parameter
			param.val += Grad_desc_epsilon_*grad_J*param.scale;
			ROS_INFO_STREAM("Grad is:" << grad_J);
		}
	}
	

	//Eigen::Vector2d grad_J = ComputeGradient(error_vel,pose);

	//Cycle_Target_(X) += Grad_desc_epsilon_*grad_J(X);
	//Cycle_Target_(Y) += Grad_desc_epsilon_*grad_J(Y);

	 // ROS_INFO_STREAM("gradJx: " << grad_J(X) << " gradJy: " << grad_J(Y) );

	// msg_cycle_target_.position.x = Cycle_Target_(X);
	// msg_cycle_target_.position.y = Cycle_Target_(Y);
	// msg_cycle_target_.position.z = Cycle_Target_(Z);
	// msg_cycle_target_.orientation.x = 0;
	// msg_cycle_target_.orientation.y = 0;
	// msg_cycle_target_.orientation.z = 0;
	// msg_cycle_target_.orientation.w = 0;

	// pub_cycle_target_.publish(msg_cycle_target_);

}

Eigen::Vector2d AdaptivePolishing::ComputeGradient(Eigen::Vector3d error_vel,Eigen::Vector3d pose){


	//initialise the gradient vector
	Eigen::Vector2d grad;

	Eigen::Vector3d err1;
	Eigen::Vector3d err2;

	Cycle_Target_(X) -= Grad_desc_step_;
	err1 = GetVelocityFromPose(pose);
	Cycle_Target_(X) += Grad_desc_step_;

	Cycle_Target_(X) += Grad_desc_step_;
	err2 = GetVelocityFromPose(pose);
	Cycle_Target_(X) -= Grad_desc_step_;

	grad(X) = error_vel.dot((err1-err2)/(2*Grad_desc_step_));

	Cycle_Target_(Y) -= Grad_desc_step_;
	err1 = GetVelocityFromPose(pose);
	Cycle_Target_(Y) += Grad_desc_step_;

	Cycle_Target_(Y) += Grad_desc_step_;
	err2 = GetVelocityFromPose(pose);
	Cycle_Target_(Y) -= Grad_desc_step_;

	grad(Y) = error_vel.dot((err1-err2)/(2*Grad_desc_step_));

	// ROS_INFO_STREAM("gradJx: " << grad(X) << " gradJy: " << grad(Y) );

	return grad;
}




