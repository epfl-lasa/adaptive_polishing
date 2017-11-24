#include "adaptivePolishing.h"

#define POWER_THRESHOLD 3.0
#define FORCE_THRESHOLD 10.0
#define WORKSPACE_UP_BOUND 10
#define NUM_PARAMS 6

//MACROS
#define SCALE(VAL, MIN, MAX) ( ((VAL)-(MIN)) / ((MAX)-(MIN)) )
#define SCALE_BACK(VAL, MIN, MAX) ( (VAL)*((MAX)-(MIN)) + (MIN) )
#define MIN(X, Y)  ((X) < (Y) ? (X) : (Y))
#define MAX(X, Y)  ((X) > (Y) ? (X) : (Y))

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
		std::vector<double> parameters,
		std::vector<double> min_parameters,
		std::vector<double> max_parameters,
		std::vector<double> adaptable_parameters,
		double RotationSpeed,
		double ConvergenceRate
)
	:MotionGenerator(n, frequency, input_rob_pose_topic_name,
	input_rob_vel_topic_name, input_rob_acc_topic_name,
	input_rob_force_topic_name, output_vel_topic_name,
	output_filtered_vel_topic_name, true), Cycle_speed_(RotationSpeed),
	Cycle_speed_offset_(0), Convergence_Rate_(ConvergenceRate),
	Convergence_Rate_scale_(1),Cycle_radius_(1)
{
	parameters_.resize(NUM_PARAMS);
	confidence_.resize(NUM_PARAMS);
	prev_grad_.resize(NUM_PARAMS);

	for(int i= 0; i<parameters_.size();i++){

		parameters_[i].val = parameters[i];
		parameters_[i].adapt = adaptable_parameters[i];
		parameters_[i].min = min_parameters[i];
		parameters_[i].max = max_parameters[i];
		parameters_[i].prev_grad = 0;
		parameters_[i].confidence = 1;

	}


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
	alpha = alpha/180*M_PI;

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
			double save = param.val;

			// normalize the parameter
			double tmp = param.val;
			tmp = SCALE(tmp,param.min,param.max);

			//compute backward derivative
			tmp -= Grad_desc_step_;
			param.val = SCALE_BACK(tmp,param.min,param.max);
			err1 = GetVelocityFromPose(pose);
			tmp += Grad_desc_step_;

			//compute forward derivative
			tmp += Grad_desc_step_;
			param.val = SCALE_BACK(tmp,param.min,param.max);
			err2 = GetVelocityFromPose(pose);
			tmp -= Grad_desc_step_;

			//compute gradient
			grad_J = error_vel.dot((err1-err2)/(2*Grad_desc_step_));
			param.confidence = p_*param.confidence + 
					(1-p_)*pow(grad_J - param.prev_grad,2);
					
			param.prev_grad = grad_J;

			param.confidence = MIN(param.confidence,0.01);
			//modify the concerned parameter
			tmp += (Grad_desc_epsilon_*grad_J)/param.confidence;
			param.val = SCALE_BACK(tmp,param.min,param.max);

			//set buondaries
			param.val = MIN(param.val,param.max);
			param.val = MAX(param.val,param.min);
		}
	}
}



