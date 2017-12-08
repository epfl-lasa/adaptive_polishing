#include "AttractorDS.h"

#define POWER_THRESHOLD 0.4
#define FORCE_THRESHOLD 10.0
#define WORKSPACE_UP_BOUND 10
#define NUM_PARAMS 3

//MACROS
#define SCALE(VAL, MIN, MAX) ( ((VAL)-(MIN)) / ((MAX)-(MIN)) )
#define SCALE_BACK(VAL, MIN, MAX) ( (VAL)*((MAX)-(MIN)) + (MIN) )
#define MIN(X, Y)  ((X) < (Y) ? (X) : (Y))
#define MAX(X, Y)  ((X) > (Y) ? (X) : (Y))

enum Coordinate { X=0 , Y=1 , Z=2 };
enum Params {OFFSET_X,OFFSET_Y,OFFSET_Z};


AttractorDS::AttractorDS(ros::NodeHandle &n,
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
		double ConvergenceRate,
		bool adaptable
)
	:MotionGenerator(n, frequency, input_rob_pose_topic_name,
	input_rob_vel_topic_name, input_rob_acc_topic_name,
	input_rob_force_topic_name, output_vel_topic_name,
	output_filtered_vel_topic_name, adaptable),
	Convergence_Rate_(ConvergenceRate),Convergence_Rate_scale_(1)
{
	parameters_.resize(NUM_PARAMS);

	for(int i= 0; i<parameters_.size();i++){
		parameters_[i].val = parameters[i];
		parameters_[i].adapt = adaptable_parameters[i];
		parameters_[i].min = min_parameters[i];
		parameters_[i].max = max_parameters[i];
	}

	ROS_INFO_STREAM("AttractorDS.CPP: Attractor node is created at: " <<
			nh_.getNamespace() << " with freq: " << frequency << "Hz");

	pub_attractor_target_ = nh_.advertise<geometry_msgs::Pose>(
			"DS/attractor/attractor_target", 1000, 1);

	dyn_rec_f_ = boost::bind(&AttractorDS::DynCallback, this, _1, _2);
	dyn_rec_srv_.setCallback(dyn_rec_f_);

}


Eigen::Vector3d AttractorDS::GetVelocityFromPose(Eigen::Vector3d pose)
{
	// msg_cycle_target_.position.x = Cycle_Target_(X);
	// msg_cycle_target_.position.y = Cycle_Target_(Y);
	// msg_cycle_target_.position.z = Cycle_Target_(Z);
	// msg_cycle_target_.orientation.x = 0;
	// msg_cycle_target_.orientation.y = 0;
	// msg_cycle_target_.orientation.z = 0;
	// msg_cycle_target_.orientation.w = 0;
	// pub_cycle_target_.publish(msg_cycle_target_);

	Eigen::Vector3d attractor_target;

	attractor_target << parameters_[OFFSET_X].val,
			parameters_[OFFSET_Y].val,
			parameters_[OFFSET_Z].val;

	Eigen::Vector3d output_velocity;

	Eigen::Vector3d error_pose = pose - attractor_target;

	double k = Convergence_Rate_ * Convergence_Rate_scale_;

	output_velocity = -k*Eigen::MatrixXd::Identity(3,3)*error_pose;

	return output_velocity;
}

void AttractorDS::DynCallback(
		adaptive_polishing::attractor_paramsConfig &config, uint32_t level) 
{


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

	Convergence_Rate_scale_ = config.ConvergenceSpeed;
	Velocity_limit_ = config.vel_trimming;
	Grad_desc_epsilon_ = config.grad_descent_epsilon; 
	Grad_desc_step_ = config.grad_descent_step;


	if (Convergence_Rate_scale_ < 0) {
		ROS_ERROR("RECONFIGURE: The scaling factor for convergence rate cannot be negative!");
	}

	if (Velocity_limit_ < 0) {
		ROS_ERROR("RECONFIGURE: The limit for velocity cannot be negative!");
	}
}

void AttractorDS::AdaptTrajectoryParameters(Eigen::Vector3d pose){

	//if outside of workspace adapt if force is applied to robot 
	if(real_pose_(Z) > WORKSPACE_UP_BOUND){

		if(rob_sensed_force_.norm() < FORCE_THRESHOLD){
			return;
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


	// get the error on the velocity
	Eigen::Vector3d error_vel = desired_velocity_ - real_vel_;

	double grad_J(0);

	Eigen::Vector3d err1;
	Eigen::Vector3d err2;
	double tmp(0);


	for(auto& param : parameters_){
		if(param.adapt)
		{
			// normalize the parameter
			tmp = param.val;
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

			tmp += (Grad_desc_epsilon_*grad_J);
			param.val = SCALE_BACK(tmp,param.min,param.max);

			//set boundaries
			param.val = MIN(param.val,param.max);
			param.val = MAX(param.val,param.min);
		}
	}
}


