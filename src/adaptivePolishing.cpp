
#include "adaptivePolishing.h"

#define POWER_THRESHOLD 0.4
#define FORCE_THRESHOLD 10.0
#define WORKSPACE_UP_BOUND 10
#define NUM_PARAMS 6

#define SUB_BUFFER_SIZE 1000

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
	output_filtered_vel_topic_name, true),
	Cycle_speed_(RotationSpeed),Cycle_speed_offset_(0),
	Convergence_Rate_(ConvergenceRate),Convergence_Rate_scale_(1),
	Cycle_radius_(1),average_speed_counter_(0),average_pose_counter_(0),
	average_pose_(Eigen::Vector3d::Zero()),average_speed_(Eigen::Vector3d::Zero())
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

	previousPoses.resize(num_points_);
	previousVels.resize(num_points_);

	double duration = (double)adaptTimeWindow_/num_points_/1000.0;
	adaptTimer_ = nh_.createTimer(ros::Duration(duration), 
			&AdaptivePolishing::adaptBufferFillingcallback,this);

	ROS_INFO_STREAM("AP.CPP: Adaptive polishing node is created at: " <<
			nh_.getNamespace() << " with freq: " << frequency << "Hz");

	pub_cycle_target_ = nh_.advertise<geometry_msgs::Pose>(
			"DS/adaptivePolishing/cycle_target", 1000, 1);

	pub_cycle_param_ = nh_.advertise<adaptive_polishing::cycleParam_msg>(
			"DS/adaptivePolishing/cycle_param", 1000, 1);
 
	sub_real_pose_ = nh_.subscribe(input_rob_pose_topic_name, SUB_BUFFER_SIZE,
			&AdaptivePolishing::SaveRealPosition, this,
			ros::TransportHints().reliable().tcpNoDelay());

	sub_real_vel_ = nh_.subscribe(input_rob_vel_topic_name, SUB_BUFFER_SIZE,
			&AdaptivePolishing::SaveRealVelocity, this,
			ros::TransportHints().reliable().tcpNoDelay());

	dyn_rec_f_ = boost::bind(&AdaptivePolishing::DynCallback, this, _1, _2);
	dyn_rec_srv_.setCallback(dyn_rec_f_);

}


Eigen::Vector3d AdaptivePolishing::GetVelocityFromPose(Eigen::Vector3d pose)
{
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

	if(num_points_ != config.num_points || 
			adaptTimeWindow_ != config.adaptTimeWindow){
		adaptBufferReady_ = false;

		num_points_ = config.num_points; 
		adaptTimeWindow_ = config.adaptTimeWindow;

		real_num_points_ = MIN(round((double)adaptTimeWindow_/4.0),num_points_);

		double duration = (double)adaptTimeWindow_/real_num_points_/1000.0;
		adaptTimer_.setPeriod(ros::Duration(duration));

		adaptBufferCounter_ = 0;
		previousPoses.resize(real_num_points_);
		previousVels.resize(real_num_points_);
	}

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

	// //if outside of workspace adapt if force is applied to robot 

	// if(real_pose_(Z) > WORKSPACE_UP_BOUND){

	// 	if(rob_sensed_force_.norm() < FORCE_THRESHOLD){
	// 		return;
	// 	}
	// // if inside workspace adapt if there is a power exchange
	// }else{
	// double power = rob_sensed_force_.dot(real_vel_);
	// 	if(power < POWER_THRESHOLD){
	// 		return;
	// 	}
	// }

	if(rob_sensed_force_.norm() < FORCE_THRESHOLD){
		return;
	}

	if(Grad_desc_step_ == 0){
		return;
	}

	if(!adaptBufferReady_)
	{
		return;
	}

	// get the error on the velocity for all the saved poses with current parameters
	std::vector<Eigen::Vector3d> error_vel(real_num_points_) ;
	for(int i=0;i<previousPoses.size();i++){
		error_vel[i] = GetVelocityFromPose(previousPoses[i]) - previousVels[i];
	}

	//double cost_function_J = 1/2 * error_vel.squaredNorm();

	double grad_J(0);

	std::vector<Eigen::Vector3d> err1(real_num_points_);
	std::vector<Eigen::Vector3d> err2(real_num_points_);
	double tmp(0);

	double dx,dy,grad1J,grad2J;

	for(auto& param : parameters_){
		grad_J = 0;
		if(param.adapt)
		{
			// normalize the parameter
			tmp = param.val;
			tmp = SCALE(tmp,param.min,param.max);

			//compute backward derivative
			tmp -= Grad_desc_step_;
			param.val = SCALE_BACK(tmp,param.min,param.max);
			for(int i=0;i<previousPoses.size();i++)
				err1[i] = GetVelocityFromPose(previousPoses[i]);
			tmp += Grad_desc_step_;

			//compute forward derivative
			tmp += Grad_desc_step_;
			param.val = SCALE_BACK(tmp,param.min,param.max);
			for(int i=0;i<previousPoses.size();i++)
				err2[i] = GetVelocityFromPose(previousPoses[i]);
			tmp -= Grad_desc_step_;

			//compute gradient
			for(int i=0;i<previousPoses.size();i++)
				grad_J += error_vel[i].dot((err1[i]-err2[i])/(2*Grad_desc_step_));

			// // uncomment the following lines in order to filter the gradient 
			// param.confidence = p_*param.confidence + (1-p_)*pow(grad_J - param.prev_grad,2);
			// param.prev_grad = grad_J;
			// param.confidence = MIN(param.confidence,0.01);
			// //modify the concerned parameter
			// tmp += (Grad_desc_epsilon_*grad_J)/param.confidence;

			tmp += (Grad_desc_epsilon_*grad_J);
			param.val = SCALE_BACK(tmp,param.min,param.max);

			//set boundaries
			param.val = MIN(param.val,param.max);
			param.val = MAX(param.val,param.min);
		}
	}
}


void AdaptivePolishing::adaptBufferFillingcallback(const ros::TimerEvent&)
{
	if(gotFirstPosition_){

		//uncomment in order to filter the save positions and speed
		// previousPoses.at(adaptBufferCounter_) = average_pose_ - target_offset_;
		// previousVels.at(adaptBufferCounter_) = average_speed_ - target_offset_;
		// average_pose_counter_ = 0;
		// average_speed_counter_ = 0;

		//comment if previous lines are not commented
		previousPoses.at(adaptBufferCounter_) = real_pose_ - target_offset_;
		previousVels.at(adaptBufferCounter_) = real_vel_ - target_offset_;

		adaptBufferCounter_++;
		if(adaptBufferCounter_ == real_num_points_){
			adaptBufferCounter_ = 0;
			adaptBufferReady_ = true;
		}
	}
}


void AdaptivePolishing::SaveRealPosition(
		const geometry_msgs::Pose::ConstPtr& msg) 
{
	if(gotFirstPosition_)
	{
		
		average_pose_(X) = (average_pose_(X)*average_pose_counter_ + msg->position.x)/(average_pose_counter_+1);
		average_pose_(Y) = (average_pose_(Y)*average_pose_counter_ + msg->position.x)/(average_pose_counter_+1);
		average_pose_(Z) = (average_pose_(Z)*average_pose_counter_ + msg->position.x)/(average_pose_counter_+1);

		average_pose_counter_++;
	}
}


void AdaptivePolishing::SaveRealVelocity(
		const geometry_msgs::Twist::ConstPtr& msg)
{
	average_speed_(X) = (average_speed_(X)*average_speed_counter_ + msg->linear.x)/(average_speed_counter_+1);
	average_speed_(Y) = (average_speed_(Y)*average_speed_counter_ + msg->linear.y)/(average_speed_counter_+1);
	average_speed_(Z) = (average_speed_(Z)*average_speed_counter_ + msg->linear.z)/(average_speed_counter_+1);

	average_speed_counter_++;
}


void AdaptivePolishing::PublishOnTimer(const ros::TimerEvent&){

	msg_cycle_target_.position.x = parameters_[OFFSET_X].val;
	msg_cycle_target_.position.y = parameters_[OFFSET_Y].val;
	msg_cycle_target_.position.z = parameters_[OFFSET_Z].val;
	msg_cycle_target_.orientation.x = 0;
	msg_cycle_target_.orientation.y = 0;
	msg_cycle_target_.orientation.z = 0;
	msg_cycle_target_.orientation.w = 0;
	pub_cycle_target_.publish(msg_cycle_target_);


	msg_cycleParam_.cycle_target_x = parameters_[OFFSET_X].val;
	msg_cycleParam_.cycle_target_y = parameters_[OFFSET_Y].val;
	msg_cycleParam_.semi_axis_x = parameters_[SEMI_AXIS_A].val;
	msg_cycleParam_.semi_axis_y = parameters_[SEMI_AXIS_B].val;
	pub_cycle_param_.publish(msg_cycleParam_);

}