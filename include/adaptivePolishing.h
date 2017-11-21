#ifndef __ADAPTIVE_POLISHING_V2_H__
#define __ADAPTIVE_POLISHING_V2_H__


#include "MotionGenerator.h"

#include "geometry_msgs/Pose2D.h"

#include "MathLib.h"
#include <vector>

#include <dynamic_reconfigure/server.h>
#include <adaptive_polishing/polishing_paramsConfig.h>

class AdaptivePolishing : public MotionGenerator {

private:

	struct Parameter{
		double val;
		bool adapt;
		double scale;
	};
	// Motion detail
	Eigen::Vector3d Cycle_Target_;

	//publisher and msg to publish the cycle target when it is adapting
	geometry_msgs::Pose msg_cycle_target_;
	ros::Publisher pub_cycle_target_;


	double Cycle_radius_;
	double Cycle_speed_;
	double Cycle_speed_offset_;
	double Convergence_Rate_;
	double Convergence_Rate_scale_;

	// Adaptation parameters
	double Grad_desc_step_; //step for numerical derivation
	double Grad_desc_epsilon_; // epsilon for state adaptation
	std::vector<Parameter> parameters_;


	//dynamic reconfig setting
	dynamic_reconfigure::Server<adaptive_polishing::polishing_paramsConfig> dyn_rec_srv_;
	dynamic_reconfigure::Server<adaptive_polishing::polishing_paramsConfig>::CallbackType dyn_rec_f_;


public:
	AdaptivePolishing(ros::NodeHandle &n,
			double frequency,
			std::string input_rob_pos_topic_name,
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
	);

private:

	void DynCallback(adaptive_polishing::polishing_paramsConfig &config, 
			uint32_t level);

	virtual Eigen::Vector3d GetVelocityFromPose(Eigen::Vector3d pose) override;

	void AdaptTrajectoryParameters(Eigen::Vector3d pose) override;

	Eigen::Vector2d ComputeGradient(Eigen::Vector3d error_vel,
			Eigen::Vector3d pose);


};


#endif //__ADAPTIVE_POLISHING_V2_H__