#ifndef __ADAPTIVE_POLISHING_V2_H__
#define __ADAPTIVE_POLISHING_V2_H__


#include "MotionGenerator.h"

#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Int32.h"
#include <adaptive_polishing/cycleParam_msg.h>

#include "MathLib.h"
#include <vector>

#include <dynamic_reconfigure/server.h>
#include <adaptive_polishing/polishing_paramsConfig.h>

class AdaptivePolishing : public MotionGenerator {

private:

	ros::Subscriber sub_real_pose_;
	ros::Subscriber sub_real_vel_;

	struct Parameter{
		double val;
		bool adapt;
		double min;
		double max;
		double prev_grad;
		double confidence;
	};

	//publisher and msg to publish the cycle target when it is adapting
	geometry_msgs::Pose msg_cycle_target_;
	ros::Publisher pub_cycle_target_;
	ros::Publisher pub_active_node_;
	std_msgs::Int32 msg_active_node;

	ros::Publisher pub_cycle_param_;
	adaptive_polishing::cycleParam_msg msg_cycleParam_;

	// Motion detail
	std::vector<Parameter> parameters_;
	double Cycle_radius_;
	double Cycle_speed_;
	double Cycle_speed_offset_;
	double Convergence_Rate_;
	double Convergence_Rate_scale_;


	// Adaptation parameters
	double Grad_desc_step_; //step for numerical derivation
	double Grad_desc_epsilon_; // epsilon for state adaptation
	int func_used_ = 1;
	std::vector<double> confidence_;
	double p_ = 0.95;
	std::vector<double> prev_grad_;
	std::vector<Eigen::Vector3d> previousPoses;
	std::vector<Eigen::Vector3d> previousVels;
	int adaptBufferCounter_ = 0;
	bool adaptBufferReady_ = false;
	int num_points_ = 10;
	int real_num_points_ = num_points_;
	int adaptTimeWindow_ = 1000;//1 second
	Eigen::Vector3d average_speed_;
	Eigen::Vector3d average_pose_;
	int average_speed_counter_;
	int average_pose_counter_;
	ros::Timer adaptTimer_;



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
			std::vector<double> parameters,
			std::vector<double> min_parameters,
			std::vector<double> max_parameters,
			std::vector<double> adaptable_parameters,
			double RotationSpeed,
			double ConvergenceRate
	);

private:

	void SaveRealPosition(const geometry_msgs::Pose::ConstPtr& msg);

	void SaveRealVelocity(const geometry_msgs::Twist::ConstPtr& msg);

	void DynCallback(adaptive_polishing::polishing_paramsConfig &config, 
			uint32_t level);

	virtual Eigen::Vector3d GetVelocityFromPose(Eigen::Vector3d pose) override;

	virtual void PublishOnTimer(const ros::TimerEvent&) override;

	void AdaptTrajectoryParameters(Eigen::Vector3d pose) override;

	void adaptBufferFillingcallback(const ros::TimerEvent&);



};

#endif //__ADAPTIVE_POLISHING_V2_H__