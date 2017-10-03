#ifndef __ADAPTIVE_POLISHING_H__
#define __ADAPTIVE_POLISHING_H__

#include "ros/ros.h"
#include <vector>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
// #include "geometry_msgs/PointStamped.h"
// #include "nav_msgs/Path.h"



#include "MathLib.h"
// #include "GMRDynamics.h"
#include "CDDynamics.h"

#include <mutex>


#include <dynamic_reconfigure/server.h>
#include <adaptive_polishing/polishing_paramsConfig.h>

class Adaptive_polishing {


private:

	// ROS variables
	ros::NodeHandle nh_;
	ros::Rate loop_rate_;

	std::string input_topic_name_;
	std::string output_topic_name_;
	std::string output_filtered_topic_name_;


	ros::Subscriber sub_real_pose_;
	ros::Publisher pub_desired_twist_;
	ros::Publisher pub_desired_twist_filtered_;


	double dt_;



	// DS variables
	// std::unique_ptr<GMRDynamics> SED_GMM_;

	// int K_gmm_;
	// int dim_;
	// std::vector<double> Priors_;
	// std::vector<double> Mu_;
	// std::vector<double> Sigma_;
	// std::vector<double> attractor_;

	// double max_desired_vel_;

	// Filter variables: it takes x and use Wn as gains and dx ddx as limits
	std::unique_ptr<CDDynamics> filter_;
	double filter_Wn_;
	MathLib::Vector filter_dxLim_;
	MathLib::Vector filter_ddxLim_;





	// ros::Publisher pub_target_;
	// ros::Publisher pub_DesiredPath_;



	// geometry_msgs::Pose msg_real_pose_;
	geometry_msgs::TwistStamped msg_desired_velocity_;
	geometry_msgs::TwistStamped msg_desired_velocity_filtered_;

	// nav_msgs::Path msg_DesiredPath_;
	// int MAX_FRAME = 200;



	//dynamic reconfig settig
	dynamic_reconfigure::Server<adaptive_polishing::polishing_paramsConfig> dyn_rec_srv_;
	dynamic_reconfigure::Server<adaptive_polishing::polishing_paramsConfig>::CallbackType dyn_rec_f_;


	// Class variables
	std::mutex mutex_;

	MathLib::Vector real_pose_;
	MathLib::Vector target_pose_;
	MathLib::Vector target_offset_;


	// Motion detail
	std::vector<double> Cycle_Target_;

	double Cycle_radius_;
	double Cycle_radius_scale_;

	double Cycle_speed_;
	double Cycle_speed_offset_;

	double Convergence_Rate_;
	double Convergence_Rate_scale_;

	double Velocity_limit_;


	MathLib::Vector desired_velocity_;
	MathLib::Vector desired_velocity_filtered_;

	// double scaling_factor_;
	// double ds_vel_limit_;



public:
	Adaptive_polishing(ros::NodeHandle &n,
		double frequency,
		std::string input_topic_name,
		std::string output_topic_name,
		std::string output_filtered_topic_name,
		std::vector<double> CenterRotation,
		double radius,
		double RotationSpeed,
		double ConvergenceRate
        );

	bool Init();

	void Run();

private:

	bool InitializeROS();


	// bool InitializeDS();

	void UpdateRealPosition(const geometry_msgs::Pose::ConstPtr& msg);

	void ComputeDesiredVelocity();

	void PublishDesiredVelocity();

	// void PublishFuturePath();


	void DynCallback(adaptive_polishing::polishing_paramsConfig &config, uint32_t level);

};


#endif //__ADAPTIVE_POLISHING_H__