#ifndef __ADAPTIVE_POLISHING_V2_H__
#define __ADAPTIVE_POLISHING_V2_H__


#include "MotionGenerator.h"

#include "MathLib.h"
#include <vector>

#include <dynamic_reconfigure/server.h>
#include <adaptive_polishing/polishing_paramsConfig.h>

class AdaptivePolishing : public MotionGenerator {

private:

	// Motion detail
	std::vector<double> Cycle_Target_;

	double Cycle_radius_;
	double Cycle_radius_scale_;

	double Cycle_speed_;
	double Cycle_speed_offset_;

	double Convergence_Rate_;
	double Convergence_Rate_scale_;

	//dynamic reconfig setting
	dynamic_reconfigure::Server<adaptive_polishing::polishing_paramsConfig> dyn_rec_srv_;
	dynamic_reconfigure::Server<adaptive_polishing::polishing_paramsConfig>::CallbackType dyn_rec_f_;


public:
	AdaptivePolishing(ros::NodeHandle &n,
		double frequency,
		std::string input_rob_pos_topic_name,
		std::string output_vel_topic_name,
		std::string output_filtered_vel_topic_name,
		std::string input_rob_vel_topic_name,
		std::string input_rob_force_ee_topic_name,
		std::vector<double> CenterRotation,
		double radius,
		double RotationSpeed,
		double ConvergenceRate
        );

private:

	void DynCallback(adaptive_polishing::polishing_paramsConfig &config, uint32_t level);

	virtual MathLib::Vector GetVelocityFromPose(MathLib::Vector pose) override;


};


#endif //__ADAPTIVE_POLISHING_V2_H__