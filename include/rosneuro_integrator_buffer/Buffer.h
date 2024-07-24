#ifndef ROSNEURO_INTEGRATOR_BUFFER_H_
#define ROSNEURO_INTEGRATOR_BUFFER_H_

#include <Eigen/Dense>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <dynamic_reconfigure/server.h>
#include "rosneuro_integrator/GenericIntegrator.h"
#include "rosneuro_integrator_buffer/BufferConfig.h"

#define INCREMENT_HARD 0
#define INCREMENT_SOFT 1

namespace rosneuro {
	namespace integrator {

using rosneuro_config_buffer = rosneuro_integrator_buffer::BufferConfig;
using dyncfg_buffer         = dynamic_reconfigure::Server<rosneuro_config_buffer>;

class Buffer : public GenericIntegrator {

	public:
		Buffer(void);
		~Buffer(void);

		bool configure(void);
		Eigen::VectorXf apply(const Eigen::VectorXf& input);
		bool reset(void);
		void setClasses(int value);
		void setIncrement(int value);
		void setBuffersSize(std::vector<int> value);
		void setInitVal(std::vector<float> init_val);
		void setBufferSize(int value, int idx_buffer);
		std::vector<float> getInitVal(void);

	private:
		Eigen::VectorXf uniform_vector(int size, float value);
		void on_request_reconfigure(rosneuro_config_buffer &config, uint32_t level);

	private:
		ros::NodeHandle p_nh_;
		Eigen::VectorXf data_;
		int increment_;
		int n_classes_;
		std::vector<int> buffers_size_;
		std::vector<float> init_val_;
		dyncfg_buffer recfg_srv_;
  		dyncfg_buffer::CallbackType recfg_callback_type_;
		bool first_;
};

PLUGINLIB_EXPORT_CLASS(rosneuro::integrator::Buffer, rosneuro::integrator::GenericIntegrator)

	}
}

#endif
