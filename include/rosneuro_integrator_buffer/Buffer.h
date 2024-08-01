#ifndef ROSNEURO_INTEGRATOR_BUFFER_H_
#define ROSNEURO_INTEGRATOR_BUFFER_H_

#include <Eigen/Dense>
#include <ros/ros.h>
#include <numeric>
#include <random>
#include <pluginlib/class_list_macros.h>
#include <dynamic_reconfigure/server.h>
#include "rosneuro_integrator/GenericIntegrator.h"
#include "rosneuro_integrator_buffer/BufferConfig.h"

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
        void setNClasses(int value);
        void setClasses(std::vector<int> value);
        void setThresholds(std::vector<float> value);
        void setBufferSize(int value);
        void setInitPercentual(std::vector<float> init_percentual);
        std::vector<float> getInitPrecentual(void);

    private:
        Eigen::VectorXf uniform_vector(int size, float value);
        void on_request_reconfigure(rosneuro_config_buffer &config, uint32_t level);

    private:
        ros::NodeHandle p_nh_;
		Eigen::VectorXf buffer_;
        int n_classes_;
        int buffers_size_;
        float init_val_;
        dyncfg_buffer recfg_srv_;
        dyncfg_buffer::CallbackType recfg_callback_type_;
        int index_;
        std::vector<int> classes_;
        std::vector<float> init_percentual_;
};

PLUGINLIB_EXPORT_CLASS(rosneuro::integrator::Buffer, rosneuro::integrator::GenericIntegrator)

    }
}

#endif
