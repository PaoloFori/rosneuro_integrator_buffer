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
        void setclasses(int value);
        void setincrement(int value);
        void setbuffersize(int value);
        void setinitval(std::vector<float> init_val);
        void setRejection(std::vector<float> values);
        Eigen::VectorXf getData(void);
        std::vector<float> getInitPrecentual(void);

    private:
        Eigen::VectorXf uniform_vector(int size, float value);
        void on_request_reconfigure(rosneuro_config_buffer &config, uint32_t level);

    private:
        ros::NodeHandle p_nh_;
        Eigen::VectorXf data_;
        int increment;
        int n_classes;
        int buffer_size;
        std::vector<float> init_val_;
        std::vector<float> rejections_;
        dyncfg_buffer recfg_srv_;
        dyncfg_buffer::CallbackType recfg_callback_type_;
};

PLUGINLIB_EXPORT_CLASS(rosneuro::integrator::Buffer, rosneuro::integrator::GenericIntegrator)

    }
}

#endif