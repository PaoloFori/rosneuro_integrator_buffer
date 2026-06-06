#include "rosneuro_integrator_buffer/Buffer.h"

namespace rosneuro {
    namespace integrator {

Buffer::Buffer(void) : p_nh_("~") {
    this->setName("Integrator Buffer");
}

Buffer::~Buffer(void) {
}

bool Buffer::configure(void) {

    // Bind dynamic reconfigure callback
    this->recfg_callback_type_ = boost::bind(&Buffer::on_request_reconfigure, this, _1, _2);
    this->recfg_srv_.setCallback(this->recfg_callback_type_);

    // Getting parameters from launcher file
    int increment, n_classes, buffer_size;

    this->p_nh_.param<int>("buffer_size", buffer_size, 64);
    this->p_nh_.param<int>("n_classes", n_classes, 2);
    this->p_nh_.param<int>("increment", increment, INCREMENT_SOFT);
    this->p_nh_.param<float>("k_gain", this->k_gain_, 2.5f);

    std::vector<float> init_val;
    this->p_nh_.param<std::vector<float>>("init_val", init_val, std::vector<float>(2, 1.0/n_classes));
    if(init_val.size() != n_classes) {
        ROS_ERROR("You must specify an initial value for each class. 'n_classes' is %d but 'init_val' has size %d",n_classes,(int)init_val.size());
        return false;
    }    

    std::vector<float> ths_rejection;
    this->p_nh_.param<std::vector<float>>("thresholds_rejection", ths_rejection, std::vector<float>(2, 1.0/n_classes));
    if(ths_rejection.size() != n_classes){
        ROS_ERROR("[%s] Parameter 'thresholds_rejection' must have 2 values (2-class problem)", this->name().c_str());
        return false;
    }

    this->setRejection(ths_rejection);
    this->setBufferSize(buffer_size);
    this->setIncrement(increment);
    this->setClasses(n_classes);
    this->setInitVal(init_val);

    this->reset();

    return true;
}

void Buffer::setRejection(std::vector<float> values) {
    bool valid_values = true;
    for(auto val : values){
        if(val < 0.5f || val > 1.0f){
            valid_values = false;
            ROS_ERROR("[%s] Rejection value is not legal (rejection=%f)", this->name().c_str(), val);
            break;
        }
    }
    if(valid_values){
        this->rejections_ = values;
    }
}

void Buffer:: setClasses(int value){
    this->n_classes = value;
}

void Buffer:: setIncrement(int value){
    this-> increment = value;
}

void Buffer:: setBufferSize(int value){
    this-> buffer_size = value;
}
    
void Buffer:: setInitVal(std::vector<float> init_val){
    this->init_val_=init_val;
}

Eigen::VectorXf Buffer::getData(void) {
    return this->data_;
}

Eigen::VectorXf Buffer::apply(const Eigen::VectorXf& input) {
    double increment;
    Eigen::Index maxIndex;
    if(input.size() != this->n_classes) {
        ROS_WARN("[%s] Input size (%ld) is not equal to declared input size (%d)", this->name().c_str(),input.size(),this->n_classes);
        return this->data_;
    }

    input.maxCoeff(&maxIndex);
    if(input(maxIndex) > this->rejections_.at(maxIndex)){
        if(this->increment == INCREMENT_HARD){
            increment = 1.0 / this->buffer_size;
        }
        else if(this->increment == INCREMENT_SOFT){
            double p_max = static_cast<double>(input(maxIndex));
            double velocity_factor = std::abs(p_max - 0.5) * 2.0 * static_cast<double>(this->k_gain_);
            if(velocity_factor > 1.0) velocity_factor = 1.0;
            double base_step = 1.0 / static_cast<double>(this->buffer_size);
            increment = base_step * velocity_factor;
        }
        else{
            ROS_WARN("Apparently Increment type (%d) is wrong.",this->increment);
            return this->data_;
        }

        //Increment Buffer (in double to avoid float32 accumulation drift)
        for (int i=0; i<this->n_classes;i++){
            if (i == (int)maxIndex)
                this->data_d_[i] += increment;
            else
                this->data_d_[i] -= increment;
        }
        // [0,1] clipping
        this->data_d_ = this->data_d_.cwiseMax(0.0).cwiseMin(1.0);
        this->data_ = this->data_d_.cast<float>();
    }

    return this->data_;
}

bool Buffer::reset(void) {
    this->data_d_ = Eigen::VectorXd::Constant(this->n_classes, 0.5);
    for (int i=0; i<this->n_classes;i++){
        this->data_d_[i] = static_cast<double>(this->init_val_.at(i));
    }
    this->data_ = this->data_d_.cast<float>();
    return true;
}

std::vector<float> Buffer::getInitPrecentual(void){
    return this->init_val_;
}

void Buffer::on_request_reconfigure(rosneuro_config_buffer &config, uint32_t level) {

    if( config.increment != this->increment) {
        this->setIncrement(config.increment);
    }

    if( config.buffer_size != this->buffer_size) {
        this->setBufferSize(config.buffer_size);
    }
}

}
}