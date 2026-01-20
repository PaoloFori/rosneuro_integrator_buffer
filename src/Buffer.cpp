#include "rosneuro_integrator_buffer/Buffer.h"

namespace rosneuro {
    namespace integrator {

Buffer::Buffer(void) : p_nh_("~") {
    this->setName("buffer");
}

Buffer::~Buffer(void) {
}

bool Buffer::configure(void) {

    int increment, n_classes, buffer_size;

    this->p_nh_.param<int>("buffer_size", buffer_size, 64);
    this->p_nh_.param<int>("n_classes", n_classes, 2);
    this->p_nh_.param<int>("increment", increment, INCREMENT_SOFT);
    this->p_nh_.param<float>("k_gain", this->k_gain_, 2.5f);

    std::vector<float> init_val;
    // Getting init_val
    if(this->p_nh_.getParam("init_val", init_val) == false) {
        ROS_ERROR("Parameter 'init_val' is mandatory");
        return false;
    } 
    else if(init_val.size() != n_classes) {
        ROS_ERROR("You must specify an initial value for each class. 'n_classes' is %d but 'init_val' has size %d",n_classes,(int)init_val.size());
        return false;
    }    

    std::vector<float> ths_rejection;
    this->p_nh_.param<std::vector<float>>("thresholds_rejection", ths_rejection, std::vector<float>(2, 0.5f));
    if(ths_rejection.size() != n_classes){
        ROS_ERROR("[%s] Parameter 'thresholds_rejection' must have 2 values (2-class problem)", this->name().c_str());
        return false;
    }

    // Bind dynamic reconfigure callback
    this->recfg_callback_type_ = boost::bind(&Buffer::on_request_reconfigure, this, _1, _2);
    this->recfg_srv_.setCallback(this->recfg_callback_type_);

    this->setRejection(ths_rejection);
    this->setbuffersize(buffer_size);
    this->setincrement(increment);
    this->setclasses(n_classes);
    this->setinitval(init_val);

    this->reset();

    return true;
}

void Buffer::setRejection(std::vector<float> values) {
    bool valid_values = true;
    for(auto val : values){
        if(val < 0.5f | val > 1.0f){
            valid_values = false;
            ROS_ERROR("[%s] Rejection value is not legal (rejection=%f)", this->name().c_str(), val);
            break;
        }
    }
    if(valid_values){
        this->rejections_ = values;
    }
}

void Buffer:: setclasses(int value){
    this->n_classes = value;
}

void Buffer:: setincrement(int value){
    this-> increment = value;
}

void Buffer:: setbuffersize(int value){
    this-> buffer_size = value;
    std::cout << "Buffer size set to " << this-> buffer_size << std::endl;
}
    
void Buffer:: setinitval(std::vector<float> init_val){
    this->init_val_=init_val;
}

Eigen::VectorXf Buffer::getData(void) {
    return this->data_;
}

Eigen::VectorXf Buffer::apply(const Eigen::VectorXf& input) {
    float increment;
    Eigen::Index maxIndex;
    if(input.size() != this->n_classes) {
        ROS_WARN("[%s] Input size (%ld) is not equal to declared input size (%d)", this->name().c_str(),input.size(),this->n_classes); 
        return this->data_;
    }

    input.maxCoeff(&maxIndex);
    if(input(maxIndex) > this->rejections_.at(maxIndex)){
        if(this->increment == INCREMENT_HARD){
            increment = 1.0F/this-> buffer_size;
        }
        else if(this->increment == INCREMENT_SOFT){
            float p_max = input(maxIndex);
            float velocity_factor = std::abs(p_max - 0.5f) * 2.0f * this->k_gain_;
            if(velocity_factor > 1.0f) velocity_factor = 1.0f;
            float base_step = 1.0f / (float)this->buffer_size;
            increment = base_step * velocity_factor;

            // increment = input[maxIndex]/this-> buffer_size;
        }
        else{
            ROS_WARN("Apparently Increment type (%d) is wrong.",this->increment); 
            return this->data_;
        }

        //Increment Buffer
        for (int i=0; i<this->n_classes;i++){

            if (i == (int)maxIndex) 
                this->data_[i] += increment;  
            else 
                this->data_[i] -= increment;  
        }
        // [0,1] clipping
        this->data_ = this->data_.cwiseMax(0.0).cwiseMin(1.0);
    }

    return this->data_;
}

bool Buffer::reset(void) {
    this->data_ = this->uniform_vector(n_classes,0.5);
    for (int i=0; i<this->n_classes;i++){
        this->data_[i] = this->init_val_.at(i);
    }
    return true;
}

Eigen::VectorXf Buffer::uniform_vector(int size, float value) {
    return Eigen::VectorXf::Constant(size, value);
}

std::vector<float> Buffer::getInitPrecentual(void){
    return this->init_val_;
}

void Buffer::on_request_reconfigure(rosneuro_config_buffer &config, uint32_t level) {

    if( config.increment != this->increment) {
        this->setincrement(config.increment);
    }

    if( config.buffer_size != this->buffer_size) {
        this->setbuffersize(config.buffer_size);
    }
}

}
}