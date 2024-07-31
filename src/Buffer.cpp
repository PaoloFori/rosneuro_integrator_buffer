#include "rosneuro_integrator_buffer/Buffer.h"

namespace rosneuro {
    namespace integrator {

Buffer::Buffer(void) : p_nh_("~") {
    this->setName("integrator_buffer");
}

Buffer::~Buffer(void) {
}

bool Buffer::configure(void) {

    int n_classes;
    int buffers_size;
    std::vector<int> classes;

    if(this->p_nh_.getParam("buffers_size", buffers_size) == false) {
        ROS_ERROR("Parameter 'buffers_size' is mandatory");
        return false;
    }
    if(this->p_nh_.getParam("classes", classes) == false) {
        ROS_ERROR("Parameter 'classes' is mandatory");
        return false;
    }
    n_classes = classes.size();

    std::vector<float> init_percentual;
    if(this->p_nh_.getParam("init_percentual", init_percentual) == false) {
        ROS_ERROR("Parameter 'init_val' is mandatory");
        return false;
    }
    if(init_percentual.size() != n_classes) {
        ROS_ERROR("Parameter 'init_percentual' must have the same size of 'classes'");
        return false;
    }else if(std::accumulate(init_percentual.begin(), init_percentual.end(), 0.0) != 1.0){
        ROS_ERROR("Parameter 'init_percentual' must sum to 1.0");
        return false;
    }

    // Bind dynamic reconfigure callback
    this->recfg_callback_type_ = boost::bind(&Buffer::on_request_reconfigure, this, _1, _2);
    this->recfg_srv_.setCallback(this->recfg_callback_type_);

    this->setBufferSize(buffers_size);
    this->setNClasses(n_classes);
    this->setClasses(classes);
    this->setInitPercentual(init_percentual);

    this->reset();

    this->first_ = false;
    return true;
}

void Buffer::setClasses(std::vector<int> value){
    this->classes_ = value;
}

void Buffer::setNClasses(int value){
    this->n_classes_ = value;
}

void Buffer::setBufferSize(int value){
    this-> buffers_size_ = value;
}
    
void Buffer::setInitPercentual(std::vector<float> init_percentual){
    this->init_percentual_ = init_percentual;
}

std::vector<float> Buffer::getInitPrecentual(void){
    return this->init_percentual_;
}

Eigen::VectorXf Buffer::apply(const Eigen::VectorXf& input) {
    Eigen::Index maxIndex;
    if (this->first_ == false){
        this->first_ = true;
        ROS_INFO("[integrator_buffer] first frame received");
    }
    if(input.size() != this->n_classes_) {
        ROS_ERROR("[%s] Input size (%ld) is not equal to declared input size (%d)", this->name().c_str(),input.size(),this->n_classes_); 
    }

	// take the max coefficient of the input vector
    input.maxCoeff(&maxIndex);
	if(this->index_ >= this->buffers_size_){
		this->index_ = 0;
	}
	this->buffer_(this->index_) = this->classes_[maxIndex];
	this->index_ = this->index_ + 1;

	// compute the percentage of the classes in the buffer
	Eigen::VectorXf prob = Eigen::VectorXf::Zero(this->n_classes_);
	for (int i = 0; i < this->buffers_size_; ++i) {
		for(int j = 0; j < this->n_classes_; ++j){
			if(this->buffer_(i) == this->classes_[j]){
				prob(j) = prob(j) + 1;
			}
		}
	}

	prob = prob / this->buffers_size_;

    return prob;
}

// TODO: keep the percentage of the classes asked in the init_percentual vector, may be the comment part
bool Buffer::reset(void) { 
    this->buffer_ = Eigen::VectorXf(this->buffers_size_);
    for (int i = 0; i < buffers_size_; ++i) {
        this->buffer_(i) = this->classes_.at(i % this->n_classes_);
    }

	this->index_ = 0;

    // compute the percentage of the classes in the buffer
    /*
    // Calculate the number of occurrences for each class
    std::vector<int> counts(this->n_classes_);
    for (int i = 0; i < this->n_classes_; ++i) {
        counts[i] = static_cast<int>(this->buffers_size_ * this->init_percentual_[i]);
    }

    // Adjust the last count to ensure the sum is exactly `size`
    int sum_counts = std::accumulate(counts.begin(), counts.end(), 0);
    if (sum_counts != this->buffers_size_) {
        counts.back() += (this->buffers_size_ - sum_counts);
    }

    std::vector<float> temp_vector;
    temp_vector.reserve(this->buffers_size_);

    // Fill the temp_vector with the calculated number of each class value
    for (int i = 0; i < this->n_classes_; ++i) {
        temp_vector.insert(temp_vector.end(), counts[i], this->classes_[i]);
    }

    // Shuffle the temp_vector
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(temp_vector.begin(), temp_vector.end(), g);

    // Transfer the shuffled values to the Eigen vector
    for (int i = 0; i < this->buffers_size_; ++i) {
        this->buffer_(i) = temp_vector[i];
    }
    std::cout << this->buffer_ << std::endl;
    */

    return true;
}

Eigen::VectorXf Buffer::uniform_vector(int size, float value) {
    return Eigen::VectorXf::Constant(size, value);
}

void Buffer::on_request_reconfigure(rosneuro_config_buffer &config, uint32_t level) {


    if(config.buffer_size != this->buffers_size_){
        this->setBufferSize(config.buffer_size);
    }
}

}
}