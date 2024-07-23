#include "rosneuro_integrator_buffer/Buffer.h"

namespace rosneuro {
	namespace integrator {

Buffer::Buffer(void) : p_nh_("~") {
	this->setName("buffer");
}

Buffer::~Buffer(void) {
}

bool Buffer::configure(void) {

	int increment, n_classes;
	std::vector<int> buffers_size;

	if(this->p_nh_.getParam("buffers_size", buffers_size) == false) {
        ROS_ERROR("Parameter 'thresholds' is mandatory");
        return false;
	}
	n_classes = buffers_size.size();
	
	this->p_nh_.param<int>("increment", increment, INCREMENT_HARD);

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

	// Bind dynamic reconfigure callback
	this->recfg_callback_type_ = boost::bind(&Buffer::on_request_reconfigure, this, _1, _2);
	this->recfg_srv_.setCallback(this->recfg_callback_type_);

	this->setBuffersSize(buffers_size);
	this->setIncrement(increment);
	this->setClasses(n_classes);
	this->setInitVal(init_val);

	this->reset();

	this->first_ = false;
	return true;
}

void Buffer::setClasses(int value){
	this->n_classes_ = value;
}

void Buffer::setIncrement(int value){
	this-> increment_ = value;
}

void Buffer::setBuffersSize(std::vector<int> value){
	this-> buffers_size_ = value;
}

void Buffer::setBufferSize(int value, int idx_buffer){
	this-> buffers_size_.at(idx_buffer) = value;
}
	
void Buffer::setInitVal(std::vector<float> init_val){
	this->init_val_ = init_val;
}

Eigen::VectorXf Buffer::apply(const Eigen::VectorXf& input) {
	float increment_tmp;
	Eigen::Index maxIndex;
	if (this->first_ == false){
		this->first_ =true;
		ROS_INFO("[buffer] first frame received");
	}
	if(input.size() != this->n_classes_) {
		ROS_WARN("[%s] Input size (%ld) is not equal to declared input size (%d)", this->name().c_str(),input.size(),this->n_classes_); 
		return this->data_;
	}

	input.maxCoeff(&maxIndex);
	if(this->increment_ == INCREMENT_HARD){
		increment_tmp = 1.0F; /// this->buffers_size.at(maxIndex);
	}else if(this->increment_ == INCREMENT_SOFT){
		increment_tmp = input[maxIndex]; /// this->buffers_size.at(maxIndex);
	}

	//Increment Buffer
	for(int i = 0; i < this->n_classes_; i++){
		if (i == (int)maxIndex){
			this->data_[i] += (increment_tmp/this->buffers_size_.at(i));
		}else{
			this->data_[i] -= (increment_tmp/this->buffers_size_.at(i));
		}
	}
	// [0,1] clipping
	this->data_ = this->data_.cwiseMax(0.0).cwiseMin(1.0);

	return this->data_;
}

bool Buffer::reset(void) {
	this->data_ = this->uniform_vector(n_classes_,0);
	for (int i = 0; i < this->n_classes_; i++){
		this->data_[i] = this->init_val_.at(i);
	}
	return true;
}

Eigen::VectorXf Buffer::uniform_vector(int size, float value) {
	return Eigen::VectorXf::Constant(size, value);
}

void Buffer::on_request_reconfigure(rosneuro_config_buffer &config, uint32_t level) {

	if(config.increment != this->increment_){
		this->setIncrement(config.increment);
	}

	if(config.buffer_size_0 != this->buffers_size_.at(0)){
		this->setBufferSize(config.buffer_size_0, 0);
	}

	if(config.buffer_size_1 != this->buffers_size_.at(1)){
		this->setBufferSize(config.buffer_size_1, 1);
	}
}

}
}