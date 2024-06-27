#include "rosneuro_integrator_buffer/Buffer.h"

namespace rosneuro {
	namespace integrator {

Buffer::Buffer(void) : p_nh_("~") {
	this->setname("buffer");
}

Buffer::~Buffer(void) {
}

bool Buffer::configure(void) {

	int increment, n_classes, buffer_size;

	this->p_nh_.param<int>("buffer_size", buffer_size, 48);
	this->p_nh_.param<int>("n_classes", n_classes, 2);
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
	//this->p_nh_.param<float>("init_val", init_val, 1.0f/n_classes);

	this->setbuffersize(buffer_size);
	this->setincrement(increment);
	this->setclasses(n_classes);
	this->setinitval(init_val);

	this->reset();

	// Bind dynamic reconfigure callback
	this->recfg_callback_type_ = boost::bind(&Buffer::on_request_reconfigure, this, _1, _2);
	this->recfg_srv_.setCallback(this->recfg_callback_type_);

	this->first = false;
	return true;
}

void Buffer:: setclasses(int value){
	this->n_classes = value;
	}

void Buffer:: setincrement(int value){
	this-> increment = value;
	}

void Buffer:: setbuffersize(int value){
	this-> buffer_size = value;
	}
	
void Buffer:: setinitval(std::vector<float> init_val){
	this->init_val=init_val;
}

Eigen::VectorXf Buffer::apply(const Eigen::VectorXf& input) {
	float increment;
	Eigen::Index maxIndex;
	if (this->first == false){
		this->first =true;
		ROS_INFO("[buffer] first frame received");
	}
	if(input.size() != this->n_classes) {
		ROS_WARN("[%s] Input size (%ld) is not equal to declared input size (%d)", this->name().c_str(),input.size(),this->n_classes); 
		return this->data_;
	}

	input.maxCoeff(&maxIndex);
	if(this->increment == INCREMENT_HARD){
		increment = 1.0F/this-> buffer_size;
	}
	else if(this->increment == INCREMENT_SOFT){
		increment = input[maxIndex]/this-> buffer_size;
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

	return this->data_;
}

bool Buffer::reset(void) {
	this->data_ = this->uniform_vector(n_classes,0);
	for (int i=0; i<this->n_classes;i++){
		this->data_[i] = this->init_val.at(i);
	}
	return true;
}

Eigen::VectorXf Buffer::uniform_vector(int size, float value) {
	return Eigen::VectorXf::Constant(size, value);
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
