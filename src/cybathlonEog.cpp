#ifndef CYBATHLON_EOG_CPP
#define CYBATHLON_EOG_CPP

#include "cybathlon_eog/cybathlonEog.hpp"

namespace cybathlon {

EogBci::EogBci(void): p_nh_("~") {

	this->eog_channels_ = EOG_CHANNELS;
	
	this->sub_topic_data_  =  "/neurodata_filtered";
	this->pub_topic_data_  =  "/events/bus";

}

EogBci::~EogBci(void) {
}

bool EogBci::configure(void) {
	
    // Take parameters
	ros::param::param("~buffer_size", (int&) this->buffer_size_, 512);
	ros::param::param("~n_channels", (int&) this->n_channels_, 16);
	ros::param::param("~sampling_freq", (int&) this->sampling_freq_, 512);
	ros::param::param("~n_samples", (int&) this->n_samples_, 32);

	this->framerate_ = 1000.0f*this->n_samples_/this->sampling_freq_;
	ros::param::param("~eog_threshold", (double&) this->eog_threshold_, 30.0);
    ros::param::param("~eog_left_channel", (int&) this->eog_lchannel_, 12); 
    ros::param::param("~eog_right_channel", (int&) this->eog_rchannel_, 16);   


	// Setup EOG channels to channel vector indeces
	this->chleft_ = this->eog_lchannel_ - 1; // Gloria we can avoid to define two variables
	this->chright_ = this->eog_rchannel_ - 1; // Gloria we can avoid to define two variables
	
	
	// Setup temporary data matrices 
	this->dmap_ = Eigen::MatrixXf::Zero(this->n_channels_, this->n_samples_);
	this->dfet_ = Eigen::MatrixXd::Zero(this->n_samples_, this->eog_channels_);
	this->heog_     = Eigen::VectorXd::Zero(this->n_samples_);
	this->veog_     = Eigen::VectorXd::Zero(this->n_samples_);
	this->hvalue_ = Eigen::VectorXd::Zero(this->buffer_size_);
	this->vvalue_ = Eigen::VectorXd::Zero(this->buffer_size_);

	// Set subscriber and publisher
	this->sub_data_ = this->p_nh_.subscribe(this->sub_topic_data_, 1000, &EogBci::on_received_data, this);
	this->pub_data_ = this->p_nh_.advertise<rosneuro_msgs::NeuroEvent>(this->pub_topic_data_, 1); 
	this->new_neuro_frame_= false;

	// Dynamic reconfiguration server
	this->reconfig_function_ = boost::bind(&EogBci::on_reconfigure_callback, this, _1, _2);
	this->reconfig_server_.setCallback(this->reconfig_function_);
	std::cout << "Eog reconfigure set up" << std::endl;

	return true;

}

void EogBci::SetThreshold(double value) {
	ROS_INFO("Eog Threshold value set at %f", value);
	this->eog_threshold_ = value;
}

float EogBci::GetFrameRate(void) {
	
	return this->framerate_;
}

void EogBci::on_received_data(const rosneuro_msgs::NeuroFrame::ConstPtr& msg) {

	if(msg->eeg.info.nsamples == this->n_samples_ && msg->eeg.info.nchannels == this->n_channels_) {
		this->new_neuro_frame_ = true;
		this->data_ = msg->eeg.data;
	}

}

bool EogBci::Apply(void) {

	// Copy data in eigen structure
	if(this->new_neuro_frame_== false)
	{
		//ROS_WARN("Not available data to classify");
		return false;
	}

    // Take the data
	this->dmap_ = Eigen::Map<Eigen::MatrixXf>(this->data_.data(), this->n_channels_, this->n_samples_);
	this->dmap_.transposeInPlace();

	// Extract channels from buffer
	this->dfet_.col(0) = this->dmap_.col(this->chleft_).cast<double>();
	this->dfet_.col(1) = this->dmap_.col(this->chright_).cast<double>();

	// Compute HEOG and VEOG
	this->heog_ = this->dfet_.col(0) - this->dfet_.col(1);	
	this->veog_ = (this->dfet_.col(0) + this->dfet_.col(1)) / 2.0f;	

	// Rectify
	this->hvalue_ = this->heog_.cwiseAbs();
	this->vvalue_ = this->veog_.cwiseAbs();

    // Set data used
	this->new_neuro_frame_= false;

	return true;
}

void EogBci::HasArtifacts(void) {
	if(this->hvalue_.maxCoeff() >= this->eog_threshold_ || this->vvalue_.maxCoeff() >= this->eog_threshold_){
        this->emsg_.header = this->msg_.header;
	 	this->emsg_.header.stamp = ros::Time::now();
	 	this->emsg_.event = EOG_EVENT;
	
		// Publish starting eog
	 	this->pub_data_.publish(this->emsg_);
		ROS_INFO("EOG detected"); 

		ros::Duration(2.0).sleep();

		this->emsg_.header = this->msg_.header;
	 	this->emsg_.header.stamp = ros::Time::now();
	 	this->emsg_.event = EOG_EVENT_FINISH;
	
		// Publish finish eog
	 	this->pub_data_.publish(this->emsg_);
    }
}

void EogBci::on_reconfigure_callback(cybathlon_eog::EogBciConfig &config, 
								uint32_t level) {
	
	// Eog threshold
	if(this->update_if_different(config.eog_threshold, this->eog_threshold_))
		ROS_WARN("Updated eog threshold to %f", this->eog_threshold_);

}

bool EogBci::update_if_different(const double& first, double& second, double epsilon) {

	bool is_different = false;
	if(std::fabs(first - second) >= epsilon) {
		second = first;
		is_different = true;
	}

	return is_different;
}

}

#endif
