#include "cybathlon_eog/EogDetector.h"

namespace cybathlon {

EogDetector::EogDetector(void): p_nh_("~") {

    this->sub_topic_data_  =  "/neurodata_filtered";
    this->pub_topic_data_  =  "/events/bus";
}

EogDetector::~EogDetector(void) {
}

bool EogDetector::configure(void) {
    
    // Take parameters 
    ros::param::param("~buffer_size", (int&) this->buffer_size_, 512);
    ros::param::param("~n_channels", (int&) this->n_channels_, 16);
    ros::param::param("~n_samples", (int&) this->n_samples_, 32);

    ros::param::param("~eog_event", (int&) this->eog_event_, 1024);
    ros::param::param("~eog_period", (double&) this->eog_period_, 2.0);
    ros::param::param("~eog_threshold", (double&) this->eog_threshold_, 30.0);
    ros::param::param("~eog_left_channel", (int&) this->eog_lchannel_, 12); 
    ros::param::param("~eog_right_channel", (int&) this->eog_rchannel_, 16);   


    // Setup EOG channels to channel vector indeces
    this->chleft_ = this->eog_lchannel_ - 1; // Gloria we can avoid to define two variables
    this->chright_ = this->eog_rchannel_ - 1; // Gloria we can avoid to define two variables
    
    
    // Setup temporary data matrices 
    this->dmap_ = Eigen::MatrixXf::Zero(this->n_channels_, this->n_samples_);
    this->dfet_ = Eigen::MatrixXd::Zero(this->n_samples_, 2);
    this->heog_     = Eigen::VectorXd::Zero(this->n_samples_);
    this->veog_     = Eigen::VectorXd::Zero(this->n_samples_);
    this->hvalue_ = Eigen::VectorXd::Zero(this->buffer_size_);
    this->vvalue_ = Eigen::VectorXd::Zero(this->buffer_size_);

    // Set subscriber and publisher
    this->sub_data_ = this->p_nh_.subscribe(this->sub_topic_data_, 1, &EogDetector::on_receive_data, this);
    this->pub_data_ = this->p_nh_.advertise<rosneuro_msgs::NeuroEvent>(this->pub_topic_data_, 1); 


	// Set Timer for EOG (oneshot)
	this->init_timer(this->eog_period_, true);

    // Dynamic reconfiguration server
	this->recfg_srv_ = new dyncfg_eogdetector(this->recfg_mutex_);
	this->recfg_callback_type_ = boost::bind(&EogDetector::on_reconfigure_callback, this, _1, _2);
	this->recfg_srv_->setCallback(this->recfg_callback_type_);
	


    ROS_INFO("[cybathlon_eog] EOG Detector correctly configured");

    return true;

}



void EogDetector::on_receive_data(const rosneuro_msgs::NeuroFrame::ConstPtr& msg) {

	std::vector<float> data;

    if(msg->eeg.info.nsamples != this->n_samples_ || msg->eeg.info.nchannels != this->n_channels_) {
		ROS_ERROR("[cybathlon_eog] Received a different number of samples or channels");
		return;
    }

	data = msg->eeg.data;

    // Convert data in eigen
    this->dmap_ = Eigen::Map<Eigen::MatrixXf>(data.data(), this->n_channels_, this->n_samples_);
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

	// Check for values overthreshold
	if(this->hvalue_.maxCoeff() >= this->eog_threshold_ || 
	   this->vvalue_.maxCoeff() >= this->eog_threshold_) {

		this->is_eog_detected_ = true;
    } else {
		this->is_eog_detected_ = false;
	}

}

void EogDetector::on_timer_elapsed(const ros::TimerEvent& event) {
	this->is_period_elapsed_ = true;

	// Publish off event
    this->msg_.header.stamp = ros::Time::now();
    this->msg_.event = this->eog_event_ + 0x8000;
    this->pub_data_.publish(this->msg_);
}

void EogDetector::run(void) {

	ros::Rate r(512);

	while(ros::ok()) {

		if(this->is_eog_detected_ == true && this->is_period_elapsed_ == true) {
			
			// publish message
        	this->msg_.header.stamp = ros::Time::now();
        	this->msg_.event = this->eog_event_;
        	this->pub_data_.publish(this->msg_);

			// Restart timer
			this->timer_.start();
			this->is_period_elapsed_ = false;
		}

		ros::spinOnce();
		r.sleep();
	}


}

/*
bool EogDetector::Apply(void) {

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

void EogDetector::HasArtifacts(void) {
    if(this->hvalue_.maxCoeff() >= this->eog_threshold_ || this->vvalue_.maxCoeff() >= this->eog_threshold_){
        this->emsg_.header = this->msg_.header;
        this->emsg_.header.stamp = ros::Time::now();
        this->emsg_.event = EOG_EVENT;
    
        // Publish starting eog
        this->pub_data_.publish(this->emsg_);
        //ROS_INFO("EOG detected"); 
        this->detect_eog_ = true;
        this->time_detect_eog_ = this->emsg_.header.stamp;
    }
    if(this->detect_eog_ && 
        ((ros::Time::now().toSec() - this->time_detect_eog_.toSec()) >= this->time_eog_)){
        
        this->detect_eog_ = false;

        // publish finish EOG
        this->emsg_.header = this->msg_.header;
        this->emsg_.header.stamp = ros::Time::now();
        this->emsg_.event = EOG_EVENT + 0x8000;
        //ROS_INFO("Finish EOG");
    
        this->pub_data_.publish(this->emsg_);
    }
}*/

void EogDetector::on_reconfigure_callback(config_eogdetector &config, uint32_t level) {
    
    // Eog threshold
    if(this->update_if_different(config.eog_threshold, this->eog_threshold_))
        ROS_WARN("Updated eog threshold to %f", this->eog_threshold_);

    // Eog threshold
    if(this->update_if_different(config.eog_period, this->eog_period_)){
		this->init_timer(this->eog_period_, true);
        ROS_WARN("Updated eog time to wait after EOG detection %f", this->eog_period_);
	}

}

bool EogDetector::update_if_different(const double& first, double& second, double epsilon) {

    bool is_different = false;
    if(std::fabs(first - second) >= epsilon) {
        second = first;
        is_different = true;
    }

    return is_different;
}

void EogDetector::init_timer(double period, bool oneshot) {

	this->timer_ = this->nh_.createTimer(ros::Duration(period), 
									     &EogDetector::on_timer_elapsed, this, oneshot); 
	this->timer_.stop();
}

}

