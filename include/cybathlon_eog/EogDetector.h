#ifndef CYBATHLON_EOGDETECTOR_H_
#define CYBATHLON_EOGDETECTOR_H_

#include <string>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <dynamic_reconfigure/server.h>

#include <rosneuro_msgs/NeuroFrame.h>
#include <rosneuro_msgs/NeuroOutput.h>
#include <rosneuro_data/NeuroDataTools.hpp>
#include <rosneuro_msgs/NeuroEvent.h>
#include "cybathlon_eog/EogDetectorConfig.h"


namespace cybathlon {

using config_eogdetector = cybathlon_eog::EogDetectorConfig;
using dyncfg_eogdetector = dynamic_reconfigure::Server<config_eogdetector>;

class EogDetector {


    public:

        EogDetector(void);
        ~EogDetector(void);
        
        bool configure(void);
		void run(void);


    private: 
        void on_receive_data(const rosneuro_msgs::NeuroFrame::ConstPtr& msg);
		void on_timer_elapsed(const ros::TimerEvent& event);
        void on_reconfigure_callback(config_eogdetector &config, uint32_t level);

        bool  update_if_different(const double& first, double& second, double epsilon = 0.00001);

		void init_timer(double period, bool oneshot);

    private: 

        ros::NodeHandle nh_;
        ros::NodeHandle p_nh_;
        std::string     sub_topic_data_;
        std::string     pub_topic_data_;
        ros::Subscriber sub_data_;
        ros::Publisher  pub_data_;
		ros::Timer 		timer_;

        rosneuro_msgs::NeuroEvent   msg_;
		
		dyncfg_eogdetector* recfg_srv_;
  		dyncfg_eogdetector::CallbackType recfg_callback_type_;
		boost::recursive_mutex recfg_mutex_;

        unsigned int     buffer_size_;
        unsigned int     n_channels_;
        unsigned int     n_samples_;
        
        double          eog_fcoff1_;
        double          eog_fcoff2_;
        unsigned int    eog_lchannel_;
        unsigned int    eog_rchannel_;
        unsigned int    chleft_;
        unsigned int    chright_;

		int 	eog_event_;
        double	eog_period_;
        double  eog_threshold_;
		bool 	is_period_elapsed_;
		bool 	is_eog_detected_;


        

        Eigen::MatrixXf dmap_;
        Eigen::MatrixXd dfet_;
        Eigen::VectorXd heog_;
        Eigen::VectorXd veog_;
        Eigen::VectorXd hvalue_;
        Eigen::VectorXd vvalue_;

};

    
}


#endif
