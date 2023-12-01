#ifndef CYBATHLON_EOG_HPP
#define CYBATHLON_EOG_HPP

#include <string>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <dynamic_reconfigure/server.h>

#include "rosneuro_msgs/NeuroFrame.h"
#include "rosneuro_msgs/NeuroOutput.h"
#include "rosneuro_data/NeuroDataTools.hpp"
#include "rosneuro_msgs/NeuroEvent.h"
#include "cybathlon_eog/EogBciConfig.h"

#define EOG_CHANNELS 2
int EOG_EVENT = 1024;

namespace cybathlon {

typedef dynamic_reconfigure::Server<cybathlon_eog::EogBciConfig> EogBciReconfig;

class EogBci {


    public:

        EogBci(void);
        ~EogBci(void);
        
        bool configure(void);
        float GetFrameRate(void);
        void SetThreshold(double value);
        bool Apply(void);
        void HasArtifacts(void);


    private: 
        void on_received_data(const rosneuro_msgs::NeuroFrame::ConstPtr& msg);
        void on_reconfigure_callback(cybathlon_eog::EogBciConfig &config, 
                                     uint32_t level);

        bool  update_if_different(const double& first, double& second, double epsilon = 0.00001);


    private: 

        ros::NodeHandle           nh_;
        ros::NodeHandle           p_nh_;
        std::string               sub_topic_data_;
        std::string               pub_topic_data_;
        bool                      new_neuro_frame_;
        bool                      detect_eog_;
        ros::Subscriber           sub_data_;
        ros::Publisher            pub_data_;

        rosneuro_msgs::NeuroOutput  msg_;
        rosneuro_msgs::NeuroEvent   emsg_;

        EogBciReconfig                    reconfig_server_;
        EogBciReconfig::CallbackType      reconfig_function_;


        unsigned int     buffer_size_;
        unsigned int     n_channels_;
        unsigned int     n_samples_;
        unsigned int     sampling_freq_;
        
        unsigned int    eog_channels_;
        double          eog_fcoff1_;
        double          eog_fcoff2_;
        double          eog_threshold_;
        unsigned int    eog_lchannel_;
        unsigned int    eog_rchannel_;
        unsigned int    chleft_;
        unsigned int    chright_;

        double               time_eog_;
		ros::Time            time_detect_eog_;
		ros::Duration        time_for_eog_;
        
        float         framerate_;

        std::vector<float> data_;
        Eigen::MatrixXf dmap_;
        Eigen::MatrixXd dfet_;
        Eigen::VectorXd heog_;
        Eigen::VectorXd veog_;
        Eigen::VectorXd hvalue_;
        Eigen::VectorXd vvalue_;

};

    
}


#endif
