#include <ros/ros.h>
#include "cybathlon_eog/CybathlonEog.hpp"

int main(int argc, char** argv) {

	
	// ros initialization
	ros::init(argc, argv, "cybathlonEog_node");

	cybathlon::EogBci eogbci;
	
	if(eogbci.configure()== false) {
		std::cerr<<"SETUP ERROR"<<std::endl;
		return -1;
	}

	ros::Rate r(256);
	while(ros::ok())
	{
		 if(eogbci.Apply() == true) {
			ROS_INFO_ONCE("Eog detection started"); 
		 }

		eogbci.HasArtifacts();

         ros::spinOnce();
		 r.sleep();
	}

	ros::shutdown();
	return 0;
}
