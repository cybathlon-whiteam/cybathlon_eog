#include <ros/ros.h>
#include "cybathlon_eog/EogDetector.h"

int main(int argc, char** argv) {

	
	// ros initialization
	ros::init(argc, argv, "eog_detector");

	cybathlon::EogDetector eogdetector;
	
	if(eogdetector.configure()== false) {
		return -1;
	}

	eogdetector.run();


	ros::shutdown();

	return 0;
}
