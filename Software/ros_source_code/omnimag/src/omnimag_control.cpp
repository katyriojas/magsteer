/********************************************************************
 Main file for Omnimag
 Katy Riojas

 Last Revised:  1/22/2019
********************************************************************/

#include "ros/ros.h"
#include "omnimag/mag_data.h"

using namespace std;

// PUT GLOBAL VARIABLES HERE

int main(int argc, char **argv) {
   
/*******************************************************************************
		SETUP NODE AND PUBLISHER
********************************************************************************/	

	ros::init(argc, argv, "omnimag_control"); // Initialize Node
	ros::NodeHandle n;
	ROS_INFO("%s", "Starting Omnimag Node.");

	// Setup publishing of field
	ros::Publisher mag_data_pub  = n.advertise<omnimag::mag_data>("mag_data", 1); //publish desired magnetic field

/*******************************************************************************
		LOOP: PUBLISH MAGNETIC FIELD
********************************************************************************/

	// Loop rate and speed variables
	omnimag::mag_data msg;
	ros::Rate loop_rate(250);
	int count = 0;

	while (ros::ok()) {
		
		//Let's create a loop to periodically change the magnetic field specified
		//for (int i=0; i<6; i++) {	
			msg.mag_field.at(0) = count/2; 
			msg.mag_field.at(1) = count/3; 	    
			msg.mag_field.at(2) = count/4; 

		mag_data_pub.publish(msg); //publish this message	
		ros::spinOnce(); //Spin
		loop_rate.sleep(); //Sleep

		count++; //increment count
		if (count>100) {
			count = 0;
		}

		}
	return 0;

} // end main
