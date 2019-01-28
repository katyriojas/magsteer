/********************************************************************
 Main file for smaract node that reads the current position and publishes that data, 
with a subscriber reading that position and publishing it.

Katy Riojas & Neal Dillon
revised:  6/9/2017
********************************************************************/

#include "ros/ros.h"
#include "iostream"
#include "MCSControl.h" //SmarAct header
#include "ActsAsynch.h" //Actuator class header
#include "smaract/act_data.h"
#include "smaract/setZeroActs.h" //header file generated from srv
#include "smaract/moveRelative.h" 
#include "smaract/moveAbsolute.h" 
#include "smaract/movetoLimit.h" 

using namespace std;

// GLOBAL VARIABLES
ActsAsynch LinActs; // SmarAct object
int ConnChan[6] = {0, 0, 0, 0, 0, 0}; // list of which SmarAct channels are connected
SA_INDEX mcsHandle;
SA_STATUS result; //holds initialization status

// SERVICE PROTOTYPES
bool zeroedActs(smaract::setZeroActs::Request &req, smaract::setZeroActs::Response &res);
bool cmd_moveRelative(smaract::moveRelative::Request &req, smaract::moveRelative::Response &res);
bool cmd_moveAbsolute(smaract::moveAbsolute::Request &req, smaract::moveAbsolute::Response &res);
bool cmd_movetoLimit(smaract::movetoLimit::Request &req, smaract::movetoLimit::Response &res);

int main(int argc, char **argv) {

//set variable types if will be putting them into the functions in the .cpp file of the class
    //const char loc[] = "network:192.168.1.200:5000";
    double dist = -2.0; // distance (in mm) to move away from end stop after zeroing 
    unsigned int maxfreq = 500; // max frequency of actuators in Hz
    double speed = 0.10; // [mm/s] speed of actuator
    double accel = 0; //[mm/s^2] acceleration of actuators
	unsigned int safeDir = 1; //specifying safe direction is in the forward direction
   
/*******************************************************************************
		SETUP NODE, PUBLISHER, SERVICES
********************************************************************************/	

	ros::init(argc, argv, "smaract_control"); // Initialize Node
	ros::NodeHandle n;
	ROS_INFO("%s", "Starting node to control SmarAct actuators.");

	// Setup publishing of force_readings
	ros::Publisher act_data_pub  = n.advertise<smaract::act_data>("act_data", 1); //publish custom msg

	// Create and advertise service
    ros::ServiceServer srv_setZeroActs = n.advertiseService("setZeroActs", zeroedActs);
	ros::ServiceServer srv_moveRelative = n.advertiseService("moveRelative", cmd_moveRelative); 
	ros::ServiceServer srv_moveAbsolute = n.advertiseService("moveAbsolute", cmd_moveAbsolute); 
	ros::ServiceServer srv_movetoLimit = n.advertiseService("movetoLimit", cmd_movetoLimit); 

/*******************************************************************************
		Initialize systema and get vector of connected actuators
********************************************************************************/

	// Initialize connection based on asynchronous communication    
    result = LinActs.Init(&mcsHandle);
	cout << result << endl;	
	if (result != SA_OK) {
		ROS_INFO("SmarAct system SUCCESSFULLY initialized");
	}
	else {
		ROS_ERROR("SmarAct system not successfully initialized, sorry.");
	}
// Get connected channels and write to screen
	ConnChan[0] = 1;
	ConnChan[1] = 1;
	ROS_INFO("%s = [%d, %d, %d, %d, %d, %d]", "Connected Channels", ConnChan[0], ConnChan[1], 		
	ConnChan[2], ConnChan[3], ConnChan[4], ConnChan[5]); //output which of the smar acts are connected
	//result = LinActs.getInitStatus();
	//cout <<"Initialized?" << result << endl;	

/****************************************************************************
                Set max frequencies and safe direction of travel
*****************************************************************************/
	
LinActs.SetMaxFrequencies(mcsHandle, ConnChan, maxfreq); // set max frequency for all connected actuators
	
SA_STATUS resultSD_outer = SA_SetSafeDirection_A(mcsHandle, 0, 0); //set safe direction of connected actuator one
        if(resultSD_outer != SA_OK) {
                COUT << "Error setting Safe Direction for Outer Tube. Error code " << resultSD_outer << endl;
            }
		else COUT << "Safe direction set" << endl;
	SA_STATUS resultSD_inner = SA_SetSafeDirection_A(mcsHandle, 1, 0); //set safe direction of actuator two
        if(resultSD_inner != SA_OK) {
                COUT << "Error setting Safe Direction for Inner Tube. Error code " << resultSD_inner << endl;
            }
		else COUT << "Safe direction set" << endl;

	
/*******************************************************************************
		LOOP: READ AND PUBLISH POSITION, MOVE ACCORDING TO SERVICE CALLS
********************************************************************************/

	// Loop rate and speed variables
	smaract::act_data msg;
	ros::Rate loop_rate(250);
	int count = 0;
	double pos_mm[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double chanpos[1] = {0.0};
	bool find_pos_result;

	while (ros::ok()) {

		LinActs.FindPosition_All(mcsHandle, ConnChan, pos_mm);	
		//LinActs.FindPosition(mcsHandle,0,&chanpos);
		//msg.act_pos.at(0) = chanpos;
		for (int i=0; i<6; i++) {
		
			if (ConnChan[i]==1) {
				if (find_pos_result) {
					msg.STATUS.at(i) = 1;
				}
				else {
					msg.STATUS.at(i) = 0;
				}
			msg.act_pos.at(i) = pos_mm[i]; 
			}
	
	}
		act_data_pub.publish(msg); //publish this message*/		
		ros::spinOnce(); //Spin
		loop_rate.sleep(); //Sleep
		count++;
	}
	return 0;

} // end main

/*******************************************************************************
		                     SERVICES
********************************************************************************/
// Service for sending movement command for actuator to hit end stop
bool cmd_movetoLimit(smaract::movetoLimit::Request &req, smaract::movetoLimit::Response &res) {	
	
	if (LinActs.getInitStatus()) {
        ROS_INFO("Sending calibration command to move to hard stop."); //info stored about request
	   if (LinActs.CalibrateSensor(mcsHandle, req.chan)) {
	   ROS_INFO("movetoLimit command successful.");
	   res.moveSuccess = true;
	   return true;
		}
		else {
	    ROS_INFO("movetoLimit command unsuccessful.");
	    res.moveSuccess = false;
	    return false;
		}
	}
}

// Service for triggering zeroing of SmarActs
bool zeroedActs(smaract::setZeroActs::Request &req, smaract::setZeroActs::Response &res) {
	if (LinActs.getInitStatus()) {
		LinActs.ZeroActs_All(mcsHandle, ConnChan);
		res.zeroedActs = LinActs.getZeroedStatus(); // flag true if actuators are zeroed
		ROS_INFO("Requesting zeroing commands: %d", req.zeroActsYESNO);
   		ROS_INFO("Sending back Zeroed Status: %d", res.zeroedActs);
		return true;
	}
	else {
		ROS_INFO("Cannot zero before SmarAct system is initialized.");
		res.zeroedActs = false;
		return false;
	}
}

// Service for sending relative movement command to any number of actuators
bool cmd_moveRelative(smaract::moveRelative::Request &req, smaract::moveRelative::Response &res) {
	if (LinActs.getInitStatus()) {
		ROS_INFO("Sending relative movement command."); //info stored about request
		if (LinActs.MoveRelative(mcsHandle, req.chan, req.dist, req.speed, req.accel)) {
			ROS_INFO("Command successful.");
			res.moveSuccess = true;
			return true;
		}
		else {
			ROS_INFO("Command unsuccessful.");
			res.moveSuccess = false;
			return false;
		}
	}
	else {
		ROS_INFO("Cannot send MoveRelative command because SmarActs not initialized yet.");
		res.moveSuccess = false;
		return false;
	}
}

// Service for sending absolute movement command to any number of actuators
bool cmd_moveAbsolute(smaract::moveAbsolute::Request &req, smaract::moveAbsolute::Response &res) {
	if (LinActs.getInitStatus()) {
		ROS_INFO("Sending absolute movement command."); //info stored about request
		if (LinActs.MoveAbsolute(mcsHandle, req.chan, req.pos, req.speed, req.accel)) {
			ROS_INFO("Command successful.");
			res.moveSuccess = true;
			return true;
		}
		else {
			ROS_INFO("Command unsuccessful.");
			res.moveSuccess = false;
			return false;
		}
	}
	else {
		ROS_INFO("Cannot send MoveAbsolute command because SmarActs not initialized yet.");
		res.moveSuccess = false;
		return false;
	}
}

