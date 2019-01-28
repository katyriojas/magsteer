/********************************************************************
File for testing force sensing node (read_ft_sensor)

Neal P. Dillon
revised:  6/1/2017
********************************************************************/

#include "test_ft_sensor.h"

// %Tag(CALLBACK)%
void force_dataCallback(const ati_ft_sensor::force_data::ConstPtr &msg)
{
  ROS_INFO("F = [%2.3f, %2.3f, %2.3f]", msg->Fx, msg->Fy, msg->Fz);
}
// %EndTag(CALLBACK)%

int main(int argc, char **argv) {

	// Initialize Node
	ros::init(argc, argv, "test_ft_sensor");
	ros::NodeHandle n;
	ROS_INFO("%s", "Starting node to test force sensor.");

	// Setup subscriber
	ati_ft_sensor::force_data msg;
	ros::Subscriber sub = n.subscribe("force_data", 1, force_dataCallback);
		
	// Wait for service call to zero sensor
	ros::ServiceClient clientFT = n.serviceClient<ati_ft_sensor::setZeroFT>("setZeroFT");    
    ati_ft_sensor::setZeroFT srvFTzero; //creating variable srv that is same type as setZero

    cout << "Press ENTER to zero the force sensor: "; //asking for user input
    cin.get(); //ask user to say ya(1) or nah (0) to zeroing  
    srvFTzero.request.zeroFTYESNO = true; //occupy requested info with user input
   
	if (clientFT.call(srvFTzero)) //returns true if call to service is successful
    {
	  ROS_INFO("Zeroed Status: %d", srvFTzero.response.zeroedFT); //print zero status
    }
    else {
      ROS_ERROR("Failed to call service 'setZeroFT'"); //error, no info from the server
      return 1;
    }
	
	// Spin
	ros::spin();
	
	return 0;
}
