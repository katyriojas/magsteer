#include "ros/ros.h"
#include "omnimag/mag_data.h"
//#include "omnimag/setField.h"
#include "iostream"
using namespace std;

/********************************************************************
File for testing omnimag node (omnimag_control)

Katy Riojas
Last Revised:  1/22/2019
********************************************************************/

// %Tag(CALLBACK)%
void mag_dataCallback(const omnimag::mag_data::ConstPtr &msg)
{
  ROS_INFO("Magnetic Field: [%2.3f, %2.3f, %2.3f]", msg->mag_field.at(0), msg->mag_field.at(1), msg->mag_field.at(2));
}
// %EndTag(CALLBACK)%


int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_omnimag");
	ros::NodeHandle n;
    ROS_INFO("%s", "Starting node to test omnimag node.");

	// Setup subscriber to mag field data
	omnimag::mag_data msg;
	ros::Subscriber sub = n.subscribe("mag_data", 1, mag_dataCallback);

/********************************************************************************************
		LOOP: SUBSCRIBE TO Magfield Data
********************************************************************************************/	
	
	//ros::Rate loop_rate((int)ui_loop_rate);
	while (ros::ok()) {

		//ROS_INFO("Position [%2.4f, %2.4f]. Status [%d, %d]", msg_pos.act_pos.at(0), msg_pos.act_pos.at(1), msg_pos.STATUS.at(0), msg_pos.STATUS.at(1));
		
	ros::spinOnce();

	}
	return 0;
}
