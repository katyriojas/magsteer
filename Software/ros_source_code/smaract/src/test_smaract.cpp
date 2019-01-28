#include "ros/ros.h"
#include "smaract/act_data.h"
#include "smaract/setZeroActs.h"
#include "smaract/moveRelative.h"
#include "smaract/moveAbsolute.h"
#include "smaract/movetoLimit.h"
#include "iostream"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
using namespace std;
#include "read_ft_sensor.h"

int kbhit(void);
smaract::act_data msg_pos;

//subscribing to smaract position data
void act_dataCallback(const smaract::act_data::ConstPtr &msg) {
        ROS_INFO("Position [%2.4f, %2.4f]. Status [%d, %d]", msg->act_pos.at(0), msg->act_pos.at(1), msg->STATUS.at(0), msg->STATUS.at(1));
	// Assign values to global variable so main loop can see position
	msg_pos.act_pos.at(0) = msg->act_pos.at(0);
	msg_pos.act_pos.at(1) = msg->act_pos.at(1);
	msg_pos.STATUS.at(0) = msg->STATUS.at(0);
	msg_pos.STATUS.at(1) = msg->STATUS.at(1);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_smaract");
	ros::NodeHandle n;

	// SUBSCRIBE TO ACTUATOR DATA
	ros::Subscriber sub1 = n.subscribe("act_data", 1, act_dataCallback);

/********************************************************************************************
		SETUP SERVICES: Zeroing, Relative Movement
********************************************************************************************/

	// Set Zero
	ros::ServiceClient clientActZero = n.serviceClient<smaract::setZeroActs>("setZeroActs");
	smaract::setZeroActs setZero_cmd; // creating variable srv that is same type as setZero
	
	// Move Relative
	ros::ServiceClient clientMoveRelative = n.serviceClient<smaract::moveRelative>("moveRelative");
	smaract::moveRelative moveRel_cmd;

	// Move Absolute
	ros::ServiceClient clientMoveAbsolute = n.serviceClient<smaract::moveAbsolute>("moveAbsolute");
	smaract::moveAbsolute moveAbs_cmd;

	// Move to Mechanical Hard Stop
	ros::ServiceClient clientmovetoLimit = n.serviceClient<smaract::movetoLimit>("movetoLimit");
	smaract::movetoLimit movetoLimit_cmd;

/********************************************************************************************
		ZERO SMARACT SERVICE: Asking user for input to zero smaract 	
********************************************************************************************/
    
    cout << "Press Enter zero the actuator: "; // asking for user input
    cin.get();
    setZero_cmd.request.zeroActsYESNO = true; //occupy requested info with user input
   
	if (clientActZero.call(setZero_cmd)) { //returns true if call to service is successful
	  ROS_INFO("Actuators zeroed successfully.");
    }
    else {
      ROS_ERROR("Failed to call service 'setZeroActs'"); //error, no info from the server received
      return 1;
    }

/********************************************************************************************
		LOOP: SUBSCRIBE TO POSITION, CHECK USER INPUT TO SEND MOVEMENT COMMANDS
********************************************************************************************/	
	
	// Relative movement parameters
	int chan_outer = 0;
	int chan_inner = 1; 
	double ui_loop_rate = 10.0; // Hz
	double speed = 0.5; // mm/s
	double dist; // mm
	dist = speed/ui_loop_rate;
	double accel = 0.0; // mm/s^2
	
	ros::Rate loop_rate((int)ui_loop_rate);
	int key_value = 0;
	while (ros::ok()) {

		key_value = 0;
		key_value = kbhit();

//*************** MOVE RELATIVE KEYSTROKES ******************************
                if (key_value == 46) { // comma ',' (move outer chan bkwd dir (away from wire))
			moveRel_cmd.request.chan = chan_outer;			
			moveRel_cmd.request.dist = dist;
			moveRel_cmd.request.speed = speed;
			moveRel_cmd.request.accel = accel;
			if (!clientMoveRelative.call(moveRel_cmd)) {
				ROS_ERROR("Failed to call service 'moveRelative'");
    		}
		}
                else if (key_value == 44) { // period '.' (move outer chan fwd dir (toward wire))
			moveRel_cmd.request.chan = chan_outer;			
			moveRel_cmd.request.dist = -dist;
			moveRel_cmd.request.speed = speed;
			moveRel_cmd.request.accel = accel;
			if (!clientMoveRelative.call(moveRel_cmd)) {
				ROS_ERROR("Failed to call service 'moveRelative'");
    		}
		}
                else if (key_value == 60) { // < (move inner chan bkwd dir (away from wire)) (SHIFT COMMA)
			moveRel_cmd.request.chan = chan_inner;			
			moveRel_cmd.request.dist = dist;
			moveRel_cmd.request.speed = speed;
			moveRel_cmd.request.accel = accel;
			if (!clientMoveRelative.call(moveRel_cmd)) {
				ROS_ERROR("Failed to call service 'moveRelative'");
    		}
		}

                else if (key_value == 62) { // > (move inner chan fwd dir (toward wire)) (SHIFT PERIOD)
			moveRel_cmd.request.chan = chan_inner;			
			moveRel_cmd.request.dist = -dist;
			moveRel_cmd.request.speed = speed;
			moveRel_cmd.request.accel = accel;
			if (!clientMoveRelative.call(moveRel_cmd)) {
				ROS_ERROR("Failed to call service 'moveRelative'");
    		}
		}

//*************** MOVE ABSOLUTE KEYSTROKE ******************************

            else if (key_value == 39) { // Apostrophe (move chan outer back to start zero)
			moveAbs_cmd.request.chan = chan_outer;			
			moveAbs_cmd.request.pos = 0.0;
			moveAbs_cmd.request.speed = 5.0;
			moveAbs_cmd.request.accel = accel;
			if (!clientMoveAbsolute.call(moveAbs_cmd)) {
				ROS_ERROR("Failed to call service 'moveAbsolute'");
    		}
		}

		else if (key_value == 34) { // Quote (move chan inner back to start zero) shift apost
			moveAbs_cmd.request.chan = chan_inner;			
			moveAbs_cmd.request.pos = 0.0;
			moveAbs_cmd.request.speed = 5.0;
			moveAbs_cmd.request.accel = accel;
			if (!clientMoveAbsolute.call(moveAbs_cmd)) {
				ROS_ERROR("Failed to call service 'moveAbsolute'");
    		}
		}

//*************** MOVE TO LIMIT KEYSTROKE ******************************

		else if (key_value ==59) { //Semicolon (will home chan outer in specified safe direction)
			movetoLimit_cmd.request.chan = chan_outer;			
			movetoLimit_cmd.request.calibrate = true;				
			if (!clientmovetoLimit.call(movetoLimit_cmd)) {
				ROS_ERROR("Failed to send Actuator to Positive Mechanical End Stop");
    		}
		}

		else if (key_value ==58) { // Colon (will home chan inner in specified safe direction)
			movetoLimit_cmd.request.chan = chan_inner;			
			movetoLimit_cmd.request.calibrate = true;				
			if (!clientmovetoLimit.call(movetoLimit_cmd)) {
				ROS_ERROR("Failed to send Actuator to Positive Mechanical End Stop");
    		}
		}

		ROS_INFO("Position [%2.4f, %2.4f]. Status [%d, %d]", msg_pos.act_pos.at(0), msg_pos.act_pos.at(1), msg_pos.STATUS.at(0), msg_pos.STATUS.at(1));
		
		ros::spinOnce(); // Spin
		loop_rate.sleep(); // Sleep

	}
	return 0;
}

/********************************************************************************************
		OTHER FUNCTIONS
********************************************************************************************/	

int kbhit(void) {

	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if(ch != EOF)
	{
		//ungetc(ch, stdin);
		return ch;
	}
	else {
		return 0;
	}
}

