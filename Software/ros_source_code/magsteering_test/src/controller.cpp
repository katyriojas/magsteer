/* This node will give zero commands to zero the force sensor and the 
smaracts. It will also give movement commands to the smaracts.
It will subscribe to force data and position data*/

#include "controller.h"
// Global variables
smaract::act_data msg_pos;
ati_ft_sensor::force_data msg_ft;
int chan_outer = 0;
int chan_inner = 1;

int main(int argc, char **argv)
{

/********************************************************************************************
		SETUP NODE
********************************************************************************************/
	
	ros::init(argc, argv, "controller");
	ros::NodeHandle n;
	
	// Setup subscribers
	ros::Subscriber sub1 = n.subscribe("act_data", 1, act_dataCallback); // actuator data
	ros::Subscriber sub2 = n.subscribe("force_data", 1, force_dataCallback); // force data

	// Setup clients
	// (1) Zeroing FT sensor
	ros::ServiceClient clientFT = n.serviceClient<ati_ft_sensor::setZeroFT>("setZeroFT");    
    ati_ft_sensor::setZeroFT srvFTzero; // creating variable srv that is same type as setZero
	// (2) Zeroing Actuators	
	ros::ServiceClient clientAct = n.serviceClient<smaract::setZeroActs>("setZeroActs"); 
    smaract::setZeroActs srvActzero; // creating variable srv that is same type as setZero
	// (3) Relative motion of actuators
	ros::ServiceClient clientMoveRelative = n.serviceClient<smaract::moveRelative>("moveRelative");
	smaract::moveRelative moveRel_cmd;
	// (4) Absolute motion of actuators
	ros::ServiceClient clientMoveAbsolute = n.serviceClient<smaract::moveAbsolute>("moveAbsolute");
	smaract::moveAbsolute moveAbs_cmd;
	// (5) Move actuator to hit hard stop
    ros::ServiceClient clientmovetoLimit = n.serviceClient<smaract::movetoLimit>("movetoLimit");
	smaract::movetoLimit movetoLimit_cmd;

/********************************************************************************************
		GET PARAMETERS FROM SERVER
********************************************************************************************/

	double insertion_vel;
	if (ros::param::has("insertion_vel")) {
		ros::param::get("insertion_vel", insertion_vel);
	}
	else {
		insertion_vel = 0.5; // default value
	}

	double safety_offset;
	if (ros::param::has("safety_offset")) {
		ros::param::get("safety_offset", safety_offset);
	}
	else {
		safety_offset = 1.0; // default value to add to electrode length to back off from wire hardstop
	}
	

	double target_offset;
	if (ros::param::has("target_offset")) {
		ros::param::get("target_offset", target_offset);
	}
	else {
		target_offset = 10.0; // default value, amount that the two tubes will move together
	}
    
	double electrode_length;
	if (ros::param::has("electrode_length")) {
		ros::param::get("electrode_length", electrode_length);
	}
	else {
		electrode_length = 25.0; // default value, equal to slit length in this case
	}
	
	double delta; //diff between back of slit and inner wire when both at max
	if (ros::param::has("delta")) {
		ros::param::get("delta", delta);
	}
	else {
		delta = 4.0; // default value, change this value to change difference in inner and outer tube at zero position depending on electrode length
	}

/********************************************************************************************
		ZERO FT SENSOR-- uncomment this section when including the force sensor
********************************************************************************************/	
    cout << "\nPress ENTER to zero the force sensor: "; //asking for user input
    cin.get();
    srvFTzero.request.zeroFTYESNO = true;
   
        if (!clientFT.call(srvFTzero)) {
      ROS_ERROR("Failed to call service 'setZeroFT'"); //error, no info from the server
      return 1;
    }

/********************************************************************************************
		MOVE OUTER TUBE TO HIT THE HARD STOP
********************************************************************************************/
    cout << "\nPress ENTER to move outer and inner tubes to hit their end stops"; //asking for user input
    cin.get();
    movetoLimit_cmd.request.calibrate = true;
    movetoLimit_cmd.request.chan = chan_outer;
	
	if (!clientmovetoLimit.call(movetoLimit_cmd)) {
	  ROS_ERROR("Failed to call service 'movetoLimit'"); //error, no info from the server received
	}
	movetoLimit_cmd.request.chan = chan_inner;
	
	if (!clientmovetoLimit.call(movetoLimit_cmd)) {
	  ROS_ERROR("Failed to call service 'movetoLimit'"); //error, no info from the server received
	}
//move inner to hit hard stop

/********************************************************************************************
		  MOVE OUTER AND INNER TUBE INTO STARTING POSITION
********************************************************************************************/	
	double speed_jog = 1.5; // mm/s
	double accel = 0.0; // mm/s^2
	
	cout << "\nPress ENTER to move both tubes to starting position (inner tube to the back of slit and outer tube back from hard stop): "; //asking for user input
    cin.get(); // press enter to move to retracted position
			moveRel_cmd.request.chan = chan_outer;		
			moveRel_cmd.request.dist = (electrode_length - delta);//backoff from hard stop
			moveRel_cmd.request.speed = speed_jog;
			moveRel_cmd.request.accel = accel;
			//ROS_INFO("Sending service triggered by pressing enter");
			if (!clientMoveRelative.call(moveRel_cmd)) {
				ROS_ERROR("Failed to call service 'moveRelative'");
    		}
//move inner to face 
			moveRel_cmd.request.chan = chan_inner;		
			moveRel_cmd.request.dist = electrode_length + safety_offset; //distance from endstop of inner tube to right behind electrode
			moveRel_cmd.request.speed = speed_jog;
			moveRel_cmd.request.accel = accel;
			//ROS_INFO("Sending service triggered by pressing enter");
			if (!clientMoveRelative.call(moveRel_cmd)) {
				ROS_ERROR("Failed to call service 'moveRelative'");
    		}

/********************************************************************************************
		ZERO SMARACTS
********************************************************************************************/	

    cout << "\nPress ENTER to zero the actuators: "; //asking for user input
    cin.get();
    srvActzero.request.zeroActsYESNO = true;
   
	if (!clientAct.call(srvActzero)) {
      ROS_ERROR("Failed to call service 'setZeroActs'"); //error, no info from the server received
      return 1;
    }

/********************************************************************************************
		LOOP: ALLOW FOR USER TO SPECIFY WHEN TO START MOTION
********************************************************************************************/	
	
	cout << "\nPress SPACEBAR to retract both tubes together and align tool. \nLoad Electrode\nPress again to move to opening.\nPress once more to begin insertion:\n"; //asking for user input
	bool tool_in_position = false;	
	bool tool_aligned = false;
	bool insertion_started = false;
	int key_value = 0;
	ros::Rate loop_rate(100);
	while (ros::ok()) {

		key_value = 0;
		key_value = kbhit();
		if (key_value == 32) {
			if (!tool_in_position) {
				moveAbs_cmd.request.chan = chan_outer;
				moveAbs_cmd.request.speed = 0.5;
				moveAbs_cmd.request.accel = 0.0;				
				moveAbs_cmd.request.pos = target_offset;
				if (!clientMoveAbsolute.call(moveAbs_cmd)) {
					ROS_ERROR("Failed to call service 'moveAbsolute' for outer tube");
    			}
				moveAbs_cmd.request.chan = chan_inner; //moving both tubes back by target offset from hardstop+1mm
				if (!clientMoveAbsolute.call(moveAbs_cmd)) {
					ROS_ERROR("Failed to call service 'moveAbsolute' for inner tube");
    			}
				tool_in_position = true;
				//cout << "\nAfter tubes are in position near cochleostomy, press SPACEBAR to begin insertion. Press again to retract when finished.";
			}
			else if (!tool_aligned) {
				moveAbs_cmd.request.chan = chan_outer;
				moveAbs_cmd.request.speed = insertion_vel;
				moveAbs_cmd.request.accel = 0.0;				
				moveAbs_cmd.request.pos = 0.0; //both move to entry point (what we set as zero)
				if (!clientMoveAbsolute.call(moveAbs_cmd)) {
					ROS_ERROR("Failed to call service 'moveAbsolute' for outer tube");
    			}
				moveAbs_cmd.request.chan = chan_inner;
				if (!clientMoveAbsolute.call(moveAbs_cmd)) {
					ROS_ERROR("Failed to call service 'moveAbsolute' for inner tube");
    			}

				tool_aligned = true;
			}

			else if (!insertion_started) {
				moveAbs_cmd.request.chan = chan_inner;
				moveAbs_cmd.request.speed = insertion_vel;
				moveAbs_cmd.request.accel = 0.0;				
				moveAbs_cmd.request.pos = - (electrode_length - 0.5); //moving wire past outer tube
				if (!clientMoveAbsolute.call(moveAbs_cmd)) {
					ROS_ERROR("Failed to call service 'moveAbsolute' for inner tube");
    			}
				insertion_started = true;
			}
		}

		printf("\rPosition = [% 05.2f, % 05.2f] mm. Force = [% 05.2f, % 05.2f, % 05.2f] N    ", msg_pos.act_pos.at(0), msg_pos.act_pos.at(1), msg_ft.Fx, msg_ft.Fy, msg_ft.Fz);
		
		ros::spinOnce(); // Spin
		loop_rate.sleep(); // Sleep

	}
 
	return 0;
}

/********************************************************************************************
		SUBSCRIBER CALLBACKS
********************************************************************************************/	

//subscribing to smaract position data
void act_dataCallback(const smaract::act_data::ConstPtr &msg)
{
  	//ROS_INFO("Position [%2.4f, %2.4f]. Status [%d, %d]", msg->act_pos.at(0), msg->act_pos.at(1), msg->STATUS.at(0), msg->STATUS.at(1));
	msg_pos.act_pos.at(0) = msg->act_pos.at(0);
	msg_pos.act_pos.at(1) = msg->act_pos.at(1);
	msg_pos.STATUS.at(0) = msg->STATUS.at(0);
	msg_pos.STATUS.at(1) = msg->STATUS.at(1);
}

//subscribing to force sensor data
void force_dataCallback(const ati_ft_sensor::force_data::ConstPtr &msg)
{
  //ROS_INFO("Force = [%2.2f, %2.2f, %2.2f]", msg->Fx, msg->Fy, msg->Fz);
	msg_ft.Fx = msg->Fx;
	msg_ft.Fy = msg->Fy;
	msg_ft.Fz = msg->Fz;
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

