/********************************************************************
Class for communicating with SmarAct controller in ASYNCHRONOUS mode.
Neal P. Dillon and Katy Riojas
revised:  7/11/2017
********************************************************************/

#include <ActsAsynch.h>

ActsAsynch::ActsAsynch() {
	InitStatus = false; // not initialized yet
	ZeroedStatus = false; // not zeroed yet
} // Constructor

ActsAsynch::~ActsAsynch() { // Deconstructor
    //cout << endl << "Hit return key to end this program.";
    SA_STATUS SA_CloseSystem(SA_INDEX mcsHandle); // Release SmarAct object.
}

/* SYSTEM INITIALIZATION FUNCTIONS */

bool ActsAsynch::Init(SA_INDEX *systemIndex,const char *loc) {
    bool success_flag = true;
    SA_STATUS result = SA_OpenSystem(systemIndex,loc,"async");
    if (result!= SA_OK) {
        COUT << endl << "There has been an error in initalizing the SmarAct MCS Unit";
        COUT << endl << "The code is: " << result << endl;
        success_flag = false;
    }
    else{
        COUT << "System successfully initialized for asynchronous communication." << endl;
		InitStatus = true;
    }
	return success_flag;
}

bool ActsAsynch::getInitStatus() {
	return InitStatus;
}

/* ACTUATOR CALIBRATION & SETUP FUNCTIONS */

bool ActsAsynch::ZeroActs(SA_INDEX mcsHandle, SA_INDEX chan) {
    bool success_flag = true;
    SA_STATUS result = SA_SetPosition_A(mcsHandle, chan, 0);
    if(result != SA_OK) {
        COUT << "Error in zeroing actuator # " << chan << ". Error code " << result << endl;
        success_flag = false;
    }
    return success_flag;
}

bool ActsAsynch::ZeroActs_All(SA_INDEX mcsHandle, int *ConnChannels) {
    bool success_flag = true;
    SA_STATUS result;
    for(int i=0; i<6; i++) {
        if (ConnChannels[i] == 1) {
            result = SA_SetPosition_A(mcsHandle, i, 0);
            if(result != SA_OK) {
                COUT << "Error zeroing Actuator # " << i << ". Error code " << result << endl;
                success_flag = false;
            }
        }
    }
	ZeroedStatus = true;
    return success_flag;
}

bool ActsAsynch::getZeroedStatus() {
	return ZeroedStatus;
}

bool ActsAsynch::CalibrateSensor(SA_INDEX mcsHandle, SA_INDEX chan) {
    bool success_flag = true;
    SA_STATUS result = SA_SetBufferedOutput_A(mcsHandle, SA_BUFFERED_OUTPUT); // Set up buffer to send all commands at once to controller
    result = SA_CalibrateSensor_A(mcsHandle, chan);
    if(result != SA_OK) {
        COUT << "Error in calibrating actuator # " << chan << ". Error code " << result << endl;
        success_flag = false;
    } else {cout << "U calibrated" << endl;}
    SA_FlushOutput_A(mcsHandle);
    return success_flag;
}

bool ActsAsynch::CalibrateSensor_All(SA_INDEX mcsHandle, int *ConnChannels) {
    bool success_flag = true;
    SA_STATUS result;
    result = SA_SetBufferedOutput_A(mcsHandle, SA_BUFFERED_OUTPUT); // Set up buffer to send all commands at once to controller
    for(int i=0; i<6; i++) {
        if (ConnChannels[i] == 1) {
            result = SA_CalibrateSensor_A(mcsHandle, i);
            if(result != SA_OK) {
                COUT << "Error calibrating Actuator # " << i << ". Error code " << result << endl;
                success_flag = false;
            }
        }
    }
    SA_FlushOutput_A(mcsHandle);
    SA_SetBufferedOutput_A(mcsHandle, SA_UNBUFFERED_OUTPUT);
    return success_flag;
}

/* CHECK POSITION FUNCTIONS */

bool ActsAsynch::FindPosition(SA_INDEX mcsHandle, SA_INDEX chan, double *pos_mm) {
	
    SA_STATUS result; // status of various commands
    bool success_flag = true; // function return flag
    signed int pos_nano; // actuator position in nanometers

    // Send GetPosition command
    result = SA_GetPosition_A(mcsHandle, chan);
    if (result != SA_OK) {
        COUT << "Failed to send GetPosition command for actuator #" << chan << ". The status code is: " << result << "." << endl;
        success_flag = false;
    }

    // Next check packets from each channel
    do {
        result = SA_ReceiveNextPacket_A(mcsHandle, 0, &packet);
        if (result != SA_OK) {
            COUT << "Failed to send ReceiveNextPacket command. The status code is: " << result << "." << endl;
            success_flag = false;
        }
    } while (packet.packetType == SA_NO_PACKET_TYPE);

    if (packet.packetType == SA_POSITION_PACKET_TYPE && packet.channelIndex == chan) {
        pos_nano = packet.data2;
        *pos_mm = (double)pos_nano*NM2MM;
    }
    else {
        COUT << "Error receiving position data packet." << endl;
        COUT << endl << "    Packet Type = " << packet.packetType;
        COUT << endl << "    Error Type = " << packet.data1;
        COUT << endl << "    Channel Index = " << packet.channelIndex;
        success_flag = false;
    }

    // Clear any packets from buffer
    do {
        result = SA_ReceiveNextPacket_A(mcsHandle, 0, &packet);
        if (result != SA_OK) {
            COUT << "Failed to send ReceiveNextPacket command while clearing buffer. The status code is: " << result << "." << endl;
            success_flag = false;
        }
    } while (packet.packetType != SA_NO_PACKET_TYPE);

    return success_flag;
}

bool ActsAsynch::FindPosition_All(SA_INDEX mcsHandle, int *ConnChannels, double *pos_mm) {

    SA_STATUS result; // status of various commands
    bool success_flag = true; // function return flag
    signed int pos_nano; // actuator position in nanometers

    // Send GetPosition commands to all actuators connected
    int num_cmds_sent = 0;
    for(int i=0; i<6; i++) {
        if (ConnChannels[i] == 1) {
            result = SA_GetPosition_A(mcsHandle, i);
            if(result != SA_OK) {
                COUT << "Failed to send GetPosition command for actuator #" << i << ". The status code is: " << result << "." << endl;
                success_flag = false;
            }
			num_cmds_sent++;
        }
    }

    // Next check packets from each channel
    for(int i=0; i<num_cmds_sent; i++) {
        do {
            result = SA_ReceiveNextPacket_A(mcsHandle, 0, &packet);
            if (result != SA_OK) {
                COUT << "Failed to send ReceiveNextPacket command. The status code is: " << result << "." << endl;
                success_flag = false;
            }
        } while (packet.packetType == SA_NO_PACKET_TYPE);
		if (packet.packetType == SA_POSITION_PACKET_TYPE) {          
			pos_nano = packet.data2;
            pos_mm[packet.channelIndex] = (double)pos_nano*NM2MM;
        }
        else {
            COUT << "Error receiving position data packet." << endl;
            COUT << endl << "    Packet Type = " << packet.packetType;
            COUT << endl << "    Error Type = " << packet.data1;
            COUT << endl << "    Channel Index = " << packet.channelIndex;
            success_flag = false;
        }
    }
	

    // Clear any packets from buffer
    do {
        result = SA_ReceiveNextPacket_A(mcsHandle, 0, &packet);
        if (result != SA_OK) {
            COUT << "Failed to send ReceiveNextPacket command while clearing buffer. The status code is: " << result << "." << endl;
            success_flag = false;
        }
    } while (packet.packetType != SA_NO_PACKET_TYPE);
    return success_flag;
}

/* MOVEMENT FUNCTIONS */

bool ActsAsynch::SetMaxFrequencies(SA_INDEX mcsHandle, int *ConnChannels, unsigned int max_freq) {

    SA_STATUS result;
    bool success_flag = true;

    result = SA_SetBufferedOutput_A(mcsHandle, SA_BUFFERED_OUTPUT); // Set up buffer to send all commands at once to controller

    if (result != SA_OK) {
        COUT << "Error setting up buffered output. Error code " << result << "." << endl;
        success_flag = false;
    }
    // Call functions to set CL max frequency
    for(int i=0; i<6; i++) {
        if (ConnChannels[i] == 1) {
            result = SA_SetClosedLoopMaxFrequency_A(mcsHandle, i, max_freq);
            if(result != SA_OK) {
                COUT << "Error setting CL Max Frequency for Actuator # " << i << ". Error code " << result << endl;
                success_flag = false;
            }
        }
    }

    // Flush output and update buffer
    SA_FlushOutput_A(mcsHandle);
    SA_SetBufferedOutput_A(mcsHandle, SA_UNBUFFERED_OUTPUT);

    return success_flag;
}

bool ActsAsynch::MoveAbsolute(SA_INDEX mcsHandle, SA_INDEX chan, double pos, double speed, double accel) {

    SA_STATUS result;
    bool success_flag = true;
    int pos_nano; // desired position in nm
    unsigned int speed_nano; // desired speed in nm/s
	unsigned int accel_nano; // desired accel in nm/s^2

    // Set up buffer to send all commands at once to controller
    result = SA_SetBufferedOutput_A(mcsHandle, SA_BUFFERED_OUTPUT);
    if (result != SA_OK) {
        COUT << "Error setting up buffered output for Actuator # " << chan << ". The status code is: " << result << "." << endl;
        success_flag = false;
    }

    // Set new closed-loop acceleration
    accel_nano = accel*MM2NM;
    result = SA_SetClosedLoopMoveAcceleration_A(mcsHandle, chan, accel_nano);
    if (result != SA_OK) {
        COUT << "Error specifying CL acceleration for actuator #" << chan << ". The status code is: " << result << "." << endl;
        success_flag = false;
    }

    // Set new closed-loop speed
    speed_nano = speed*MM2NM;
    result = SA_SetClosedLoopMoveSpeed_A(mcsHandle, chan, speed_nano);
    if (result != SA_OK) {
        COUT << "Error specifying CL speed for actuator #" << chan << ". The status code is: " << result << "." << endl;
        success_flag = false;
    }

    // Send absolute position command
    pos_nano = pos*MM2NM;
    result = SA_GotoPositionAbsolute_A(mcsHandle, chan, pos_nano, 0);
    if (result != SA_OK) {
        COUT << "Error commanding new position for actuator #" << chan << ". The status code is: " << result << "." << endl;
        success_flag = false;
    }

    SA_FlushOutput_A(mcsHandle);
    SA_SetBufferedOutput_A(mcsHandle, SA_UNBUFFERED_OUTPUT);

    return success_flag;
}

bool ActsAsynch::MoveRelative(SA_INDEX mcsHandle, SA_INDEX chan, double dist, double speed, double accel) {

    SA_STATUS result;
    bool success_flag = true;
    signed int dist_nano; // desired position in nm
    unsigned int speed_nano; // desired speed in nm/s
    unsigned int accel_nano; // desired accel in nm/s^2

	// Set up buffer to send all commands at once to controller
    result = SA_SetBufferedOutput_A(mcsHandle, SA_BUFFERED_OUTPUT);
    if (result != SA_OK) {
        cout << "Error setting up buffered output for Actuator # " << chan << ". The status code is: " << result << "." << endl;
        success_flag = false;
    }

    // Set new closed-loop acceleration
    accel_nano = accel*MM2NM;
    result = SA_SetClosedLoopMoveAcceleration_A(mcsHandle, chan, accel_nano);
    if (result != SA_OK) {
        COUT << "Error specifying CL acceleration for actuator #" << chan << ". The status code is: " << result << "." << endl;
        success_flag = false;
    }

    // Set new closed-loop speed
    speed_nano = speed*MM2NM;
    result = SA_SetClosedLoopMoveSpeed_A(mcsHandle, chan, speed_nano);
    if (result != SA_OK) {
        COUT << "Error specifying CL speed for actuator #" << chan << ". The status code is: " << result << "." << endl;
        success_flag = false;
    }

    // Send absolute position command
    dist_nano = dist*MM2NM;
    result = SA_GotoPositionRelative_A(mcsHandle, chan, dist_nano, 0);
    if (result != SA_OK) {
        COUT << "Error commanding relative motion distance for actuator #" << chan << ". The status code is: " << result << "." << endl;
        success_flag = false;
    }

    SA_FlushOutput_A(mcsHandle);
    SA_SetBufferedOutput_A(mcsHandle, SA_UNBUFFERED_OUTPUT);

    return success_flag;
}

bool ActsAsynch::Stop(SA_INDEX mcsHandle, SA_INDEX chan) {

	bool success_flag = true;
	SA_STATUS result;
    result = SA_Stop_A(mcsHandle, chan);
	if (result != SA_OK) {
        COUT << "Error clearing command queue for Actuator # " << chan << ". The status code is: " << result << "." << endl;
        success_flag = false;
    }
	return success_flag;
}
