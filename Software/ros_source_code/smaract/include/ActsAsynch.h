/********************************************************************
Header file for Acts_Asynch class
Neal P. Dillon
revised:  6/5/2017
********************************************************************/

#ifndef ACTS_H
#define ACTS_H

#include <MCSControl.h> // header file for SmarAct APIv2 functions
#include <string>
#include <iostream>

//using namespace std;

#define COUT if (1) cout // Use this line to toggle btwn debug modes
//1 indicates we will be outputting debug tools
#define NM2MM 0.000001
#define MM2NM 1000000

class ActsAsynch {

public:

    ActsAsynch(); // Constructor
    ~ActsAsynch(); // Destructor

    // Initialize system
    bool Init(SA_INDEX *mcsHandle); // initalize system for asynchronous communication
    bool getInitStatus(); // gets initialization status flag set when Init() function called

    // Zero Actuators
    bool ZeroActs(SA_INDEX mcsHandle, SA_INDEX chan); // zeros position for single actuator
    bool ZeroActs_All(SA_INDEX mcsHandle, int *ConnChannels); // zeros position for all connected actuators
    bool getZeroedStatus();

    // Calibrate Actuators
    bool CalibrateSensor(SA_INDEX mcsHandle, SA_INDEX chan); // calibrate a single actuator
    bool CalibrateSensor_All(SA_INDEX mcsHandle, int *ConnChannels); // calibrate all connected actuators

    // Get Current Position of Actuators
    bool FindPosition(SA_INDEX mcsHandle, SA_INDEX chan, double *pos_mm);
    bool FindPosition_All(SA_INDEX mcsHandle, int *ConnChannels, double *pos_mm);

    // Movement Functions
    bool SetMaxFrequencies(SA_INDEX mcsHandle, int *ConnChannels, unsigned int max_freq);
    bool MoveAbsolute(SA_INDEX mcsHandle, SA_INDEX chan, double pos, double speed, double accel);
    //bool MoveAbsolute_All(SA_INDEX mcsHandle, int *ConnChannels, double *pos, double *speed, double *accel);
    bool MoveRelative(SA_INDEX mcsHandle, SA_INDEX chan, double dist, double speed, double accel);
    bool Stop(SA_INDEX mcsHandle, SA_INDEX chan);

private:

    SA_PACKET packet;
    bool InitStatus;
    bool ZeroedStatus;
    SA_INDEX systemIndex;
};

#endif // ACTS_H
