#include <QCoreApplication>
#include <ActsAsynch.h>
#include "MCSControl.h"
#include <string>
#include <iostream>
#include <stdio.h>
#include <sstream>
#include <stdlib.h>
using namespace std;

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

        SA_INDEX mcsHandle; //handle to opened mcs system
        const char loc[] = "network:192.168.1.200:5000"; //system locater
        int ConnChan[6] = {1,1,0,0,0,0};
        SA_INDEX chanA = 0;
        SA_INDEX chanB = 1;
        double speednan = 500000; //closed loop move speed in nm/s
        double distnan = -2000000; //0.1 mm
        double distmm = 1.0; //1mm
        double speed =0.5;
        double accel = 0.0;
        unsigned int max_freq = 2000; //max frequency of the system Hz
        SA_PACKET packet;
        double pos_mm[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
        SA_STATUS result;
        int stoppedA = 1;
        int stoppedB = 1;

      /******************************************************************
                         INITIALIZATION
         ******************************************************************/
        ActsAsynch LinActs; //initialize the smaract system object
        LinActs.Init(&mcsHandle, loc); //initialize smaract system connection
        LinActs.SetMaxFrequencies(mcsHandle,ConnChan, max_freq); //set max freq

     /******************************************************************
                        CALIBRATE AND SET ZERO
     ******************************************************************/
        LinActs.CalibrateSensor_All(mcsHandle, ConnChan);//calibrate all
        LinActs.ZeroActs_All(mcsHandle, ConnChan);//zero all actuators

     /******************************************************************
                          MOVE
     ******************************************************************/
        //LinActs.MoveRelative(mcsHandle, chanA, distmm, speed, accel);//move relative
        //LinActs.MoveRelative(mcsHandle, chanB, distmm, speed, accel);//move relative
        LinActs.FindPosition_All(mcsHandle,ConnChan,pos_mm); //read position
        cout << "Position A " << pos_mm[0] << endl;
        cout << "Position B " << pos_mm[1] << endl;
        return a.exec();
}

