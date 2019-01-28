/**********************************************************************
* Copyright (c) 2013 SmarAct GmbH
*
* This is a programming example for the Modular Control System API.
*
* THIS  SOFTWARE, DOCUMENTS, FILES AND INFORMATION ARE PROVIDED 'AS IS'
* WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING,
* BUT  NOT  LIMITED  TO,  THE  IMPLIED  WARRANTIES  OF MERCHANTABILITY,
* FITNESS FOR A PURPOSE, OR THE WARRANTY OF NON-INFRINGEMENT.
* THE  ENTIRE  RISK  ARISING OUT OF USE OR PERFORMANCE OF THIS SOFTWARE
* REMAINS WITH YOU.
* IN  NO  EVENT  SHALL  THE  SMARACT  GMBH  BE  LIABLE  FOR ANY DIRECT,
* INDIRECT, SPECIAL, INCIDENTAL, CONSEQUENTIAL OR OTHER DAMAGES ARISING
* OUT OF THE USE OR INABILITY TO USE THIS SOFTWARE.
**********************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <MCSControl.h>
#ifdef SA_PLATFORM_WINDOWS
#  include <Windows.h>
#else
#  include <time.h>
#endif



int main(int argc, char* argv[])
{
    SA_STATUS error = SA_OK;
    SA_INDEX mcsHandle;
    unsigned int numOfChannels = 0;
    unsigned int channel = 0;
    unsigned int sensorType;
    int linearSensorPresent = 0;
    int key;
    unsigned int status;
    int position;

    // ----------------------------------------------------------------------------------
    // open the first MCS with USB interface in synchronous communication mode
    error = SA_OpenSystem(&mcsHandle,"usb:ix:0","sync");
    printf("Open system: Error: %u\n", error);
    if(error)
        return 1;

    error = SA_GetNumberOfChannels(mcsHandle,&numOfChannels);
    printf("Number of Channels: %u\n",numOfChannels);


    // ----------------------------------------------------------------------------------
    // check availability of linear sensor. only if a sensor is present
    // the position can be read with SA_GetPosition_S
    error = SA_GetSensorType_S(mcsHandle, channel, &sensorType);												
    if (sensorType == SA_S_SENSOR_TYPE ||
        sensorType == SA_M_SENSOR_TYPE ||
        sensorType == SA_SC_SENSOR_TYPE ||
        sensorType == SA_SP_SENSOR_TYPE) {
            linearSensorPresent = 1;
            printf("Linear sensor present\n");
        } else {
            linearSensorPresent = 0;
            printf("No linear sensor present\n");
        }
    // ----------------------------------------------------------------------------------
    if(linearSensorPresent)
    {
        printf("\nENTER COMMAND AND RETURN\n"
                "+  Move positioner up by 100um\n"
                "-  Move positioner down by 100um\n"
                "q  Quit program\n");
    }else{
        printf("\nENTER COMMAND AND RETURN\n"
                "+  Move positioner up by 200 steps\n"
                "-  Move positioner down by 200 steps\n"
                "q  Quit program\n");
    }
    // ----------------------------------------------------------------------------------
    do
    {
        key = getchar();
        if (key == '-')											
            if (linearSensorPresent)							
                SA_GotoPositionRelative_S(mcsHandle, channel, -100000, 1000);	
            else
                SA_StepMove_S(mcsHandle, channel,-200, 4095, 2000);							

        if (key == '+')											
            if (linearSensorPresent)							
                SA_GotoPositionRelative_S(mcsHandle, channel, 100000, 1000);	
            else
                SA_StepMove_S(mcsHandle, channel, 200, 4095, 2000);							

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // wait until movement has finished
        // in synchronous communication mode, the current status of each channel
        // must be checked periodically ('polled') to know when a movement has
        // finished:
        do {
            SA_GetStatus_S(mcsHandle, channel, &status);	
#ifdef SA_PLATFORM_WINDOWS
            Sleep(50);
#else
            timespec tim;
            tim.tv_sec  = 0;
            tim.tv_nsec = 50000000L;
            nanosleep(&tim,NULL);
#endif
        } while (status == SA_TARGET_STATUS);

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        if (linearSensorPresent) {
            SA_GetPosition_S(mcsHandle, channel, &position);
            printf("Position: %d nm\n", position);
        }
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    } while (key != 'q');

    error = SA_CloseSystem(mcsHandle);
    printf("Close system: Error: %u\n", error);

    return 0;
}