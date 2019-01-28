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


/* All MCS commands return a status/error code which helps analyzing 
   problems */
void ExitIfError(SA_STATUS st) {
    if(st != SA_OK) {
        printf("MCS error %u\n",st);
        exit(1);
    }
}

int main(int argc, char* argv[])
{
    unsigned int sensorEnabled = 0;
    SA_INDEX mcsHandle;

    /* When opeing a controller you must select one of the two communication
       modes:
    sync: only commands from the set of synchronous commands can 
        be used in the program. In sync. communication mode commands like
        GetPosition, GetStatus etc. return the requested value directly. 
        this is easier to program, especially for beginners.
    async: only asynchronous commands can be used. In this mode Get... 
        commands send a request message to the MCS controller but do not 
        wait for the reply. The replied message must be catched with special
        commands ReceiveNextPacket, ReceiveNextPacketIfChannel or 
        LookAtNextPacket, which are only available in async. communication
        mode. Please read the MCS Programmer's Guide for more information. */

    /* Open the first MCS with USB interface in synchronous communication mode */
    ExitIfError( SA_OpenSystem(&mcsHandle,"usb:ix:0","sync") );

    /* Now the MCS is initialized and can be used.
       In this demo program all we do is reading the sensor power-mode. */

    ExitIfError( SA_GetSensorEnabled_S(mcsHandle,&sensorEnabled) );
    switch(sensorEnabled)
    {
    case SA_SENSOR_DISABLED: printf("Sensors are disabled\n"); break;
    case SA_SENSOR_ENABLED: printf("Sensors are enabled\n"); break;
    case SA_SENSOR_POWERSAVE: printf("Sensors are in power-save mode\n"); break;
    default: printf("Error: unknown sensor power status\n"); break;
    }

    /* At the end of the program you should release all opened systems. */

    ExitIfError( SA_CloseSystem(mcsHandle) );

    return 0;
}