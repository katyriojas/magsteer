/**********************************************************************
* Copyright (c) 2013 SmarAct GmbH
*
* This is a programming example for the Modular Control System API.
* It demonstrates programming with asynchronous functions.
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


void PrintMcsError(SA_STATUS st) {
    printf("MCS error %u\n",st);
}

/* All MCS commands return a status/error code which helps analyzing 
   problems */
void ExitIfError(SA_STATUS st) {
    if(st != SA_OK) {
        PrintMcsError(st);
        exit(1);
    }
}

int main(int argc, char* argv[])
{
    //Initialize Variables
    SA_INDEX mcsHandle;
    unsigned int numOfChannels = 0;
    SA_INDEX channelA = 0, channelB = 1;
    int stop = 0;
    int chanAStopped = 0, chanBStopped = 0;
    SA_PACKET packet;

    //Open the first MCS with USB interface in asyncronous communication mode
    ExitIfError( SA_OpenSystem(&mcsHandle,"usb:ix:0","async") );

    //Get number of channels connected to this device
    ExitIfError( SA_GetNumberOfChannels(mcsHandle,&numOfChannels) );
    printf("Number of Channels: %u\n",numOfChannels);

    /*Set buffered or unbuffered output, with buffered output the commands are
    collected in a buffer on the PC side*/
    ExitIfError( SA_SetBufferedOutput_A(mcsHandle,SA_BUFFERED_OUTPUT) );

    // Send movement commands for two positioners(stored in buffer)
    ExitIfError( SA_StepMove_A(mcsHandle,channelA,3000,4000,800) );
    ExitIfError( SA_StepMove_A(mcsHandle,channelB,2000,4000,1000) );

    /*FlushOutput sends them to the MCS so both positioners will start
    moving (almost) simultaneously. */
    ExitIfError( SA_FlushOutput_A(mcsHandle) );

    /* now poll the status of the two channels until both have 'stopped' status */

    while(!stop)
    {
        //Store get command in buffer and grab it with flush output
        ExitIfError( SA_GetStatus_A(mcsHandle,channelA) );
        ExitIfError( SA_GetStatus_A(mcsHandle,channelB) );
        ExitIfError( SA_FlushOutput_A(mcsHandle) );

        /* Receive packets from the MCS. The code should be prepared to handle
           unexpected packets like error packets beside the expected ones 
           (here: status packets). Also note that ReceiveNextPacket could 
           timeout before a packet is received, which is indicated by a 
           SA_NO_PACKET_TYPE packet. */

        ExitIfError( SA_ReceiveNextPacket_A(mcsHandle,1000,&packet) );
        switch(packet.packetType)
        {
        case SA_NO_PACKET_TYPE:         /* SA_ReceiveNextPacket_A timed out */
            break;
        case SA_ERROR_PACKET_TYPE:      /* the MCS has sent an error message */
            PrintMcsError(packet.data1);
            stop = 1;
            break;
        case SA_STATUS_PACKET_TYPE:     /* received a channel status packet */
            if(packet.channelIndex == channelA)
            {
                if(!chanAStopped && packet.data1 == SA_STOPPED_STATUS)
                {
                    chanAStopped = 1;
                    printf("channel A has stopped\n");
                }
            }
            else if(packet.channelIndex == channelB)
            {
                if(!chanBStopped && packet.data1 == SA_STOPPED_STATUS)
                {
                    chanBStopped = 1;
                    printf("channel B has stopped\n");
                }
            }
            stop = (chanAStopped && chanBStopped);
            break;
        }
    }

    ExitIfError( SA_CloseSystem(mcsHandle) );

    return 0;
}
