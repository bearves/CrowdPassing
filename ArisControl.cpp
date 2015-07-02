/*
 * Push recovery mimic program
 *
 *
 *  Created on: Jun 18, 2015
 *      Author: Q. Sun
 */
#include <iostream>
#include "Aris_Control.h"
#include "Aris_Message.h"
#include "Aris_Thread.h"
#include "Aris_Socket.h"
#include "Server.h"

using namespace std;
using namespace Aris::RT_CONTROL;
using namespace RobotHighLevelControl;

static CGait gait;
static EGAIT gaitcmd[AXIS_NUMBER];
static EGAIT gaitcmdtemp[AXIS_NUMBER];

Aris::RT_CONTROL::ACTUATION controlSystem;
Aris::RT_CONTROL::CSysInitParameters initParam;

static const char *GS_STRING[] = 
{
    "GAIT_NULL          ",
    "GAIT_STANDSTILL    ",
    "GAIT_HOME2START    ",
    "GAIT_MOVE          ",
    "GAIT_MOVE_BACK     ",
    "GAIT_FAST_MOVE     ",
    "GAIT_FAST_MOVE_BACK",
    "GAIT_TROT          ",
    "GAIT_LEGUP         ",
    "GAIT_TURN_LEFT     ",
    "GAIT_TURN_RIGHT    ",
    "GAIT_HOME          ",
    "GAIT_ONLINE        "
};

static const double forceForTest[] =
{
    0,   0,   0,
    90,  0,   0,
    120, 0,   0,
    260, 0,   0,
    0,   90,  0,
    0,   120, 0,
    0,   260, 0,
    60,  60,  0,
    100, 100, 0
};

double givenForce[] = {0, 0, 0};

static int forceSelectionFlag = 0;

enum MACHINE_CMD
{
    NOCMD         = 1000,
    POWEROFF      = 1001,
    STOP          = 1002,
    ENABLE        = 1003,
    RUNNING       = 1004,
    GOHOME_1      = 1005,
    GOHOME_2      = 1006,
    HOME2START_1  = 1007,
    HOME2START_2  = 1008,
    FORWARD       = 1009,
    BACKWARD      = 1010,
    FAST_FORWARD  = 1011,
    FAST_BACKWARD = 1012,
    LEGUP         = 1013,
    TURNLEFT      = 1014,
    TURNRIGHT     = 1015,
    ONLINEGAIT    = 1016,
    ONLINEBEGIN   = 1017,
    ONLINEEND     = 1018,
    SET_NO_FORCE  = 1025,
    SET_FORCE_1   = 1026,
    SET_FORCE_2   = 1027,
    SET_FORCE_3   = 1028,
    SET_FORCE_4   = 1029,
    SET_FORCE_5   = 1030,
    SET_FORCE_6   = 1031,
    SET_FORCE_7   = 1032,
    SET_FORCE_8   = 1033,
    CLEAR_FORCE   = 1034
};

enum REPLY_MSG_ID
{
    DATA_REPORT   = 1050
};

int count;
int rtCycleCounter = 0;

// Message loop daemon
void* MessageLoopDaemon(void *)
{
    cout<<"running msgloop"<<endl;
    Aris::Core::RunMsgLoop();
    return NULL;
};


int initFun(Aris::RT_CONTROL::CSysInitParameters& param)
{
    for ( int i = 0; i < AXIS_NUMBER; i++)
    {
        gaitcmd[i] = GAIT_NULL;
        gaitcmdtemp[i] = GAIT_NULL;
    }
    forceSelectionFlag = 0;
    gait.InitGait(param);
    return 0;
};

int tg(Aris::RT_CONTROL::CMachineData& machineData,
       Aris::RT_CONTROL::RT_MSG& msgRecv, 
       Aris::RT_CONTROL::RT_MSG& msgSend)
{
    int CommandID;

    rtCycleCounter++;

    double timeNow = rtCycleCounter * 0.001;

    if (rtCycleCounter % 1000 == 0)
    {
        for( int i = 0; i < 1; i++ )
        {
            rt_printf("No. %d GS. %d MS. %d POS. %d \n",
                 i, gaitcmd[i], machineData.motorsStates[i], machineData.feedbackData[i].Position);
            rt_printf("Force given: %4.1lf, %4.1lf, %4.1lf\n", givenForce[0], givenForce[1], givenForce[2]);
        }
        
    }
    if (rtCycleCounter % 200 == 0){
        rt_printf("Actual Force: %4.1lf, %4.1lf, %4.1lf\n", 
                    machineData.forceData[0].forceValues[0]/1000.0,
                    machineData.forceData[0].forceValues[1]/1000.0,
                    machineData.forceData[0].forceValues[2]/1000.0);
    }

    //if (rtCycleCounter % 100 == 0)
    //{
        //msgSend.SetMsgID(DATA_REPORT);
        //msgSend.Copy((const void*)&machineData, sizeof(machineData));
        
        //controlSystem.RT_PostMsg(msgSend);
    //}

    CommandID=msgRecv.GetMsgID();
    switch(CommandID)
    {
        case Aris::RT_CONTROL::RT_MSG::INVALID_MSG_ID:
            break;
        case NOCMD:
            for(int i=0;i<18;i++)
            {
                machineData.motorsCommands[i]=EMCMD_NONE;
                gaitcmd[i] = GAIT_NULL;
            }
            rt_printf("NONE Command Get in NRT\n" );
            break;

        case ENABLE:
            gait.onlinePlanner.Offline();
            for(int i=0;i<18;i++)
            {
                machineData.motorsCommands[i]=EMCMD_ENABLE;
                gaitcmd[i] = GAIT_NULL;
            }
            rt_printf("ENABLE Command Get in NRT\n" );

            break;
        case POWEROFF:
            gait.onlinePlanner.Offline();
            for(int i=0;i<18;i++)
            {
                machineData.motorsCommands[i]=EMCMD_POWEROFF;
                gaitcmd[i] = GAIT_NULL;
            }
            rt_printf("POWEROFF Command Get in NRT\n" );

            break;
        case STOP:
            gait.onlinePlanner.Offline();
            for(int i=0;i<18;i++)
            {
                machineData.motorsCommands[i]=EMCMD_STOP;
                gaitcmd[i] = GAIT_NULL;
            }
            rt_printf("STOP Command Get in NRT\n" );

            break;
        case RUNNING:
            gait.onlinePlanner.Offline();
            for(int i=0;i<18;i++)
            {
                machineData.motorsCommands[i]=EMCMD_RUNNING;
                gaitcmd[i] = GAIT_STANDSTILL;
            }
            rt_printf("RUNNING Command Get in NRT\n" );
            break;

        case GOHOME_1:

            machineData.motorsCommands[MapAbsToPhy[0]]=EMCMD_GOHOME;
            machineData.motorsCommands[MapAbsToPhy[1]]=EMCMD_GOHOME;
            machineData.motorsCommands[MapAbsToPhy[2]]=EMCMD_GOHOME;
            machineData.motorsCommands[MapAbsToPhy[6]]=EMCMD_GOHOME;
            machineData.motorsCommands[MapAbsToPhy[7]]=EMCMD_GOHOME;
            machineData.motorsCommands[MapAbsToPhy[8]]=EMCMD_GOHOME;
            machineData.motorsCommands[MapAbsToPhy[12]]=EMCMD_GOHOME;
            machineData.motorsCommands[MapAbsToPhy[13]]=EMCMD_GOHOME;
            machineData.motorsCommands[MapAbsToPhy[14]]=EMCMD_GOHOME;

            gaitcmd[MapAbsToPhy[0]]=EGAIT::GAIT_HOME;
            gaitcmd[MapAbsToPhy[1]]=EGAIT::GAIT_HOME;
            gaitcmd[MapAbsToPhy[2]]=EGAIT::GAIT_HOME;
            gaitcmd[MapAbsToPhy[6]]=EGAIT::GAIT_HOME;
            gaitcmd[MapAbsToPhy[7]]=EGAIT::GAIT_HOME;
            gaitcmd[MapAbsToPhy[8]]=EGAIT::GAIT_HOME;
            gaitcmd[MapAbsToPhy[12]]=EGAIT::GAIT_HOME;
            gaitcmd[MapAbsToPhy[13]]=EGAIT::GAIT_HOME;
            gaitcmd[MapAbsToPhy[14]]=EGAIT::GAIT_HOME;

            rt_printf("GOHOME_1 Command Get in NRT\n" );

            break;

        case GOHOME_2:

            machineData.motorsCommands[MapAbsToPhy[3]]=EMCMD_GOHOME;
            machineData.motorsCommands[MapAbsToPhy[4]]=EMCMD_GOHOME;
            machineData.motorsCommands[MapAbsToPhy[5]]=EMCMD_GOHOME;
            machineData.motorsCommands[MapAbsToPhy[9]]=EMCMD_GOHOME;
            machineData.motorsCommands[MapAbsToPhy[10]]=EMCMD_GOHOME;
            machineData.motorsCommands[MapAbsToPhy[11]]=EMCMD_GOHOME;
            machineData.motorsCommands[MapAbsToPhy[15]]=EMCMD_GOHOME;
            machineData.motorsCommands[MapAbsToPhy[16]]=EMCMD_GOHOME;
            machineData.motorsCommands[MapAbsToPhy[17]]=EMCMD_GOHOME;

            gaitcmd[MapAbsToPhy[3]]=EGAIT::GAIT_HOME;
            gaitcmd[MapAbsToPhy[4]]=EGAIT::GAIT_HOME;
            gaitcmd[MapAbsToPhy[5]]=EGAIT::GAIT_HOME;
            gaitcmd[MapAbsToPhy[9]]=EGAIT::GAIT_HOME;
            gaitcmd[MapAbsToPhy[10]]=EGAIT::GAIT_HOME;
            gaitcmd[MapAbsToPhy[11]]=EGAIT::GAIT_HOME;
            gaitcmd[MapAbsToPhy[15]]=EGAIT::GAIT_HOME;
            gaitcmd[MapAbsToPhy[16]]=EGAIT::GAIT_HOME;
            gaitcmd[MapAbsToPhy[17]]=EGAIT::GAIT_HOME;


            rt_printf("GOHOME_2 Command Get in NRT\n" );

            break;

        case HOME2START_1:

            if(gait.m_gaitState[MapAbsToPhy[0]]==GAIT_STOP)
            {
                for(int i=0;i<18;i++)
                {
                    machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
                }
                gaitcmd[MapAbsToPhy[0]]=EGAIT::GAIT_HOME2START;
                gaitcmd[MapAbsToPhy[1]]=EGAIT::GAIT_HOME2START;
                gaitcmd[MapAbsToPhy[2]]=EGAIT::GAIT_HOME2START;
                gaitcmd[MapAbsToPhy[6]]=EGAIT::GAIT_HOME2START;
                gaitcmd[MapAbsToPhy[7]]=EGAIT::GAIT_HOME2START;
                gaitcmd[MapAbsToPhy[8]]=EGAIT::GAIT_HOME2START;
                gaitcmd[MapAbsToPhy[12]]=EGAIT::GAIT_HOME2START;
                gaitcmd[MapAbsToPhy[13]]=EGAIT::GAIT_HOME2START;
                gaitcmd[MapAbsToPhy[14]]=EGAIT::GAIT_HOME2START;

                rt_printf("HOME2START_1 Command Get in NRT\n" );

            }
            break;

        case HOME2START_2:

            if(gait.m_gaitState[MapAbsToPhy[3]]==GAIT_STOP)
            {
                for(int i=0;i<18;i++)
                {
                    machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
                }

                gaitcmd[MapAbsToPhy[3]]=EGAIT::GAIT_HOME2START;
                gaitcmd[MapAbsToPhy[4]]=EGAIT::GAIT_HOME2START;
                gaitcmd[MapAbsToPhy[5]]=EGAIT::GAIT_HOME2START;
                gaitcmd[MapAbsToPhy[9]]=EGAIT::GAIT_HOME2START;
                gaitcmd[MapAbsToPhy[10]]=EGAIT::GAIT_HOME2START;
                gaitcmd[MapAbsToPhy[11]]=EGAIT::GAIT_HOME2START;
                gaitcmd[MapAbsToPhy[15]]=EGAIT::GAIT_HOME2START;
                gaitcmd[MapAbsToPhy[16]]=EGAIT::GAIT_HOME2START;
                gaitcmd[MapAbsToPhy[17]]=EGAIT::GAIT_HOME2START;

                rt_printf("HOME2START_2 Command Get in NRT\n" );
            }
            break;

        case FORWARD:
            for(int i=0;i<18;i++)
            {
                machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
                gaitcmdtemp[i]=EGAIT::GAIT_MOVE;
                machineData.motorsCommands[i]=EMCMD_RUNNING;


                if(gait.m_gaitState[i]!=GAIT_STOP)
                {
                    gait.Gait_iter[i]=gait.Gait_iter[i]+1;
                }
                else
                {
                    gaitcmd[i]=gaitcmdtemp[i];
                    gait.Gait_iter[i]=1;
                }

            }
            break;
        case BACKWARD:
            for(int i=0;i<18;i++)
            {
                machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
                gaitcmdtemp[i]=EGAIT::GAIT_MOVE_BACK;
                machineData.motorsCommands[i]=EMCMD_RUNNING;


                if(gait.m_gaitState[i]!=GAIT_STOP)
                {
                    gait.Gait_iter[i]=gait.Gait_iter[i]+1;
                }
                else
                {
                    gaitcmd[i]=gaitcmdtemp[i];
                    gait.Gait_iter[i]=1;
                }

            }
            break;

        case FAST_FORWARD:
            for(int i=0;i<18;i++)
            {
                machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
                gaitcmdtemp[i]=EGAIT::GAIT_FAST_MOVE;
                machineData.motorsCommands[i]=EMCMD_RUNNING;


                if(gait.m_gaitState[i]!=GAIT_STOP)
                {
                    gait.Gait_iter[i]=gait.Gait_iter[i]+1;
                }
                else
                {
                    gaitcmd[i]=gaitcmdtemp[i];
                    gait.Gait_iter[i]=1;
                }

            }
            break;

        case FAST_BACKWARD:
            for(int i=0;i<18;i++)
            {
                machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
                gaitcmdtemp[i]=EGAIT::GAIT_FAST_MOVE_BACK;
                machineData.motorsCommands[i]=EMCMD_RUNNING;


                if(gait.m_gaitState[i]!=GAIT_STOP)
                {
                    gait.Gait_iter[i]=gait.Gait_iter[i]+1;
                }
                else
                {
                    gaitcmd[i]=gaitcmdtemp[i];
                    gait.Gait_iter[i]=1;
                }

            }
            break;

        case LEGUP:
            if(gait.m_gaitState[MapAbsToPhy[0]]==GAIT_STOP)
            {
                for(int i=0;i<18;i++)
                {
                    machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
                    gaitcmd[MapAbsToPhy[i]]=EGAIT::GAIT_LEGUP;
                    machineData.motorsCommands[i]=EMCMD_RUNNING;

                }
            }
            break;

        case TURNLEFT:
            if(gait.m_gaitState[MapAbsToPhy[0]]==GAIT_STOP)
            {
                for(int i=0;i<18;i++)
                {
                    machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
                    gaitcmd[MapAbsToPhy[i]]=EGAIT::GAIT_TURN_LEFT;
                    machineData.motorsCommands[i]=EMCMD_RUNNING;

                }
            }
            break;

        case TURNRIGHT:
            if(gait.m_gaitState[MapAbsToPhy[0]]==GAIT_STOP)
            {
                for(int i=0;i<18;i++)
                {
                    machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
                    gaitcmd[MapAbsToPhy[i]]=EGAIT::GAIT_TURN_RIGHT;
                    machineData.motorsCommands[i]=EMCMD_RUNNING;
                }
            }
            break;

        case ONLINEGAIT:
            // TODO: add online trj code here

            gait.onlinePlanner.Initialize(1);
            if(gait.m_gaitState[MapAbsToPhy[0]]==GAIT_STOP)
            {
                for(int i=0;i<18;i++)
                {
                    machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
                    gaitcmd[MapAbsToPhy[i]]=EGAIT::GAIT_ONLINE;
                    machineData.motorsCommands[i]=EMCMD_RUNNING;
                }
            }
            break;

        case ONLINEBEGIN:
            gait.onlinePlanner.Start(timeNow);
            break;

        case ONLINEEND:
            gait.onlinePlanner.Stop(timeNow);
            break;

        case SET_NO_FORCE:
            forceSelectionFlag = 0;
            break;

        case SET_FORCE_1:
            forceSelectionFlag = 1;
            break;
        case SET_FORCE_2:
            forceSelectionFlag = 2;
            break;
        case SET_FORCE_3:
            forceSelectionFlag = 3;
            break;
        case SET_FORCE_4:
            forceSelectionFlag = 4;
            break;
        case SET_FORCE_5:
            forceSelectionFlag = 5;
            break;
        case SET_FORCE_6:
            forceSelectionFlag = 6;
            break;
        case SET_FORCE_7:
            forceSelectionFlag = 7;
            break;
        case SET_FORCE_8:
            forceSelectionFlag = 8;
            break;
        case CLEAR_FORCE:
            machineData.forceData[0].isZeroingRequest = 1;
            break; 

        default:
            //DO NOTHING, CMD AND TRAJ WILL KEEP STILL
            break;
    }

    /*for(int i = 0; i < 3; i++)*/
    //{
        ////givenForce[i] = forceForTest[forceSelectionFlag * 3 + i];
    /*}*/

    givenForce[0] = 2 * machineData.forceData[0].forceValues[1] / 1000.0;
    givenForce[1] = 2 * machineData.forceData[0].forceValues[0] / 1000.0;
    givenForce[2] = 2 * machineData.forceData[0].forceValues[2] / 1000.0;
 
    for(int i = 0; i < 3; i++)
    {
        //givenForce[i] = forceForTest[forceSelectionFlag * 3 + i];
        if (fabs(givenForce[i]) < 10.0)
            givenForce[i] = 0;
        if (givenForce[i] > 180.0)
            givenForce[i] = 180.0;
        if (givenForce[i] < -180.0)
            givenForce[i] = -180.0;
    }

   
    gait.RunGait(timeNow, gaitcmd,machineData, givenForce);

    return 0;

};

//offsets driver order
static int HEXBOT_HOME_OFFSETS_RESOLVER[18] =
{
    -15849882 + 349000,	 -16354509 + 349000,	 -16354509 + 349000,
    -15849882 + 349000 + 100000,	 -16354509 + 349000 + 100000,	 -16354509 + 349000 + 100000, // 5 
    -15849882 + 349000,	 -16354509 + 349000,	 -16354509 + 349000,
    -16354509 + 349000,	 -15849882 + 349000,	 -16354509 + 349000,
    -15849882 + 349000,	 -16354509 + 349000,	 -16354509 + 349000,
    -16354509 + 349000,	 -16354509 + 349000,  -15849882 + 349000,
};


int OnGetControlCommand(Aris::Core::MSG &msg)
{
    int CommandID;
    msg.Paste(&CommandID,sizeof(int));
    Aris::Core::MSG data;

    switch(CommandID)
    {
        case 1:
            data.SetMsgID(POWEROFF);
            controlSystem.NRT_PostMsg(data);
            break;
        case 2:
            data.SetMsgID(STOP);
            controlSystem.NRT_PostMsg(data);
            break;
        case 3:
            data.SetMsgID(ENABLE);
            controlSystem.NRT_PostMsg(data);
            break;
        case 4:
            data.SetMsgID(RUNNING);
            controlSystem.NRT_PostMsg(data);
            break;
        case 5:
            data.SetMsgID(GOHOME_1);
            controlSystem.NRT_PostMsg(data);
            break;
        case 6:
            data.SetMsgID(GOHOME_2);
            controlSystem.NRT_PostMsg(data);
            break;
        case 7:
            data.SetMsgID(HOME2START_1);
            controlSystem.NRT_PostMsg(data);
            break;
        case 8:
            data.SetMsgID(HOME2START_2);
            controlSystem.NRT_PostMsg(data);
            break;
        case 9:
            data.SetMsgID(FORWARD);
            controlSystem.NRT_PostMsg(data);
            break;
        case 10:
            data.SetMsgID(BACKWARD);
            controlSystem.NRT_PostMsg(data);
            break;
        case 11:
            data.SetMsgID(FAST_FORWARD);
            controlSystem.NRT_PostMsg(data);
            break;
        case 12:
            data.SetMsgID(FAST_BACKWARD);
            controlSystem.NRT_PostMsg(data);
            break;
        case 13:
            data.SetMsgID(LEGUP);
            controlSystem.NRT_PostMsg(data);
            break;
        case 14:
            data.SetMsgID(TURNLEFT);
            controlSystem.NRT_PostMsg(data);
            break;
        case 15:
            data.SetMsgID(TURNRIGHT);
            controlSystem.NRT_PostMsg(data);
            break;
        case 16:
            data.SetMsgID(ONLINEGAIT);
            controlSystem.NRT_PostMsg(data);
            break;
        case 17:
            data.SetMsgID(ONLINEBEGIN);
            controlSystem.NRT_PostMsg(data);
            break;
        case 18:
            data.SetMsgID(ONLINEEND);
            controlSystem.NRT_PostMsg(data);
            break;

        case 19:
            data.SetMsgID(SET_NO_FORCE);
            controlSystem.NRT_PostMsg(data);
            break;
        case 20:
            data.SetMsgID(SET_FORCE_1);
            controlSystem.NRT_PostMsg(data);
            break;
        case 21:
            data.SetMsgID(SET_FORCE_2);
            controlSystem.NRT_PostMsg(data);
            break;
        case 22:
            data.SetMsgID(SET_FORCE_3);
            controlSystem.NRT_PostMsg(data);
            break;
        case 23:
            data.SetMsgID(SET_FORCE_4);
            controlSystem.NRT_PostMsg(data);
            break;
        case 24:
            data.SetMsgID(SET_FORCE_5);
            controlSystem.NRT_PostMsg(data);
            break;
        case 25:
            data.SetMsgID(SET_FORCE_6);
            controlSystem.NRT_PostMsg(data);
            break;
        case 26:
            data.SetMsgID(SET_FORCE_7);
            controlSystem.NRT_PostMsg(data);
            break;
        case 27:
            data.SetMsgID(SET_FORCE_8);
            controlSystem.NRT_PostMsg(data);
            break;
        case 28:
            data.SetMsgID(CLEAR_FORCE);
            controlSystem.NRT_PostMsg(data);
            break;

        default:
            printf("Hi! I didn't get validate cmd\n");
            break;
    }
    return CommandID;

};

int main(int argc, char** argv)
{	
    Aris::Core::RegisterMsgCallback(CS_Connected,On_CS_Connected);
    Aris::Core::RegisterMsgCallback(CS_CMD_Received,On_CS_CMD_Received);
    Aris::Core::RegisterMsgCallback(CS_Lost,On_CS_Lost);
    Aris::Core::RegisterMsgCallback(GetControlCommand,OnGetControlCommand);

    //   CONN call back
    /*设置所有CONN类型的回调函数*/
    ControlSystem.SetCallBackOnReceivedConnection(On_CS_ConnectionReceived);

    ControlSystem.SetCallBackOnReceivedData(On_CS_DataReceived);

    ControlSystem.SetCallBackOnLoseConnection(On_CS_ConnectionLost);

    ControlSystem.StartServer("5690");
    Aris::Core::THREAD threadMessageLoop;
    threadMessageLoop.SetFunction(MessageLoopDaemon);
    threadMessageLoop.Start(0);

    controlSystem.SetSysInitializer(initFun);

    controlSystem.SetTrajectoryGenerator(tg);

    //controlSystem.SetModeCycVel();

    initParam.motorNum      = 18;
    initParam.homeHighSpeed = 280000;
    initParam.homeLowSpeed  = 80000;
    initParam.homeMode      = -1;

    ////necessary steps
    initParam.homeOffsets=HEXBOT_HOME_OFFSETS_RESOLVER;
    controlSystem.SysInit(initParam);

    controlSystem.SysInitCommunication();

    controlSystem.SysStart();

    printf("Will start\n");
    while(!controlSystem.IsSysStopped())
    {
        count++;
        sleep(1);
    }

    return 0;
};


