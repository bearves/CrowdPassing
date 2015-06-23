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
    ONLINEEND     = 1018
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
    gait.InitGait(param);
    return 0;
};

int tg(Aris::RT_CONTROL::CMachineData& machineData,Aris::RT_CONTROL::RT_MSG& msg)
{
    int CommandID;

    rtCycleCounter++;

    double timeNow = rtCycleCounter * 0.001;

    CommandID=msg.GetMsgID();
    switch(CommandID)
    {
        case Aris::RT_CONTROL::RT_MSG::INVALID_MSG_ID:
            break;
        case NOCMD:
            for(int i=0;i<18;i++)
            {
                machineData.motorsCommands[i]=EMCMD_NONE;
            }
            rt_printf("NONE Command Get in NRT\n" );
            break;

        case ENABLE:
            gait.onlinePlanner.Offline();
            for(int i=0;i<18;i++)
            {
                machineData.motorsCommands[i]=EMCMD_ENABLE;
            }
            rt_printf("ENABLE Command Get in NRT\n" );

            break;
        case POWEROFF:
            gait.onlinePlanner.Offline();
            for(int i=0;i<18;i++)
            {
                machineData.motorsCommands[i]=EMCMD_POWEROFF;
            }
            rt_printf("POWEROFF Command Get in NRT\n" );

            break;
        case STOP:
            gait.onlinePlanner.Offline();
            for(int i=0;i<18;i++)
            {
                machineData.motorsCommands[i]=EMCMD_STOP;
            }
            rt_printf("STOP Command Get in NRT\n" );

            break;
        case RUNNING:
            gait.onlinePlanner.Offline();
            for(int i=0;i<18;i++)
            {
                machineData.motorsCommands[i]=EMCMD_RUNNING;
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

            gait.onlinePlanner.Initialize();
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

        default:
            //DO NOTHING, CMD AND TRAJ WILL KEEP STILL
            break;
    }

    gait.RunGait(timeNow, gaitcmd,machineData);

    return 0;

};

//offsets driver order
static int HEXBOT_HOME_OFFSETS_RESOLVER[18] =
{
    -15849882 + 349000,	 -16354509 + 349000,	 -16354509 + 349000,
    -15849882 + 349000,	 -16354509 + 349000,	 -16354509 + 349000,
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

    initParam.motorNum=18;
    initParam.homeHighSpeed=280000;
    initParam.homeLowSpeed=40000;

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


