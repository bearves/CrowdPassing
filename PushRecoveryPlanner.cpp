#include "PushRecoveryPlanner.h"

const double PushRecoveryPlanner::BASIC_BODY_POSITION[] = {0, 0, 0, 0, 0, 0};
const double PushRecoveryPlanner::BASIC_FEET_POSITION[] =
{
    -0.3,  -0.85, -0.65,
    -0.45, -0.85,  0.0,
    -0.3,  -0.85,  0.65,
    0.3,  -0.85, -0.65,
    0.45, -0.85,  0.0,
    0.3,  -0.85,  0.65
};


PushRecoveryPlanner::PushRecoveryPlanner(void)
{
    robot.LoadXML("/usr/Robots/resource/HexapodIII/HexapodIII.xml");

    for(int i = 0; i < 6; i++)
        initialBodyPosition[i] = BASIC_BODY_POSITION[i];
    
    for(int i = 0; i < 18; i++)
        initialFeetPosition[i] = BASIC_FEET_POSITION[i];

    olgaitState = OGS_OFFLINE;
}

PushRecoveryPlanner::~PushRecoveryPlanner(void)
{
}

int PushRecoveryPlanner::LoadData()
{
    return retreatGaitPlanner.LoadData();
}

int PushRecoveryPlanner::Initialize(int gaitMod)
{
    if ( olgaitState == OGS_OFFLINE && gaitMod == 1){
        virtualPlanner.Initialize();
        olgaitState = OGS_ONLINE_DRAG;
    }
    else if ( olgaitState == OGS_OFFLINE && gaitMod == 2)
    {
        retreatGaitPlanner.Initialize();
        olgaitState = OGS_ONLINE_RETREAT;
    }
    return 0;
}

int PushRecoveryPlanner::Start(double timeNow)
{
    if ( olgaitState == OGS_ONLINE_DRAG ){
        virtualPlanner.Start(timeNow);
    }
    else if (olgaitState == OGS_ONLINE_RETREAT)
    {
        retreatGaitPlanner.Start(timeNow);
    }
    return 0;
}

int PushRecoveryPlanner::Stop(double timeNow)
{
    if ( olgaitState == OGS_ONLINE_DRAG )
    {
        virtualPlanner.RequireStop(timeNow);
    }
    else if ( olgaitState == OGS_ONLINE_RETREAT)
    {
        retreatGaitPlanner.RequireStop(timeNow);
    }
    return 0;
}

int PushRecoveryPlanner::Offline()
{
    if ( olgaitState == OGS_ONLINE_DRAG )
    {
        olgaitState = OGS_OFFLINE;
    }
    return 0;
}

int PushRecoveryPlanner::GetInitialJointLength(double jointLength[])
{
    if (jointLength == NULL)
        return -1;
    robot.SetPee(initialFeetPosition, initialBodyPosition, "G");
    robot.GetPin(jointLength);
    return 0;
}

int PushRecoveryPlanner::GenerateJointTrajectory(
        double timeNow,
        double externalForce[],
        double jointLength[])
{
    if ( olgaitState == OGS_OFFLINE)
        return -1; // This function should not be called when olgaitState == OGS_OFFLINE
    
    if ( olgaitState == OGS_ONLINE_DRAG)
    {
        virtualPlanner.DoIteration(timeNow, externalForce, legGroupPosition, legGroupPositionDot);
        this->CalculateEachLegPosition();

        //initialBodyPosition = { 0, 1/2, 0, 0, 0, 0};
        //s_ep2pm(initialBodyPosition, pm, "321");
        //s_pm2ep(pm, initialBodyPosition, "313");
        robot.SetPee(feetPosition, initialBodyPosition, "G");
        robot.GetPin(jointLength);
    }

    else if (olgaitState == OGS_ONLINE_RETREAT)
    {
        retreatGaitPlanner.DoPlanning(timeNow, externalForce, legGroupPosition, bodyPosition);
        this->CalculateEachLegPosition();
        robot.SetPee(feetPosition, bodyPosition, "G");
        robot.GetPin(jointLength);
    }
    return 0;
}


int PushRecoveryPlanner::GetForwardLegPositions(double jointLengthList[], double legTipPositions[])
{
    robot.SetPin(jointLengthList, initialBodyPosition);
    robot.GetPee( legTipPositions, "G");
    return 0;
}

int PushRecoveryPlanner::CalculateEachLegPosition()
{
    for(int i = 0; i < 18; i++)
    {
        feetPosition[i] = initialFeetPosition[i];
    }
    for(int i = 0; i < 2; i++)
    {
        for( int j = 0; j < 3; j++)
        {
            feetPosition[(j * 2 + i) * 3 + 0] += legGroupPosition[(1 - i) * 3 + 0]; // all X offset
            feetPosition[(j * 2 + i) * 3 + 1] += legGroupPosition[(1 - i) * 3 + 1]; // all h offset
            feetPosition[(j * 2 + i) * 3 + 2] += legGroupPosition[(1 - i) * 3 + 2]; // all Z offset
        }
    }
    return 0;
}

