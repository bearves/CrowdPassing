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
}

PushRecoveryPlanner::~PushRecoveryPlanner(void)
{
}

int PushRecoveryPlanner::Initialize()
{
    virtualPlanner.Initialize();
    return 0;
}

int PushRecoveryPlanner::Start(double timeNow)
{
    return virtualPlanner.Start(timeNow);
}

int PushRecoveryPlanner::Stop()
{
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
    virtualPlanner.DoIteration(timeNow, externalForce, legGroupPosition, legGroupPositionDot);
    this->CalculateEachLegPosition();
    
    robot.SetPee(feetPosition, initialBodyPosition, "G");
    robot.GetPin(jointLength);
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
            feetPosition[(j * 2 + i) * 3 + 0] += legGroupPosition[i * 3 + 0]; // all X offset
            feetPosition[(j * 2 + i) * 3 + 1] += legGroupPosition[i * 3 + 1]; // all h offset
            feetPosition[(j * 2 + i) * 3 + 2] += legGroupPosition[i * 3 + 2]; // all Z offset
        }
    }
    return 0;
}

