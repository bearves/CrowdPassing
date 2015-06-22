#ifndef PUSH_RECOVERY_PLANNER_H
#define PUSH_RECOVERY_PLANNER_H

#include "Hexapod_Robot.h"
#include "VirtualBipedPlanner.h"

// The planner to generate the joint trajectory for push recovery
class PushRecoveryPlanner
{
public:
    PushRecoveryPlanner(void);
    ~PushRecoveryPlanner(void);

    // Intialize the planner, should be called when receiving the GAIT_START message
    int Initialize();
    // Start the planner
    int Start(double timeNow);
    // Stop the planner, it cannot be start again
    int Stop();

    // Generate the joint trajectory due to the external force, should be called in each cycle.
    int GenerateJointTrajectory(double timeNow, double externalForce[], double jointLengthList[]);
    // Get the joint length of initial position, used for go to initial position before starting
    int GetInitialJointLength(double jointLengthList[]);

private:
    static const double BASIC_FEET_POSITION[18];
    static const double BASIC_BODY_POSITION[6];

    Hexapod_Robot::ROBOT robot;
    VirtualBipedPlanner virtualPlanner;
 
    double initialFeetPosition[18];
    double initialBodyPosition[6];
    double legGroupPosition[6];
    double legGroupPositionDot[6];
    double feetPosition[18];
    double bodyPosition[18];

    // Assign the position of leg group from the virtual model to the position of acutal legs
    int CalculateEachLegPosition();
}; 
#endif
