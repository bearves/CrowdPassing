#ifndef VIRTUAL_BIPED_PLANNER_H
#define VIRTUAL_BIPED_PLANNER_H

#include <cmath>

class VirtualBipedPlanner
{
public:

    static const int STEP_TO_COMPLETELY_STOP = 8;

    enum VIRTUAL_GAIT_STATE
    {
        VGS_READY    = 1,
        VGS_STARTED  = 2,
        VGS_STOPPING = 3,
        VGS_STOPPED  = 4
    };

    VirtualBipedPlanner();
    ~VirtualBipedPlanner();

    int Initialize();
    int Start(double timeNow);
    int RequireStop(double timeNow);
    // One dimensional at this time
    int DoIteration(/*IN*/double timeNow, /*IN*/double * fext, /*OUT*/double *pgrp, /*OUT*/double *pgrpdot);
    
    VIRTUAL_GAIT_STATE GetState() const { return gaitState;};

private:
    // State variables
    double yddot;
    double lastydot;
    double thisydot;
    
    double lasty;
    double thisy;
    
    double yswddot;
    double lastyswdot;
    double thisyswdot;

    double lastysw;
    double thisysw;

    double ytarget;
    double ytargetdot;

    double hsw;

    double timeRatio;
    int stepCount;
    double startTime;
    double requireStopTime;
    double lastTransitionTime;

    // output variables
    double ygrp[2];
    double ygrpdot[2];
    double hgrp[2];

    // Physical parameters
    double th;        // time interval
    double mass;      // body mass
    double height;    // hip height
    double gravity;   // gravitational constant
    double damp;      // virtual damping
    double kspr;      // virtual string stiffness
    double halfperiod;// half gait period
    double alpha;     // coefficient of the gravity effect
    double beta;      // coefficient of the capture point
    double stepheight;// step height
    double kpsw;      // kp of swing leg tracking
    double kdsw;      // kd of swing leg tracking

    double u1;        // for calculate swing leg target point
    double u2;
    double w1;
    double w2;

    double acclimitStance;
    double acclimitSwing;
    double vellimit;
    double poslimit;

    int stepLeft;

    VIRTUAL_GAIT_STATE gaitState;

    double Saturate(double ainput, double limit);
    int GetSwingFootTarget();
    int StateTransition();
    int PlanningFootHeight();
    int AssignStateToCorrespondFoot();
};

#endif
