#ifndef RETREAT_GAIT_H
#define RETREAT_GAIT_H

#include <cmath>
#include "rtdk.h"

class RetreatGait
{
public:

    enum RETREAT_GAIT_STATE
    {
        RGS_READY    = 1,
        RGS_WAITING  = 2,
        RGS_INMOTION = 3,
        RGS_STOPPED  = 4
    };

    enum ACTION
    {
        NO_ACTION              = 0,
        GO_FORWARD             = 1,
        GO_BACKWARD            = 2,
        BODY_DOWN              = 3,
        BODY_RETREAT_TO_LEFT   = 4,
        BODY_RETREAT_TO_RIGHT  = 5,
        BODY_RETREAT_TO_FRONT  = 6,
        BODY_RETREAT_TO_BEHIND = 7
    };

    RetreatGait();
    ~RetreatGait();

    int Initialize();
    int Start(double timeNow);
    int RequireStop(double timeNow);
    int DoPlanning(/*IN*/double timeNow, /*IN*/double * fext, /*OUT*/double *feetPos, /*OUT*/double *bodyPos);
    
    RETREAT_GAIT_STATE GetState() const { return gaitState;};
    ACTION GetAction() const { return currentAction;}

private:
    // State variables
    static const double timeOfAction[8];

    RETREAT_GAIT_STATE gaitState;
    ACTION currentAction; 

    bool isStopRequired;

    double timeWhenEnterAction;

    double initialBodyPosition[6];
    double currentBodyPosition[6];
    double initialFeetPosition[6]; // three legs as a group
    double currentFeetPosition[6]; // three legs as a group

    bool DetermineAction(double *fext, ACTION& actionToDo);
    void ActionPlanning(/*IN*/double timeNow, /*IN*/double * fext, /*OUT*/double *feetPos, /*OUT*/double *bodyPos);
};

#endif
