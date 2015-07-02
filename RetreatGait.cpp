#include "RetreatGait.h"

using namespace std;

const double RetreatGait::timeOfAction[] = 
     { 0.02, 3.6, 3.6, 5, 7, 7, 7, 7};

RetreatGait::RetreatGait()
{
}

RetreatGait::~RetreatGait()
{
}

int RetreatGait::Initialize()
{
    for(int i = 0; i < 6; i++)
    {
        initialBodyPosition[i] = 0;
        currentBodyPosition[i] = 0;
        initialFeetPosition[i] = 0;
        currentFeetPosition[i] = 0; 
    }

    timeWhenEnterAction = 0;

    currentAction = NO_ACTION;
    gaitState = RGS_READY;
    isStopRequired = false;
    return 0;
}

int RetreatGait::Start(double timeNow)
{
    if (gaitState == RGS_READY)
    {
        gaitState = RGS_WAITING;
        isStopRequired = false;
        currentAction = NO_ACTION;
        timeWhenEnterAction = timeNow;
    }
    return 0;
}

int RetreatGait::RequireStop(double timeNow)
{
    isStopRequired = true;
    return 0;
}

int RetreatGait::DoPlanning( 
        double timeNow, /*in*/
        double *fext,   /*in*/
        double *feetPos,/*out*/  
        double *bodyPos/*out*/)
{
    
    for (int i = 0; i < 6; i++)
    { // Stay at the current position
        feetPos[i]  = currentFeetPosition[i];
        bodyPos[i] = currentBodyPosition[i];
    }
    switch (gaitState)
    {
        case RGS_READY:
            for (int i = 0; i < 6; i++)
            { // Stay at the initial position
                feetPos[i] = initialFeetPosition[i];
                bodyPos[i] = initialBodyPosition[i];
            }
            break;
        case RGS_WAITING:
            for (int i = 0; i < 6; i++)
            { // Stay at the current position
                feetPos[i]  = currentFeetPosition[i];
                bodyPos[i] = currentBodyPosition[i];
            }
            // if a stop is required, quit to STOPPED
            if (isStopRequired)
            {
                gaitState = RGS_STOPPED;
            }
            // Determine if an action should be taken
            if (this->DetermineAction(fext, currentAction))
            {
                gaitState = RGS_INMOTION;
                timeWhenEnterAction = timeNow;
            }
            break;
        case RGS_INMOTION:
            this->ActionPlanning(timeNow, fext, feetPos, bodyPos);
            if (timeNow > timeWhenEnterAction + timeOfAction[currentAction])
            {
                gaitState = RGS_WAITING;
                currentAction = NO_ACTION;
            }
            break;
        case RGS_STOPPED:
            for (int i = 0; i < 6; i++)
            { // Stay at the current position
                feetPos[i]  = currentFeetPosition[i];
                bodyPos[i] = currentBodyPosition[i];
            }
            break;
    }
    if (fabs(fmod(timeNow, 0.1)) < 2e-3)
    {
        rt_printf("State: %d   Action: %d\n", gaitState, currentAction);
    }
    return 0;
}

bool RetreatGait::DetermineAction(double *fext, RetreatGait::ACTION& actionToDo)
{
    actionToDo = NO_ACTION;
    // force mainly towards front direction
    if (fext[1] > 100 && fabs(fext[2]) < 30) 
    {
        actionToDo = GO_FORWARD;
        return true;
    } 
    // force mainly towards back direction
    else if (fext[1] < -100)
    {
        actionToDo = GO_BACKWARD;
        return true;
    }
    // force mainly downwards, no too much torque
    else if (fext[2] < -100 && 
            fabs(fext[3]) < 10 && 
            fabs(fext[4]) < 10)
    {
        actionToDo = BODY_DOWN;
        return true;
    } 
    else if (fext[0] > 100)
    {
        actionToDo = BODY_RETREAT_TO_RIGHT;
        return true;
    }
    else if (fext[0] < -100)
    {
       actionToDo = BODY_RETREAT_TO_LEFT;
       return true;
    }
    return false;
};


void RetreatGait::ActionPlanning(/*IN*/double timeNow, /*IN*/double * fext, /*OUT*/double *feetPos, /*OUT*/double *bodyPos)
{
    for (int i = 0; i < 6; i++)
    { // Stay at the current position
        feetPos[i]  = currentFeetPosition[i];
        bodyPos[i] = currentBodyPosition[i];
    }
}










