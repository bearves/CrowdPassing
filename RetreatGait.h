#ifndef RETREAT_GAIT_H
#define RETREAT_GAIT_H

#include <cmath>
#include <fstream>
#include <iostream>
#include <string.h>
#include <errno.h>

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
    int LoadData();
    int Start(double timeNow);
    int RequireStop(double timeNow);
    int DoPlanning(/*IN*/double timeNow, /*IN*/double * fext, /*OUT*/double *feetPos, /*OUT*/double *bodyPos);
    
    RETREAT_GAIT_STATE GetState() const { return gaitState;};
    ACTION GetAction() const { return currentAction;}

private:
    // State variables
    static const double timeOfAction[8];
    static const int FORWARD_GAIT_LENGTH = 3401; 
    static const int SIDE_WEBB_GAIT_LENGTH = 5000;
    static const int BODY_DOWN_GAIT_LENGTH = 3500;
    static const double timeInterval;

    RETREAT_GAIT_STATE gaitState;
    ACTION currentAction; 

    int innerCounter;

    bool isStopRequired;
    bool isActionFinished;

    int timeCountWhenEnterAction;

    double initialBodyPosition[6];
    double currentBodyPosition[6];
    double initialFeetPosition[6]; // three legs as a group
    double currentFeetPosition[6]; // three legs as a group

    double forwardGaitData[FORWARD_GAIT_LENGTH][12];
    double sideWebbGaitData[SIDE_WEBB_GAIT_LENGTH][12];
    double bodyDownData[BODY_DOWN_GAIT_LENGTH][12];
    
    int LoadEachData(const char* dataPath, int dataLength, double (* dataPlace)[12]);
    bool DetermineAction(double *fext, ACTION& actionToDo);
    void ActionPlanning(/*IN*/double timeNow, /*IN*/double * fext);
    void SwapLegGroup(double *legPosition);
};

#endif
