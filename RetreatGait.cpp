#include "RetreatGait.h"

using namespace std;

const double RetreatGait::timeOfAction[] = 
     { 0.02, 3.401, 3.401, 3.5, 5, 5, 5, 5};

const double RetreatGait::timeInterval = 0.001;

RetreatGait::RetreatGait()
{
    innerCounter = 0;
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

    innerCounter = 0;
    timeCountWhenEnterAction = 0;    

    currentAction = NO_ACTION;
    gaitState = RGS_READY;
    isStopRequired = false;
    isActionFinished = true;
    return 0;
}

int RetreatGait::LoadData()
{
    int flag;
    /* gait forward */
    flag = LoadEachData("../../resource/gait/forward_body_feet.txt", FORWARD_GAIT_LENGTH, forwardGaitData);
    if (flag != 0)
        goto OPEN_FILE_FAIL;

    flag = LoadEachData("../../resource/gait/side_body_feet_with_step.txt", SIDE_WEBB_GAIT_LENGTH,  sideWebbGaitData);
    if (flag != 0)
        goto OPEN_FILE_FAIL;

    flag = LoadEachData("../../resource/gait/body_down_body_feet.txt", BODY_DOWN_GAIT_LENGTH,  bodyDownData);
    if (flag != 0)
        goto OPEN_FILE_FAIL;

    return 0;

OPEN_FILE_FAIL:
    cerr << "Retreat Gait: Open file error: " << strerror(flag) << endl;
    return errno;
}

int RetreatGait::LoadEachData(const char* dataPath, int dataLength, double (* dataPlace)[12])
{
    std::fstream fin;
    double temp;
    /* gait forward */
    fin.open(dataPath);
    if(fin.fail())
        return errno;

    for(int i = 0; i < dataLength; i++)
    {
        for(int j = 0; j < 12 ; j++)
        {
            fin>>temp;
            dataPlace[i][j] = temp;
        }

    }
    fin.close();
    return 0;
}

int RetreatGait::Start(double timeNow)
{
    if (gaitState == RGS_READY)
    {
        gaitState = RGS_WAITING;
        isStopRequired = false;
        currentAction = NO_ACTION;
        timeCountWhenEnterAction = innerCounter;
    }
    return 0;
}

int RetreatGait::RequireStop(double timeNow)
{
    isStopRequired = true;
    return 0;
}

// The gait generator
int RetreatGait::DoPlanning( 
        double timeNow, /*in*/
        double *fext,   /*in*/
        double *feetPos,/*out*/  
        double *bodyPos/*out*/)
{
    innerCounter++;
    switch (gaitState)
    {
        case RGS_READY:
            break;
        case RGS_WAITING:
            // if a stop is required, quit to STOPPED
            if (isStopRequired)
            {
                gaitState = RGS_STOPPED;
            }
            // Determine if an action should be taken
            if (this->DetermineAction(fext, currentAction))
            {
                gaitState = RGS_INMOTION;
                timeCountWhenEnterAction = innerCounter;
                isActionFinished = false;
            }
            break;
        case RGS_INMOTION:
            this->ActionPlanning(timeNow, fext);
            if (isActionFinished)
            {
                gaitState = RGS_WAITING;
                currentAction = NO_ACTION;
            }
            break;
        case RGS_STOPPED:
            break;
    }
    for (int i = 0; i < 6; i++)
    { // Stay at the current position
        feetPos[i]  = currentFeetPosition[i];
        bodyPos[i] = currentBodyPosition[i];
    }
    return 0;
}

bool RetreatGait::DetermineAction(double *fext, RetreatGait::ACTION& actionToDo)
{
    actionToDo = NO_ACTION;
    // force mainly towards front direction
    if (fext[1] > 160 && fabs(fext[2]) < 30 && fabs(fext[0]) < 40)
    {
        actionToDo = GO_FORWARD;
        return true;
    } 
    // force mainly towards back direction
    else if (fext[1] < -160 && fabs(fext[2]) < 30 && fabs(fext[0]) < 40)
    {
        actionToDo = GO_BACKWARD;
        return true;
    }
    // force mainly downwards, no too much torque
    else if (fext[2] < -160 && 
            fabs(fext[3]) < 10 && 
            fabs(fext[4]) < 10)
    {
        actionToDo = BODY_DOWN;
        return true;
    } 
    else if (fext[0] > 160 && fabs(fext[1]) < 40)
    {
        actionToDo = BODY_RETREAT_TO_RIGHT;
        return true;
    }
    else if (fext[0] < -160 && fabs(fext[1]) < 40)
    {
       actionToDo = BODY_RETREAT_TO_LEFT;
       return true;
    }
    return false;
};


void RetreatGait::ActionPlanning(/*IN*/double timeNow, /*IN*/double * fext)
{
    int index = 0;
    switch (currentAction)
    {
        case NO_ACTION:
            break;
        case GO_FORWARD:
            index = innerCounter - timeCountWhenEnterAction;
            //cout << "Action: forward " << index <<  endl;
            if (index >= 0 && index < FORWARD_GAIT_LENGTH)
            {
                for (int i = 0; i < 6; i++)
                {
                    currentBodyPosition[i] = forwardGaitData[index][i];
                    currentFeetPosition[i] = forwardGaitData[index][6 + i];
                }
            }
            else
            {
                isActionFinished = true;
            }
            break;
        case GO_BACKWARD:
            index = innerCounter - timeCountWhenEnterAction;
            //cout << "Action: backword " << index <<  endl;
            if (index >= 0 && index < FORWARD_GAIT_LENGTH)
            {
                for (int i = 0; i < 6; i++)
                {
                    currentBodyPosition[i] = forwardGaitData[index][i];
                    currentFeetPosition[i] = forwardGaitData[index][6 + i];
                }
                // reverse z direction (front to back)
                currentBodyPosition[5] = -currentBodyPosition[5]; 
                // reverse x direction of legs
                currentFeetPosition[2] = -currentFeetPosition[2];
                currentFeetPosition[5] = -currentFeetPosition[5];
            }
            else
            {
                isActionFinished = true;
            }
            break;
        case BODY_RETREAT_TO_RIGHT:
            index = innerCounter - timeCountWhenEnterAction;

            //cout << "Action: left " << index <<  endl;
            if (index >= 0 && index < SIDE_WEBB_GAIT_LENGTH)
            {
                for (int i = 0; i < 6; i++)
                {
                    currentBodyPosition[i] = sideWebbGaitData[index][i];
                    currentFeetPosition[i] = sideWebbGaitData[index][6 + i];
                }
            }
            else
            {
                isActionFinished = true;
            }
            break;
        case BODY_RETREAT_TO_LEFT:
            index = innerCounter - timeCountWhenEnterAction;
            //cout << "Action: right " << timeWhenEnterAction << endl;
            if (index >= 0 && index < SIDE_WEBB_GAIT_LENGTH)
            {
                for (int i = 0; i < 6; i++)
                {
                    currentBodyPosition[i] = sideWebbGaitData[index][i];
                    currentFeetPosition[i] = sideWebbGaitData[index][6 + i];
                }
                // reverse x direction (left to right)
                currentBodyPosition[3] = -currentBodyPosition[3]; 
                currentBodyPosition[0] = -currentBodyPosition[0];  // reverse the roll angle
                // reverse x direction of legs 
                currentFeetPosition[0] = -currentFeetPosition[0];
                currentFeetPosition[3] = -currentFeetPosition[3];
                // swap leg group1 and group2 
                SwapLegGroup(currentFeetPosition);
            }
            else
            {
                isActionFinished = true;
            }
            break;
        case BODY_DOWN:
            index = innerCounter - timeCountWhenEnterAction;

            //cout << "Action: bodydown " << index <<  endl;
            if (index >= 0 && index < BODY_DOWN_GAIT_LENGTH)
            {
                for (int i = 0; i < 6; i++)
                {
                    currentBodyPosition[i] = bodyDownData[index][i];
                    currentFeetPosition[i] = bodyDownData[index][6 + i];
                }
            }
            else
            {
                isActionFinished = true;
            }
            break;
        default:
            break; 
    }
}

void RetreatGait::SwapLegGroup(double *legPosition)
{
    double temp;
    for ( int i = 0; i < 3; i++ )
    {
        temp = legPosition[i];
        legPosition[i] = legPosition[i + 3];
        legPosition[i + 3] = temp;
    }
}








