#include <iostream>
#include <fstream>
#include "Hexapod_Robot.h"
#include "PushRecoveryPlanner.h"

void ApplyForce( double timeNow, double* fxt );

int main(int argc, char** argv)
{
    using namespace std;
    ofstream trjfile("TrjFile.txt");
    double fext[6]; 
    double screwLength[18];
    double legTipPositions[18];
    PushRecoveryPlanner planner;
    planner.LoadData();
    planner.Initialize(2);

    planner.GetInitialJointLength(screwLength);

    for(auto len : screwLength){
        cout << len << "  ";
    }
    cout << endl;

    cout << "--------------------------------------"<<endl;
    for ( int step = 0; step < 18000; step++)
    {
        double timeNow = step*0.001;

        if ( step == 0 )
        {
            planner.Start(timeNow);
        }
        if ( step == 14000 )
        {
            planner.Stop(timeNow);
        }
        ApplyForce(timeNow, fext);
        planner.GenerateJointTrajectory(timeNow, fext, screwLength);
        planner.GetForwardLegPositions(screwLength, legTipPositions);
        trjfile << timeNow << "  ";
        for (auto len : screwLength)
        {
            trjfile << std::setprecision(12) << len << "  ";
        }
        trjfile << endl;
    }
        

    trjfile.flush();
    trjfile.close();

    return 0;
}

void ApplyForce( double timeNow, double* fxt )
{
    if (timeNow > 0.6 && timeNow < 0.62)
    {
        fxt[0] = -120;
        fxt[1] = 0;
        fxt[2] = 0;
    }
    else if (timeNow > 7.1 && timeNow < 7.12)
    {
        fxt[0] = 0;
        fxt[1] = 0;
        fxt[2] = -200;
    }
    else if (timeNow > 12.1 && timeNow < 12.8)
    {
        fxt[0] = 0;
        fxt[1] = -120;
        fxt[2] = 0;
    }
    else
    {
        fxt[0] = 0;
        fxt[1] = 0;
        fxt[2] = 0;
    }
}
