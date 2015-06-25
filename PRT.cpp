#include <iostream>
#include <fstream>
#include "Hexapod_Robot.h"
#include "PushRecoveryPlanner.h"

void ApplyForce( double timeNow, double* fxt );

int main(int argc, char** argv)
{
    using namespace std;
    ofstream trjfile("TrjFile.txt");
    double fext[3]; 
    double screwLength[18];
    double legTipPositions[18];
    PushRecoveryPlanner planner;
    planner.Initialize();

    planner.GetInitialJointLength(screwLength);

    for(auto len : screwLength){
        cout << len << "  ";
    }
    cout << endl;

    cout << "--------------------------------------"<<endl;
    for ( int step = 0; step < 30000; step++)
    {
        double timeNow = step*0.001;

        if ( step == 3200 )
        {
            planner.Start(timeNow);
        }
        if ( step == 13000 )
        {
            planner.Stop(timeNow);
        }
        if ( step == 15000 )
        {
            planner.Start(timeNow);
        }
        ApplyForce(timeNow, fext);
        planner.GenerateJointTrajectory(timeNow, fext, screwLength);
        planner.GetForwardLegPositions(screwLength, legTipPositions);
        trjfile << timeNow << "  ";
        for (auto len : legTipPositions)
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
    if (timeNow > 7.5 && timeNow < 20.5)
    {
        fxt[0] = 0;
        fxt[1] = 250;
    }
    else
    {
        fxt[0] = 0;
        fxt[1] = 0;
    }
}
