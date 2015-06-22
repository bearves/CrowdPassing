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
    PushRecoveryPlanner planner;
    planner.Initialize();

    planner.GetInitialJointLength(screwLength);

    for(auto len : screwLength){
        cout << len << "  ";
    }
    cout << endl;

    planner.Start(0.0);
    cout << "--------------------------------------"<<endl;
    for ( int step = 0; step < 10000; step++)
    {
        double timeNow = step*0.001;
        ApplyForce(timeNow, fext);
        planner.GenerateJointTrajectory(timeNow, fext, screwLength);
        trjfile << timeNow << "  ";
        for (auto len : screwLength)
        {
            trjfile<< len << "  ";
        }
        trjfile << endl;
    }
        
    planner.Stop();

    trjfile.flush();
    trjfile.close();

    return 0;
}

void ApplyForce( double timeNow, double* fxt )
{
    if (timeNow > 0 && timeNow < 4.4)
    {
        fxt[0] = 120;
    }
    else
    {
        fxt[0] = 0;
    }
}
