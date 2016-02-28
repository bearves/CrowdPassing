#include <iostream>
#include <fstream>
#include "Hexapod_Robot.h"
#include "CrowdPassingPlanner.h"
#include "LowpassFilter.h"

using namespace std;

template <class T>
void ClearDataArea( T* data, int length );

void ApplyForce( double timeNow, double* fxt );

int main()
{
    ofstream trj("Trj.txt");
    CrowdPassingPlanner planner;
    LowpassFilter<6u> filter;
    Hexapod_Robot::ROBOT robot;
    double fxt[6];
    double fxtFiltered[6];
    double initialBodyPosition[6];
    double legPositionList[18];
    double jointLength[18];

    robot.LoadXML("/usr/Robots/resource/HexapodIII/HexapodIII.xml");

    planner.Initialize();
    filter.SetCutFrequency(0.03, 1000);
    filter.Initialize();

    ClearDataArea(initialBodyPosition, 6);
    ClearDataArea(jointLength, 6);
    ClearDataArea(fxt, 6);

    for (int step = 0; step < 20000; step++)
    {
        double timeNow = step*0.001;
        ClearDataArea<double>(legPositionList, 6);

        if (step == 1200)
        {
            planner.Start(timeNow);
        }
        ApplyForce(timeNow, fxt);
        filter.DoFilter(fxt, fxtFiltered);

        planner.DoIteration(timeNow, fxtFiltered, legPositionList);
        robot.SetPee(legPositionList, initialBodyPosition, "G");
        robot.GetPin(jointLength);
        auto internalData = planner.GetInternalData();

        if (step % 10 == 0){

            trj << setw(10) << fixed << setprecision(5) << timeNow << "  ";
            for (int i = 0; i < 18; i++)
            {
                trj << setw(10) << fixed << setprecision(5) << legPositionList[i] << "  ";
            }

            for (int i = 0; i < 18; i++)
            {
                trj << setw(10) << fixed << setprecision(5) << jointLength[i] << "  ";
            }

            for (int i = 0; i < 6; i++)
            {
                trj << setw(10) << fixed << setprecision(5) << internalData.svRobotD[i] << "  ";
            }
            for (int i = 0; i < 16; i++)
            {
                trj << setw(10) << fixed << setprecision(5) << internalData.svLegD[i] << "  ";
            }
            trj << setw(10) << fixed << setprecision(5) << fxtFiltered[1] << "  ";
            trj << setw(10) << fixed << setprecision(5) << fxtFiltered[5] << "  ";
            
            trj << endl;
        }

    }
    trj.flush();
    trj.close();
    return 0;
}

template <class T>
void ClearDataArea( T* data, int length ) 
{
    for (int i = 0; i < length; i++)
    {
        data[i] = 0;
    }
}

void ApplyForce( double timeNow, double* fxt )
{
    if (timeNow > 3.6 && timeNow < 4)
    {
        fxt[1] = 60;
        fxt[5] = 20;
    }
    else
    {
        fxt[1] = 0;
        fxt[5] = 0;
    }
}
