#include <iostream>
#include <fstream>
#include "VirtualBipedPlanner.h"

using namespace std;

template <class T>
void ClearDataArea( T* data, int length );

void ApplyForce( double timeNow, double* fxt );

int main()
{
    ofstream trj("Trj.txt");
    VirtualBipedPlanner planner;
    double fxt[2];
    double pgrp[6];
    double pgrpdot[6];
    double ax[0];
    planner.Initialize();
    planner.Start(0);
    for (int step = 0; step < 10000; step++)
    {
        double timeNow = step*0.001;
        ClearDataArea<double>(pgrp, 6);
        ApplyForce(timeNow, fxt);
        planner.DoIteration(timeNow, fxt, pgrp, pgrpdot);
        for (int i = 0; i < 6; i++)
        {
            trj << pgrp[i] << '\t';
        }
        trj << endl;
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
    if (timeNow > 3.6 && timeNow < 4.4)
    {
        fxt[0] = 230;
    }
    else
    {
        fxt[0] = 0;
    }
}
