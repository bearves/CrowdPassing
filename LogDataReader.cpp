#include "Aris_ControlData.h"
#include "CrowdPassingPlanner.h"
#include <fstream>
#include <iostream>
#include <errno.h>
#include <string.h>

using namespace std;
using namespace Aris::RT_CONTROL;

struct ControllerData
{
    double forceRaw[6];
    double forceFiltered[6];
    CrowdPassingPlanner::InternalData internalData;
};

int main(int argc, char** argv)
{
    CMachineData data;
    ControllerData controllerData;

    if (argc < 2)
    {
        cout << "Wrong parameter" << endl;
        cout << "Usage: LogDataRead [PATH_TO_LOGFILE]" << endl;
        return -200;
    }

    cout << "Opening log file:" << argv[1] << endl;
    cout << "Data size:" << sizeof(data) << endl;

    ifstream fin(argv[1]);
    ofstream fout("ParsedFile.txt");

    if (fin.fail())
    {
        cout << "Open file error: " << strerror(errno) << endl;
        return errno;
    }
    while( fin.read((char *)&data, sizeof(data)))
    {
        memcpy(&controllerData, data.controlData, sizeof(controllerData));
        fout << data.time << "  ";
        for(int i = 0; i < 6; i++)
        {
            fout << data.forceData[0].forceValues[i] << "  ";
        }

        for(int i = 0; i < 18; i++)
        {
            fout << data.feedbackData[i].Position << "  ";
        }

        for(int i = 0; i < 18; i++)
        {
            fout << data.commandData[i].Position << "  ";
        }

        for(int i = 0; i < 6; i++)
        {
            fout << controllerData.forceRaw[i] << "  ";
        }
        for(int i = 0; i < 6; i++)
        {
            fout << controllerData.forceFiltered[i] << "  ";
        }

        for(int i = 0; i < 6; i++)
        {
            fout << controllerData.internalData.svRobotD[i] << "  ";
        }

        for (int i = 0; i < 17; ++i) 
        {
            fout << controllerData.internalData.svLegD[i] << "  ";
        }
        fout << controllerData.internalData.svLegD[17] << endl;
        
    }

    fin.close();
    fout.flush();
    fout.close();

    cout << "Log file reading finishing" << endl;

    return 0;
}
