#include "VirtualBipedPlanner.h"
#include <iostream>

VirtualBipedPlanner::VirtualBipedPlanner()
{
}

VirtualBipedPlanner::~VirtualBipedPlanner()
{
}

int VirtualBipedPlanner::Initialize()
{
    th        = 0.001;    // s
    mass      = 260;      // kg
    height    = 0.85;     // meter
    gravity   = 9.81;     // m/s^2
    damp      = 1600;     // N/(m/s) 
    kspr      = 100;      // N/m

    halfperiod= 1.6;      // second
    alpha     = 0.4;       
    beta      = 1;      
    stepheight= 0.04;     // meter
    kpsw      = 80;  
    kdsw      = 20;  

    u1        = 0.05;
    u2        = 0.1;
    w1        = 0.85;
    w2        = 0.9;

    acclimitStance = 1.0; // m/s/s
    acclimitSwing  = 1.6; // m/s/s
    vellimit       = 2;   // m/s
    poslimit       = 0.2; // m

    lastydot = 0;
    thisydot = 0;

    lasty = 0;
    thisy = 0;

    lastyswdot = 0;
    thisyswdot = 0;

    lastysw = 0;
    thisysw = 0;

    hsw = 0;

    timeRatio = 0;
    stepCount = 0;
    startTime = 0;
    lastTransitionTime = 0;

    gaitState = VGS_READY;

    return 0;
}

int VirtualBipedPlanner::Start( double timeNow )
{
    if ( gaitState == VGS_READY )
    {
        startTime = timeNow;
        lastTransitionTime = startTime;
        gaitState = VGS_STARTED;
    }
    return 0;
}

int VirtualBipedPlanner::RequireStop( double timeNow )
{
    if ( gaitState == VGS_STARTED )
    {
        requireStopTime = timeNow;
        stepLeft = STEP_TO_COMPLETELY_STOP;
        gaitState = VGS_STOPPING;
    }
    return 0;
}

int VirtualBipedPlanner::DoIteration( 
    /*IN*/double timeNow, 
    /*IN*/double * fext, 
    /*OUT*/double *pgrp, 
    /*OUT*/double *pgrpdot )
{
    bool isaStepComplete = false;

    // automatically switch to STOPPED state
    if ( gaitState == VGS_STOPPING)
    {
        fext[0] = 0;
    }

    if ( gaitState == VGS_READY )
    {
        thisy      = 0;
        thisydot   = 0;
        thisysw    = 0;
        thisyswdot = 0;
        hsw        = 0;
    }
    else if ( gaitState == VGS_STARTED || gaitState == VGS_STOPPING)
    {
        timeRatio = (timeNow - lastTransitionTime) / halfperiod;
        if(timeRatio > 1)
        {
            timeRatio -= 1;
            lastTransitionTime = timeNow;
            isaStepComplete = true;
        }

        // Discrete equation of spring-damped linear inverted pendulum
        yddot = (-damp * lastydot 
                - kspr * lasty
                + alpha * mass * gravity / height * lasty
                + fext[0]) / mass;
        yddot = Saturate(yddot, acclimitStance);

        thisydot = lastydot + th * yddot;
        thisy    = lasty    + th * thisydot;
        thisy    = Saturate(thisy, poslimit);

        // calculate the capture point
        this->GetSwingFootTarget();

        // PD tracking to move the swing foot to the capture point
        yswddot = -kpsw * (lastysw - ytarget) - kdsw * (lastyswdot - ytargetdot);
        yswddot = Saturate(yswddot, acclimitSwing);
        thisyswdot = lastyswdot + th * yswddot;
        thisysw = lastysw + th * thisyswdot;
        thisysw = Saturate(thisysw, poslimit);

        PlanningFootHeight();

        // State transition
        if (isaStepComplete)
        {
            this->StateTransition();
            stepLeft--;
            if ( gaitState == VGS_STOPPING && stepLeft <= 0)
                gaitState = VGS_STOPPED;
            stepCount++;
        }
    }
    else if ( gaitState == VGS_STOPPED )
    {
        // do nothing here, hold where it is
    }

    // Output of this virtual model
    this->AssignStateToCorrespondFoot();
    for (int i = 0; i < 2; i++)
    {
        pgrp[i * 3 + 0] = ygrp[i];
        pgrp[i * 3 + 1] = hgrp[i];
        pgrpdot[i * 3 + 0] = ygrpdot[i];
    }

    // State updating
    lastysw = thisysw;
    lastyswdot = thisyswdot;
    lasty = thisy;
    lastydot = thisydot;

    return 0;
}

double VirtualBipedPlanner::Saturate( double ainput, double limit )
{
    if (ainput > fabs(limit))
    {
        return fabs(limit);
    } 
    else if(ainput < -fabs(limit))
    {
        return -fabs(limit);
    }
    else
    {
        return ainput;
    }
}

int VirtualBipedPlanner::GetSwingFootTarget()
{
    // target due to different situations
    double ysw1 = lastysw + lastyswdot * th;
    double ysw1dot = lastyswdot;
    double ysw2 = beta * sqrt(height / gravity) * thisydot;
    double ysw2dot = beta * sqrt(height / gravity) * yddot;
    double ysw3 = lastysw - thisydot * th;
    double ysw3dot = -thisydot;
    double p = 0;
    // do blending
    if (timeRatio < u1)
    {
        ytarget = ysw1;
        ytargetdot = ysw1dot;
    }
    else if (timeRatio < u2)
    {
        p = (timeRatio - u1) / (u2 - u1);
        ytarget = ysw1 * (1-p) + ysw2 * p;
        ytargetdot = ysw1dot * (1-p) + ysw2dot * p;
    }
    else if (timeRatio < w1)
    {
        ytarget = ysw2;
        ytargetdot = ysw2dot;
    }
    else if (timeRatio < w2)
    {
        p = (timeRatio - w1) / (w2 - w1);
        ytarget = ysw2 * (1-p) + ysw3 * p;
        ytargetdot = ysw2dot * (1-p) + ysw3dot * p;
    }
    else
    {
        ytarget = ysw3;
        ytargetdot = ysw3dot;
    }
    return 0;
}

int VirtualBipedPlanner::StateTransition()
{
    double tmp = thisy;
    double tmpdot = thisydot;
    thisy = -thisysw;
    thisydot = -thisyswdot;
    thisysw = -tmp;
    thisyswdot = -tmpdot;
    return 0;
}

int VirtualBipedPlanner::PlanningFootHeight()
{
    double tacc = 0.5;
    double theta = 0;
    double pi = 3.14159265359;
    if (timeRatio < tacc)
    {
        theta = 2 * pi * (timeRatio / tacc * 0.5) * (timeRatio / tacc * 0.5);
    }
    else if (timeRatio > 1 - tacc)
    {
        theta = pi - 2 * pi * ((1 - timeRatio) / tacc * 0.5) * ((1 - timeRatio) / tacc * 0.5);
    }
    else
    {
        theta = pi / 2.0;
    }
    hsw = stepheight * sin(theta);
    return 0;
}

int VirtualBipedPlanner::AssignStateToCorrespondFoot()
{
    int stanceLeg = 0;
    int swingLeg = 1;
    if (stepCount % 2 == 1)
    {
        stanceLeg = 1;
        swingLeg = 0;
    }
    ygrp[stanceLeg] = -thisy;
    ygrpdot[stanceLeg] = -thisydot;
    hgrp[stanceLeg] = 0;
    ygrp[swingLeg] = thisysw;
    ygrpdot[swingLeg] = thisyswdot;
    hgrp[swingLeg] = hsw;

    return 0;
}

