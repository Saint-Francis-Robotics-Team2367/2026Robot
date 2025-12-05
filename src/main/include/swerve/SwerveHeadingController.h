#pragma once
#include <Constants.h>
#include "geometry/Pose2d.h"
#include <cmath>
#include <stdexcept>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/PIDController.h>


class SwerveHeadingController {
public:
    frc::PIDController mPIDCtr {0, 0, 0};
    double mSetpoint = 0.0;
    double outputMax;
    double outputMin;

    Rotation2d desiredHeading;

public:

    enum HeadingControllerState {
        OFF, SNAP, ALIGN, MAINTAIN
    };

    HeadingControllerState mHeadingControllerState = OFF;
    
    SwerveHeadingController(double output_min = -1.0, double output_max = 1.0) {
        mPIDCtr.EnableContinuousInput(0, 360);
        outputMax = output_max;
        outputMin = output_min;
    }

    void setOutputRange(double min, double max) {
        outputMax = max;
        outputMin = min;
    }

    HeadingControllerState getHeadingControllerState() {
        return mHeadingControllerState;
    }

    void setSetpoint(double inp) 
    {
        mSetpoint = inp;
    }

    void setFieldSetpoint(int POV) 
    {
        if (POV >= 0) {
            if (POV == 0) 
            {
                mSetpoint = 45;
            } else if (POV == 180)
            {
                mSetpoint = 0;
            } else if (POV == 270) 
            {
                mSetpoint = 215;
            } else if (POV == 90) {
                mSetpoint = 90;
            }
        }

    }

    void setSetpointPOV(int POV) {
        if (POV >= 0) {
            mSetpoint = POV;
        }
    }

    void setHeadingControllerState(HeadingControllerState state) {
        mHeadingControllerState = state;
    }

    double normalizeAngle(double angle) {
        while (angle < 0) angle += 360;
        while (angle >= 360) angle -= 360;
        return angle;
    }

    double calculate(double current_angle) {
        current_angle = normalizeAngle(current_angle);
        mSetpoint = normalizeAngle(mSetpoint);
        
        switch (mHeadingControllerState) {
            case OFF:
                return 0.0;
            case SNAP:
                mPIDCtr.SetPID(0.12, 0.0, 0.01);
                break;
            case ALIGN:
                mPIDCtr.SetPID(0.04, 0.0, 0.01);
                break;
            case MAINTAIN:
                mPIDCtr.SetPID(0.02, 0.0, 0.0);
                break;
        }
        return std::clamp(mPIDCtr.Calculate(current_angle, mSetpoint), outputMin, outputMax);
    }
};