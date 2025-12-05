#pragma once

#include "ctre/phoenix6/Pigeon2.hpp"
#include "geometry/Rotation2d.h"

class Pigeon 
{
private:
    ctre::phoenix6::configs::Pigeon2Configuration currentConfiguration{};

public:
    ctre::phoenix6::hardware::Pigeon2 pigeon;

    Pigeon(int deviceID) : pigeon(ctre::phoenix6::hardware::Pigeon2(deviceID, "rio"))
    {
    }

    ctre::phoenix6::configs::Pigeon2Configuration getConfig()
    {
        return currentConfiguration;
    }

    void init() {
        ctre::phoenix6::configs::Pigeon2Configuration pigeonConfigs{};

        pigeonConfigs.MountPose.MountPosePitch = units::degree_t(0.0);
        pigeonConfigs.MountPose.MountPoseRoll = units::degree_t(0.0);
        pigeonConfigs.MountPose.MountPoseYaw = units::degree_t(0.0);

        pigeonConfigs.Pigeon2Features.EnableCompass = true;
        pigeonConfigs.FutureProofConfigs = false;

        pigeon.GetConfigurator().Apply(pigeonConfigs);
        currentConfiguration = pigeonConfigs;

        pigeon.Reset();
        pigeon.ClearStickyFaults();
    }

    /**
     * In Radians
     * GetYaw(): CCW+
     * GetAngle(): CW+
     */
    Rotation2d getBoundedAngleCCW()
    {
        return Rotation2d(Rotation2d::degreesBound(fmod(pigeon.GetYaw().GetValueAsDouble(), 360.0)) * PI / 180);
    }

    Rotation2d getBoundedAngleCW()
    {
        return Rotation2d(Rotation2d::degreesBound(-fmod(pigeon.GetYaw().GetValueAsDouble(), 360.0)) * PI / 180);
    }

    frc::Rotation2d getRotation2d()
    {
        return frc::Rotation2d(pigeon.GetRotation2d() * PI / 180);
    }
};