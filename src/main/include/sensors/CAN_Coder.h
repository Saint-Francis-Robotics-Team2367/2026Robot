#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include "geometry/Rotation2d.h"

class CAN_Coder
{
private:
    int ID;

public:
    ctre::phoenix6::hardware::CANcoder encoder;
    CAN_Coder(int canID) : encoder(ctre::phoenix6::hardware::CANcoder(canID, "rio"))
    {
        ID = canID;
    }

    Rotation2d getPosition()
    {
        return Rotation2d(Rotation2d::radiansBound(encoder.GetPosition().GetValueAsDouble() * 2 * PI));
    }

    Rotation2d getAbsolutePosition()
    {
        return Rotation2d(Rotation2d::radiansBound(encoder.GetAbsolutePosition().GetValueAsDouble() * 2 * PI));
    }

    // double getVelocity()
    // {
    //     return encoder.GetVelocity();
    // }
    int getCANID()
    {
        return ID;
    }
};