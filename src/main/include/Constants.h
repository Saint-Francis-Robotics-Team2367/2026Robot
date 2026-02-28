// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cmath>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */


 //refer with [namespace]::[varname]

namespace ModuleConstants {
    static constexpr double kDriveGearRatio = 5.357142857142857; // L3+
    static constexpr double kSteerGearRatio = 21.428571428571427; // 150/7:1
    static constexpr double kWheelRadius = 2.0; // inches
    static constexpr double kWheelDiameter = 4.0; // inches
    static constexpr double moduleMaxMPS = 5.2154328; //meters per second
    static constexpr double moduleMaxRot = 2.0; // rad
}

namespace MathConstants {
    static constexpr double TWO_PI = M_PI * 2;
}

namespace HardwareIDs { 
    static constexpr int FLsteerID = 1;
    static constexpr int FLdriveID = 2;
    static constexpr int FLencoderID = 9;
    
    static constexpr int FRsteerID = 3;
    static constexpr int FRdriveID = 4;
    static constexpr int FRencoderID = 10;

    static constexpr int BLsteerID = 7;
    static constexpr int BLdriveID = 8;
    static constexpr int BLencoderID = 12;

    static constexpr int BRsteerID = 5;
    static constexpr int BRdriveID = 6;
    static constexpr int BRencoderID = 11;

    static constexpr int pigeonID = 0;
}

namespace ControllerConstants {
    static constexpr double deadband = 0.05; //prevents joystick drift
    static constexpr double slewRate = 5.2;
}

namespace ShooterConstants {
    static constexpr double SHOOTERWHEELDIAMETER = 0.1; // in meters
    static constexpr double GRAVITY = 9.81;
    static constexpr double PI = M_PI;
    static constexpr double SHOOTEREFFICIENCY = 0.75;
    static constexpr double MeterConversionFactor = 0.0254f; // inches to meters

    // CAN IDs
    static constexpr int ShooterID = 3.0; // Wheel
    static constexpr int RackMotorID = 0.0; // Kraken
    static constexpr int RackEncoderID = 2.0; // ThroughBore
    static constexpr int FeederID = 4.0; // Feeder

    // PID Constants Feeder
    static constexpr double FeederP = 0.0;
    static constexpr double FeederI = 0.0;
    static constexpr double FeederD = 0.0;
    static constexpr double FeederV = 0.0;

    // PID Constants Shooter
    static constexpr double FlywheelP = 0.05;
    static constexpr double FlywheelI = 0.0;
    static constexpr double FlywheelD = 0.0;
    static constexpr double FlywheelV = 0.01;
    
    // PID Constants Rack
    static constexpr double RackP = 0.025;
    static constexpr double RackI = 0.0;
    static constexpr double RackD = 0.001;
    static constexpr double RackG = 0.01;
}