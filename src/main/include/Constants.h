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

namespace ModuleConstants {
    static constexpr double kDriveGearRatio = 5.357142857142857; // L3+
    static constexpr double kSteerGearRatio = 21.428571428571427; // 150/7:1
    static constexpr double kWheelRadius = 2.0; // inches
    static constexpr double kWheelDiameter = 4.0; // inches
    static constexpr double moduleMaxMPS = 5.2154328;
    static constexpr double moduleMaxRot = 2.0; // rad
}

namespace MathConstants {
    static constexpr double TWO_PI = M_PI * 2;
}

namespace HardwareIDs { // Steer Motors are ID odd numbers, Drive Motors are ID even numbers, Ex. Module 1 (1, 2, 3), Module 2: (4, 5, 6)...
    static constexpr int FLsteerID = 10;
    static constexpr int FLdriveID = 11;
    static constexpr int FLencoderID = 12;
    
    static constexpr int FRsteerID = 1;
    static constexpr int FRdriveID = 2;
    static constexpr int FRencoderID = 3;

    static constexpr int BLsteerID = 7;
    static constexpr int BLdriveID = 8;
    static constexpr int BLencoderID = 9;

    static constexpr int BRsteerID = 4;
    static constexpr int BRdriveID = 5;
    static constexpr int BRencoderID = 6;

    static constexpr int pigeonID = 0;
}

namespace ControllerConstants {
    static constexpr double deadband = 0.05;
    static constexpr double slewRate = 5.2;
}