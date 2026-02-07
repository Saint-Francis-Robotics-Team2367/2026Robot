#pragma once

#include <ctre/phoenix6/TalonFX.hpp> // Include Phoenix 6 API
#include "ctre/phoenix6/CANcoder.hpp"
#include "ctre/phoenix6/configs/Configuration.hpp"
#include "ctre/phoenix6/controls/PositionVoltage.hpp"

#include <iostream>
#include <chrono>
#include <thread>
#include <units/angular_velocity.h>
#include <cmath>
#include "Constants.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Subsystem.h>

class Shooter : public frc2::Subsystem{
public:
    void stop();
    void init();
    bool setFlywheelSpeed(float rotationsPerMinute);
    void setHoodPosition(float shooterRPM, float horizontalOffset, float yOffset, float shooterHeight = (14.186 + 2), float initialAngle = 68, float minAngle = 31, float GearRatio = 116.8831);

private:
    // Motors
    ctre::phoenix6::hardware::TalonFX ShooterMotor{ShooterConstants::ShooterID}; // Use CANivore bus if applicable
    ctre::phoenix6::hardware::TalonFX RackMotor{ShooterConstants::RackMotorID};


    // Configuration objects
    ctre::phoenix6::configs::TalonFXConfiguration FlywheelConfig;
    ctre::phoenix6::configs::TalonFXConfiguration RackConfig;


    // ThroughBore
    ctre::phoenix6::hardware::CANcoder RackEncoder{ShooterConstants::RackEncoderID}; // Uses default CAN bus


    // Timeout for configuration
    const std::chrono::milliseconds kTimeoutMs{30};

};




