#pragma once

#include <ctre/phoenix6/TalonFX.hpp> // Include Phoenix 6 API
#include "ctre/phoenix6/CANcoder.hpp"
#include "ctre/phoenix6/configs/Configuration.hpp"
#include "ctre/phoenix6/controls/PositionVoltage.hpp"
#include "ctre/phoenix6/controls/DutyCycleOut.hpp"
#include "ctre/phoenix6/controls/VelocityDutyCycle.hpp"
#include "ctre/phoenix6/controls/PositionDutyCycle.hpp"

#include <iostream>
#include <chrono>
#include <thread>
#include <units/angular_velocity.h>
#include <cmath>
#include "Constants.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/SubsystemBase.h>

class Shooter : public frc2::SubsystemBase{
public:
    void stop();
    void init();
    bool setFlywheelSpeed(float rotationsPerMinute);
    void setHoodPosition(float shooterRPM, float horizontalOffset, float yOffset, float shooterHeight = (34), float initialAngle = 68, float minAngle = 31, float MotorGearRatio = ShooterConstants::motorGearRatio, float ThroughBoreGearRatio = 16.32);
    void setManualHoodPosition(float targetAngle);
    void moveHoodToZero();
    void applyHoodBrake();
    void releaseHoodBrake(); 
    float findOptimalRPM(float horizontalOffset, float yOffset);
    double getShooterVelocity();
    void ZeroHood();
    double findHoodAngle();

    float hoodCenterRot;
    float targetAbs;

public:
    // Motors
    ctre::phoenix6::hardware::TalonFX ShooterMotor{ShooterConstants::ShooterID, "Drivetrain"}; // Use CANivore bus if applicable
    ctre::phoenix6::hardware::TalonFX RackMotor{ShooterConstants::RackMotorID, "Drivetrain"};

    // Configuration objects
    ctre::phoenix6::configs::TalonFXConfiguration FlywheelConfig;
    ctre::phoenix6::configs::TalonFXConfiguration RackConfig;

    // ThroughBore
    ctre::phoenix6::hardware::CANcoder RackEncoder{ShooterConstants::RackEncoderID, "Drivetrain"}; // Uses default CAN bus
    
    ctre::phoenix6::controls::PositionVoltage positionVoltage{0_tr};


    // Timeout for configuration
    const std::chrono::milliseconds kTimeoutMs{30};

};




