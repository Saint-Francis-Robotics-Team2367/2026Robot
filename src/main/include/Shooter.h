#pragma once

#include <ctre/phoenix6/TalonFX.hpp> // Include Phoenix 6 API
#include "ctre/phoenix6/CANcoder.hpp"
#include "ctre/phoenix6/configs/Configuration.hpp"
#include "ctre/phoenix6/controls/PositionVoltage.hpp"
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <units/angular_velocity.h>
#include <cmath>


// Constants for motor configuration
constexpr int ShooterID = 1; // CAN ID for the master flywheel motor
constexpr int RackMotorID = 2;       // CAN ID for the rack motor


// PID constants for Flywheel Motor (Velocity Control)
constexpr double FlywheelP = 0.0;
constexpr double FlywheelI = 0.0;
constexpr double FlywheelD = 0.0;
constexpr double FlywheelV = 0.0;


// PID constants for Rack Motor (Position Control)
constexpr double RackP = 0.0;
constexpr double RackI = 0.0;
constexpr double RackD = 0.0;
constexpr double RackG = 0.0;


class Shooter {
public:
    void stop();
    void init();
    bool setFlywheelSpeed(float rotationsPerMinute);
    void setHoodPosition(float shooterRPM, float horizontalOffset, float yOffset, float cameraHeight, float initialAngle, float maxAngle, float GearRatio);

private:
    // Motors
    ctre::phoenix6::hardware::TalonFX ShooterMotor{ShooterID}; // Use CANivore bus if applicable
    ctre::phoenix6::hardware::TalonFX RackMotor{RackMotorID};


    // Configuration objects
    ctre::phoenix6::configs::TalonFXConfiguration FlywheelConfig;
    ctre::phoenix6::configs::TalonFXConfiguration RackConfig;


    // ThroughBore
    ctre::phoenix6::hardware::CANcoder RackEncoder{3}; // Uses default CAN bus


    // Timeout for configuration
    const std::chrono::milliseconds kTimeoutMs{30};

};




