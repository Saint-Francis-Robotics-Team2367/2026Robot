#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include <ctre/phoenix6/controls/PositionVoltage.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/core/CoreCANcoder.hpp>

#include <frc2/command/Subsystem.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <cmath> 
#include <iostream>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"

class Turret : public frc2::Subsystem{
  public:
     
    Turret(DriveSubsystem &driveInput) : mDrive(driveInput) {};
    void init();
    void setSpeed(double speed);
    void changeSpeed(double increment);
    double getSpeed();
    void addToSetpoint(double addition);
    void stop(); //stop motors

    double getCurrentMotorAngle(); //use the gear ratio to calculate the current angle of the turret
    double getCurrentEncoderAngle();
    double getSetpoint();

    void setAngle(double targetAngle);
    bool isAtAngle(double targetAngle);

    void resetTurretPosition();
    void ZeroTurret();
    void autoMoveToTarget();
  
  private:
    DriveSubsystem &mDrive;

    ctre::phoenix6::hardware::TalonFX turretMotor{TurretConstants::turretMotorID, "Drivetrain"};
    ctre::phoenix6::configs::TalonFXConfiguration turretConfigs{};

    ctre::phoenix6::hardware::CANcoder encoder{TurretConstants::turretEncoderID, "Drivetrain"};  
    ctre::phoenix6::configs::CANcoderConfiguration encoderConfigs{}; 

    ctre::phoenix6::controls::PositionVoltage positionVoltage{0_tr}; //turns 

    double speed = 0.0;
    double setpoint = 0.0;

};
