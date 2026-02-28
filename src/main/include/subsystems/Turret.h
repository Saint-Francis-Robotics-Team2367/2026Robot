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


constexpr int encoderID = 60;
constexpr int motorID = 0;

constexpr double pulleyRatio = 44; //big wheel to small wheel (encoder) ratio
constexpr double tbTurretRatio = 8.77778;

constexpr double kP = 1.05;
constexpr double kI = 0.02;
constexpr double kD = 0.10;


class Turret : public frc2::Subsystem{
  public:

    double setpoint = 0.0;
    double smallPulleyCounter = 0.0;
     
    Turret();
    void setSpeed(double speed);
    void changeSpeed(double increment);
    double getSpeed();
    void addToSetpoint(double addition);
    void stop(); //stop motors

    double getCurrentMotorAngle(); //use the gear ratio to calculate the current angle of the turret
    double getCurrentEncoderAngle();

    void setAngle(double targetAngle);
    bool isAtAngle(double targetAngle);

    void resetTurretPosition();
  
  private:

    ctre::phoenix6::hardware::TalonFX turretMotor{motorID};
    ctre::phoenix6::configs::TalonFXConfiguration turretConfigs{};

    ctre::phoenix6::hardware::CANcoder encoder{encoderID, "rio"};  
    ctre::phoenix6::configs::CANcoderConfiguration encoderConfigs{}; 

    ctre::phoenix6::controls::PositionVoltage positionVoltage{0_tr}; //turns 

    double speed = 0.0;

};
