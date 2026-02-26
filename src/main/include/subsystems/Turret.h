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
constexpr int motorID = 13;

constexpr double pulleyRatio = 44; //big wheel to small wheel (encoder) ratio
constexpr double tbTurretrRatio = 8.77778;

constexpr double kP = 0.8;
constexpr double kI = 0.04;
constexpr double kD = 0.10;


class Turret : public frc2::Subsystem{
   public:

     double setpoint = 0.0;
    double smallPulleyCounter = 0.0;
     
     Turret();
     void init(); 
     void addToSetpoint(double addition);

     void stop(); //stop motors
     void setSpeed(double speed);
     void changeSpeed(double increment);
     double getSpeed();

        
     double getCurrentAngle(); //use the gear ratio to calculate the current angle of the turret
     void setAngle(double targetAngle);
     bool isAtAngle(double targetAngle);
     void resetTurretPosition();
     double encoderCounter();
     void updateTurret(double targetAngle);
     void correctionFunction(double targetAngle);
     double getShortestAngle(double targetAngle);

   private:

        ctre::phoenix6::hardware::TalonFX turretMotor{motorID};
        ctre::phoenix6::configs::TalonFXConfiguration turretConfigs{};

        ctre::phoenix6::hardware::CANcoder encoder{encoderID, "rio"};  
        ctre::phoenix6::configs::CANcoderConfiguration encoderConfigs{}; 

        ctre::phoenix6::controls::PositionVoltage positionVoltage{0_tr}; //turns 

        double encoderOffset;

        //since it is an absolute encoder that will go on the small pulley, need to keep track of how much the small pulley turns
        //mainly for resetting the position of the turret when the angle exceeds 180
       
        double speed = 0.0;

};