#pragma once
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>

#include "sensors/CAN_Coder.h"
#include "geometry/Rotation2d.h"




constexpr int encoderID = 12;
constexpr int motorID = 34;

constexpr double gearRatio = 30; //encoder to turret ratio

constexpr double kP = 0.3;
constexpr double kI = 0.0;
constexpr double kD = 0.0;

constexpr double encoderOffset;


class Turret{
   public:
       void init(); 
       void stop(); //stop motors
       void setSpeed(double speed);
       double getCurrentAngle(); //use the gear ratio to calculate the current angle of the turret
       void setAngle(double targetAngle);
       bool isAtAngle(double targetAngle);
       void resetTurretPosition();

   private:

        ctre::phoenix6::hardware::TalonFX turretMotor{motorID};
        ctre::phoenix6::configs::TalonFXConfiguration turretConfigs{};

        ctre::phoenix6::hardware::CANcoder encoder;  
        ctre::phoenix6::configs::CANcoderConfiguration encoderConfigs{}; 

        ctre::phoenix6::controls::VelocityVoltage velocityVoltage{0_tps}; //turns per second
        ctre::phoenix6::controls::PositionVoltage positionVoltage{0_tr}; //turns 
      
};


//1. needs to reset everytime angle change exceeds 180
//2. need to use the encoder to correct the pid

