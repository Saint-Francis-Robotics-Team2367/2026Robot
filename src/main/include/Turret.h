#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include <ctre/phoenix6/controls/PositionVoltage.hpp>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/core/CoreCANcoder.hpp>

constexpr int encoderID = 12;
constexpr int motorID = 34;

constexpr double gearRatio = 30; //encoder to turret ratio

constexpr double kP = 0.3;
constexpr double kI = 0.0;
constexpr double kD = 0.0;


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

        ctre::phoenix6::controls::PositionVoltage positionVoltage{0_tr}; //turns 

        double encoderOffset;
      
};



