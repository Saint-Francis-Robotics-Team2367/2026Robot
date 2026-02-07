#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include <ctre/phoenix6/controls/PositionVoltage.hpp>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/core/CoreCANcoder.hpp>

#include <frc2/command/Subsystem.h>

constexpr int encoderID = 12;
constexpr int motorID = 34;

constexpr double pulleyRatio = 30; //big wheel to small wheel (encoder) ratio

constexpr double kP = 0.3;
constexpr double kI = 0.0;
constexpr double kD = 0.0;


class Turret : public frc2::Subsystem{
   public:
        Turret();
        void init(); 
        void stop(); //stop motors
        void setSpeed(double speed);
        double getCurrentAngle(); //use the gear ratio to calculate the current angle of the turret
        void setAngle(double targetAngle);
        bool isAtAngle(double targetAngle);
        void resetTurretPosition();
        double encoderCounter();

   private:

        ctre::phoenix6::hardware::TalonFX turretMotor{motorID};
        ctre::phoenix6::configs::TalonFXConfiguration turretConfigs{};

        ctre::phoenix6::hardware::CANcoder encoder;  
        ctre::phoenix6::configs::CANcoderConfiguration encoderConfigs{}; 

        ctre::phoenix6::controls::PositionVoltage positionVoltage{0_tr}; //turns 

        double encoderOffset;

        //since it is an absolute encoder that will go on the small pulley, need to keep track of how much the small pulley turns
        //mainly for resetting the position of the turret when the angle exceeds 180
        double smallPulleyCounter = 0;

};



