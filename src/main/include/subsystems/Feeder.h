#pragma once

#include <ctre/phoenix6/TalonFX.hpp> // Include Phoenix 6 API
#include "ctre/phoenix6/CANcoder.hpp"
#include "ctre/phoenix6/configs/Configuration.hpp"
#include "ctre/phoenix6/controls/PositionVoltage.hpp"
#include "ctre/phoenix6/controls/DutyCycleOut.hpp"
#include "ctre/phoenix6/controls/VelocityDutyCycle.hpp"
#include "ctre/phoenix6/controls/PositionDutyCycle.hpp"

#include <units/angular_velocity.h>
#include <cmath>
#include "Constants.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/SubsystemBase.h>

class Feeder : public frc2::SubsystemBase{
public:
    void stop();
    void init();
    void setFeederSpeed(float rotationsPerMinute);

private:
    // Motors
    ctre::phoenix6::hardware::TalonFX FeederMotor{ShooterConstants::FeederID}; // Use CANivore bus if applicable

    // Configuration objects
    ctre::phoenix6::configs::TalonFXConfiguration FeederConfig;

};




