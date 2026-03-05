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
#include "frc2/command/Commands.h"
#include "frc2/command/Command.h"

class Feeder : public frc2::SubsystemBase{
public:
    void stop();
    void init();
    void setFeederSpeed(double rpm);
    frc2::CommandPtr RunFeeder(Feeder* feeder, double rpm);

private:
    // Motors
    ctre::phoenix6::hardware::TalonFX FeederMotor{ShooterConstants::FeederID, "Drivetrain"}; // Use CANivore bus if applicable
    ctre::phoenix6::controls::VelocityVoltage velocityVoltage{0_tps};

    // Configuration objects
    ctre::phoenix6::configs::TalonFXConfiguration FeederConfig;

};




