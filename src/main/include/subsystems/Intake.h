#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configuration.hpp>
#include "Constants.h"
#include <array>
#include <units/angle.h>

class Intake {
    public:
        units::angle::turn_t stowedPos = 0.0_deg;
        units::angle::turn_t deployedPos = 45.0_deg;   // Example values-->have to tune

        std::array<units::angle::turn_t, 2> pivotPositions{stowedPos, deployedPos};

        int currentState = 0;        // 0 = stowed, 1 = deployed
        units::angle::turn_t targetPosition = 0.0_deg;

        void init();
        void deploy();
        void retract();
        void setMotorSpeed(double speed);
        void intake(double speed = 1.0);
        void stop();
        int getCurrentState();

        ctre::phoenix6::hardware::TalonFX pivotMotor{IntakeConstants::intakePivotID};
        ctre::phoenix6::hardware::TalonFX rollerMotor{IntakeConstants::intakeRollerID};

        ctre::phoenix6::configs::TalonFXConfiguration pivotConfig{};
        ctre::phoenix6::configs::TalonFXConfiguration rollerConfig{};
};