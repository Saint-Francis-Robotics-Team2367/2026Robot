#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configuration.hpp>
#include <ctre/phoenix6/CANcoder.hpp>

#include "Constants.h"
#include <array>
#include <units/angle.h>

#include "frc2/command/StartEndCommand.h"
#include "frc2/command/SubsystemBase.h"
#include "frc2/command/Requirements.h"
#include "frc2/command/Commands.h"
#include "frc2/command/CommandPtr.h"

class Intake : public frc2::SubsystemBase {
public:
    units::angle::turn_t stowedPos = 0_tr;
    units::angle::turn_t deployedPos = units::turn_t(1.95);   // Example values-->have to tune
    units::turn_t hopperStowPos = 0_tr;
    units::turn_t hopperDeployedPos = units::turn_t(1/8);

    std::array<units::angle::turn_t, 2> pivotPositions{stowedPos, deployedPos};

    int currentState = 0;        // 0 = stowed, 1 = deployed
    units::angle::turn_t targetPosition = 0_tr;

    void init();
    frc2::CommandPtr deploySequence();
    frc2::CommandPtr retractSequence();
    void deployIntake();
    void deployHopper();
    void retractHopper();
    void retractIntake();
    void setMotorSpeed(double speed);
    void stop();
    int getCurrentState();

public:    
    ctre::phoenix6::hardware::TalonFX pivotMotor{IntakeConstants::intakePivotID, "Drivetrain"};
    ctre::phoenix6::hardware::TalonFX rollerMotor{IntakeConstants::intakeRollerID, "Drivetrain"};

    ctre::phoenix6::hardware::TalonFX hopperMotor{IntakeConstants::hopperMotorID, "Drivetrain"};

    // ctre::phoenix6::hardware::CANcoder pivotEncoder{IntakeConstants::intakePivotID};
    // ctre::phoenix6::hardware::CANcoder rollerEncoder{IntakeConstants::intakeRollerID};

    ctre::phoenix6::configs::CANcoderConfiguration pivotEncoderConfig{};
    ctre::phoenix6::configs::CANcoderConfiguration rollerEncoderConfig{};

    ctre::phoenix6::configs::TalonFXConfiguration pivotConfig{};
    ctre::phoenix6::configs::TalonFXConfiguration rollerConfig{};
    ctre::phoenix6::configs::TalonFXConfiguration hopperConfig{};
};