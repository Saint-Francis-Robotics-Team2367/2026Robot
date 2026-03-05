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

class DeployIntake : public frc2::SubsystemBase {
private:
    units::angle::turn_t stowedPos = 0_tr;
    units::angle::turn_t deployedPos = units::turn_t(1.95);   // Example values-->have to tune
    units::turn_t hopperStowPos = 0_tr;
    units::turn_t hopperDeployedPos = units::turn_t(1/8);
    units::angle::turn_t targetPosition = 0_tr;

    ctre::phoenix6::hardware::TalonFX pivotMotor{IntakeConstants::intakePivotID, "Drivetrain"};
    ctre::phoenix6::configs::TalonFXConfiguration pivotConfig{};

    ctre::phoenix6::hardware::TalonFX hopperMotor{IntakeConstants::hopperMotorID, "Drivetrain"};
    ctre::phoenix6::configs::TalonFXConfiguration hopperConfig{};

public:
    void init();
    frc2::CommandPtr deploySequence();
    frc2::CommandPtr retractSequence();
    frc2::CommandPtr DeployIntakeCommand(DeployIntake* intake);
    void deployIntake();
    void deployHopper();
    void retractHopper();
    void retractIntake();
    void zeroPivot(double zeroAmt = 0.0);
};