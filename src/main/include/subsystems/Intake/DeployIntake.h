#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configuration.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include "ctre/phoenix6/controls/PositionVoltage.hpp"

#include "Constants.h"
#include <array>
#include <units/angle.h>
#include "subsystems/Spindexer.h"

#include "frc2/command/StartEndCommand.h"
#include "frc2/command/SubsystemBase.h"
#include "frc2/command/Requirements.h"
#include "frc2/command/Commands.h"
#include "frc2/command/CommandPtr.h"

class DeployIntake : public frc2::SubsystemBase {
public:
    units::angle::turn_t leftDeployPos = units::turn_t(-4.00);   // Example values-->have to tune
    units::angle::turn_t rightDeployPos = units::turn_t(3.9);
    bool deployed = false;

    ctre::phoenix6::hardware::TalonFX backLeftMotor{IntakeConstants::backLeftIntakeID, "Drivetrain"};
    ctre::phoenix6::configs::TalonFXConfiguration backLeftConfig{};

    ctre::phoenix6::hardware::TalonFX backRightMotor{IntakeConstants::backRightIntakeID, "Drivetrain"};
    ctre::phoenix6::configs::TalonFXConfiguration backRightConfig{};

    ctre::phoenix6::controls::PositionVoltage positionVoltage{0_tr};

public:
    void init();
    frc2::CommandPtr deploySequence();
    frc2::CommandPtr retractSequence();
    frc2::CommandPtr masterIntakeCommand(DeployIntake* intake, Spindexer* mSpindexer, bool shooting);
    void deploy();
    void retract();
    void shooterDeploy();
    void shooterRetract();
    void zeroMotors(double zeroAmt = 0.0);
};