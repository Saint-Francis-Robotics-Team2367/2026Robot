#include "subsystems/Intake/DeployIntake.h"
#include "frc/smartdashboard/SmartDashboard.h"

void DeployIntake::init() {
    backLeftConfig.Slot0.kP = 0.5;
    backLeftConfig.Slot0.kI = 0.0;
    backLeftConfig.Slot0.kD = 0.0;

    backLeftConfig.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
    backLeftConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    backLeftMotor.GetConfigurator().Apply(backLeftConfig);

    backRightConfig.Slot0.kP = 0.5;
    backRightConfig.Slot0.kI = 0.0;
    backRightConfig.Slot0.kD = 0.0;

    backRightConfig.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
    backRightConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    backRightMotor.GetConfigurator().Apply(backRightConfig);

    backLeftMotor.SetPosition(0_tr);
    backRightMotor.SetPosition(0_tr);
}

frc2::CommandPtr DeployIntake::masterIntakeCommand(DeployIntake* intake, bool shooting) {
    if (shooting) {
        return frc2::cmd::Sequence(
            frc2::cmd::RunOnce(
                [intake] {intake->retract();}
            ),
            frc2::cmd::Wait(0.25_s),
            frc2::cmd::RunOnce(
                [intake] {intake->deploy();}
            ),
            frc2::cmd::Wait(0.25_s)
        ).Repeatedly();
    }
    else {
        return frc2::cmd::StartEnd(
            [intake] {
                if (intake->deployed == false) {
                    intake->deploy();
                    intake->deployed = true;
                }
            },
            [intake] {
                if (intake->deployed == true) {
                    intake->retract();
                    intake->deployed = false;
                }
            },
            {intake}
        );
    }
}

void DeployIntake::deploy() {
    backLeftMotor.SetControl(positionVoltage.WithPosition(deployedPos).WithSlot(0));
    backRightMotor.SetControl(positionVoltage.WithPosition(deployedPos).WithSlot(0));
}

void DeployIntake::retract() {
    backLeftMotor.SetControl(positionVoltage.WithPosition(0_tr).WithSlot(0));
    backRightMotor.SetControl(positionVoltage.WithPosition(0_tr).WithSlot(0));
}

void DeployIntake::zeroMotors(double zeroAmt) {
    backLeftMotor.SetPosition(units::turn_t(zeroAmt));
    backRightMotor.SetPosition(units::turn_t(zeroAmt));
}