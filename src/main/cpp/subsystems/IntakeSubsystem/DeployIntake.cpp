#include "subsystems/Intake/DeployIntake.h"
#include "frc/smartdashboard/SmartDashboard.h"

void DeployIntake::init() {
    pivotConfig.Slot0.kP = 0.5;
    pivotConfig.Slot0.kI = 0.0;
    pivotConfig.Slot0.kD = 0.0;
    
    pivotConfig.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
    pivotConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

    pivotConfig2.Slot0.kP = 0.5;
    pivotConfig2.Slot0.kI = 0.0;
    pivotConfig2.Slot0.kD = 0.0;

    pivotConfig2.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
    pivotConfig2.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

    pivotMotor1.GetConfigurator().Apply(pivotConfig);
    pivotMotor2.GetConfigurator().Apply(pivotConfig2);

    pivotMotor1.SetPosition(0_tr);
    pivotMotor2.SetPosition(0_tr);
}

frc2::CommandPtr DeployIntake::DeployIntakeCommand(DeployIntake* intake) {
    return frc2::cmd::StartEnd(
        [intake] {intake->deployIntake();},
        [intake] {intake->retractIntake();},
        {intake}
    );
}

void DeployIntake::deployIntake() {
    pivotMotor1.SetControl(ctre::phoenix6::controls::PositionVoltage{deployedPos}.WithSlot(0));
    pivotMotor2.SetControl(ctre::phoenix6::controls::PositionVoltage{deployedPos}.WithSlot(0));
}

void DeployIntake::retractIntake() {
    pivotMotor1.SetControl(ctre::phoenix6::controls::PositionVoltage{-1*deployedPos}.WithSlot(0));
    pivotMotor2.SetControl(ctre::phoenix6::controls::PositionVoltage{-1*deployedPos}.WithSlot(0));
}

void DeployIntake::zeroPivot(double zeroAmt) {
    pivotMotor1.SetPosition(units::turn_t(zeroAmt));
    pivotMotor2.SetPosition(units::turn_t(zeroAmt));
}