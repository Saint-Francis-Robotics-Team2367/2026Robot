#include "subsystems/Intake.h"
#include <frc/smartdashboard/SmartDashboard.h>

void Intake::init() {
    //random PID values-->need to tune
    pivotConfig.Slot0.kP = 0.05;
    pivotConfig.Slot0.kI = 0.0;
    pivotConfig.Slot0.kD = 0.0;
    pivotConfig.Slot0.kV = 0.5;

    pivotConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    pivotMotor.GetConfigurator().Apply(pivotConfig);
    
    //config for roller motor
    rollerConfig.Slot0.kP = 0.05;
    rollerConfig.Slot0.kI = 0.0;
    rollerConfig.Slot0.kD = 0.0;

    rollerConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    rollerMotor.GetConfigurator().Apply(rollerConfig);

    hopperConfig.Slot0.kP = 0.1;
    hopperConfig.Slot0.kI = 0.0;
    hopperConfig.Slot0.kD = 0.0;

    hopperConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    hopperMotor.GetConfigurator().Apply(hopperConfig);

    currentState = 0;
    targetPosition = stowedPos;
}

frc2::CommandPtr Intake::deploySequence() {
    return frc2::cmd::Sequence(
        frc2::cmd::RunOnce([this] {deployIntake();}),
        frc2::cmd::Wait(0.1_s),
        frc2::cmd::RunOnce([this] {deployHopper();})
    );
}

frc2::CommandPtr Intake::retractSequence() {
    return frc2::cmd::Sequence (
        frc2::cmd::RunOnce([this] {retractHopper();},{this}),
        frc2::cmd::Wait(0.1_s),
        frc2::cmd::RunOnce([this] {retractIntake();})
    );
}

void Intake::deployHopper() {
    hopperMotor.SetControl(ctre::phoenix6::controls::PositionVoltage{hopperDeployedPos});
}

void Intake::retractHopper() {
    hopperMotor.SetControl(ctre::phoenix6::controls::PositionVoltage{hopperStowPos});
}

void Intake::deployIntake() {
    currentState = 1;
    pivotMotor.SetControl(ctre::phoenix6::controls::PositionVoltage{deployedPos});
}

void Intake::retractIntake() {
    currentState = 0;
    pivotMotor.SetControl(ctre::phoenix6::controls::PositionVoltage{stowedPos});
}

void Intake::setMotorSpeed(double speed) {
    rollerMotor.Set(speed);
}

void Intake::stop() {
    rollerMotor.Set(0);
}

int Intake::getCurrentState() {
    return currentState;
}