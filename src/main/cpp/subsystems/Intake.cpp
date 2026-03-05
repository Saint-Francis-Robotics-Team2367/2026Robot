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
    pivotMotor.SetPosition(0_tr);
}

frc2::CommandPtr Intake::RunIntake(Intake* intake, double speed) {
    return frc2::cmd::StartEnd(
        [intake, speed] {intake->setMotorSpeed(speed);},
        [intake] {intake->stop();},
        {intake}
    );
}

frc2::CommandPtr Intake::DeployIntake(Intake* intake) {
    return frc2::cmd::StartEnd(
        [intake] {intake->deployIntake();},
        [intake] {intake->retractIntake();},
        {intake}
    );
}

// frc2::CommandPtr Intake::deploySequence() {
//     return frc2::cmd::Sequence(
//         frc2::cmd::RunOnce([this] {deployIntake();}),
//         frc2::cmd::Wait(0.1_s),
//         frc2::cmd::RunOnce([this] {deployHopper();}), 
//         {this}
//     );
// }

// frc2::CommandPtr Intake::retractSequence() {
//     return frc2::cmd::Sequence (
//         frc2::cmd::RunOnce([this] {retractHopper();}),
//         frc2::cmd::Wait(0.1_s),
//         frc2::cmd::RunOnce([this] {retractIntake();}),
//         {this}
//     );
// }

void Intake::deployHopper() {
    hopperMotor.SetControl(ctre::phoenix6::controls::PositionVoltage{hopperDeployedPos}.WithSlot(0));
}

void Intake::retractHopper() {
    hopperMotor.SetControl(ctre::phoenix6::controls::PositionVoltage{hopperStowPos}.WithSlot(0));
}

void Intake::deployIntake() {
    pivotMotor.SetControl(ctre::phoenix6::controls::PositionVoltage{deployedPos}.WithSlot(0));
}

void Intake::retractIntake() {
    pivotMotor.SetControl(ctre::phoenix6::controls::PositionVoltage{-1*deployedPos}.WithSlot(0));
}

void Intake::setMotorSpeed(double speed) {
    rollerMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{units::turns_per_second_t(speed / 60.0)}.WithSlot(0));
}

void Intake::stop() {
    rollerMotor.Set(0);
}