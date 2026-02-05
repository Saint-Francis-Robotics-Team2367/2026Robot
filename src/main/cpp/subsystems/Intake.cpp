#include "subsystems/Intake.h"
#include <frc/smartdashboard/SmartDashboard.h>

void Intake::init() {
    //random PID values-->need to tune
    pivotConfig.Slot0.kP = 0.05;
    pivotConfig.Slot0.kI = 0.0;
    pivotConfig.Slot0.kD = 0.0;

    pivotConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    pivotMotor.GetConfigurator().Apply(pivotConfig);
    
    //config for roller motor
    rollerConfig.Slot0.kP = 0.05;
    rollerConfig.Slot0.kI = 0.0;
    rollerConfig.Slot0.kD = 0.0;

    rollerConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    rollerMotor.GetConfigurator().Apply(rollerConfig);

    currentState = 0;
    targetPosition = stowedPos;
}

void Intake::deploy() {
    currentState = 1;
    pivotMotor.SetControl(ctre::phoenix6::controls::PositionVoltage{deployedPos});
}

void Intake::retract() {
    currentState = 0;
    pivotMotor.SetControl(ctre::phoenix6::controls::PositionVoltage{stowedPos});
}

void Intake::setMotorSpeed(double speed) {
    rollerMotor.Set(speed);
}

void Intake::intake(double speed) {
    rollerMotor.Set(speed);
}

void Intake::stop() {
    rollerMotor.Set(0);
}

int Intake::getCurrentState() {
    return currentState;
}