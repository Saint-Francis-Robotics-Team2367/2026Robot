#include "subsystems/Intake/RunIntake.h"
#include <frc/smartdashboard/SmartDashboard.h>

void RunIntake::init() {
    //config for roller motor
    rollerConfig.Slot0.kP = 0.05;
    rollerConfig.Slot0.kI = 0.0;
    rollerConfig.Slot0.kD = 0.0;

    rollerConfig.CurrentLimits.SupplyCurrentLimit = 15_A;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    rollerConfig.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
    rollerConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    rollerMotor.GetConfigurator().Apply(rollerConfig);
}

frc2::CommandPtr RunIntake::IntakeCommand(RunIntake* intake, double speed) {
    return frc2::cmd::StartEnd(
        [intake, speed] {intake->setMotorSpeed(speed);},
        [intake] {intake->stop();},
        {intake}
    );
}

void RunIntake::setMotorSpeed(double speed) {
    rollerMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{units::turns_per_second_t(speed / 60.0)}.WithSlot(0));
}

void RunIntake::stop() {
    rollerMotor.Set(0);
}