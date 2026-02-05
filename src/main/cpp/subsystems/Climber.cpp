#include "subsystems/Climber.h"

void Climber::init() {
    leftKrakenConfigs.CurrentLimits.SupplyCurrentLimit = 40_A;
    leftKrakenConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

    leftKrakenConfigs.Slot0.kP = 0.035;
    leftKrakenConfigs.Slot0.kI = 0.0;
    leftKrakenConfigs.Slot0.kD = 0.0;
    leftKrakenConfigs.Slot0.kV = 0.0;

    leftKrakenConfigs.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    leftKrakenConfigs.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;

    leftKrakenMotor.GetConfigurator().Apply(leftKrakenConfigs);

    rightKrakenConfigs.CurrentLimits.SupplyCurrentLimit = 40_A;
    rightKrakenConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

    rightKrakenConfigs.Slot0.kP = 0.035;
    rightKrakenConfigs.Slot0.kI = 0.0;
    rightKrakenConfigs.Slot0.kD = 0.0;
    rightKrakenConfigs.Slot0.kV = 0.0;

    rightKrakenConfigs.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    rightKrakenConfigs.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;

    rightKrakenMotor.GetConfigurator().Apply(rightKrakenConfigs);

    rightKrakenMotor.SetPosition(0_tr);
    leftKrakenMotor.SetPosition(0_tr);
}

void Climber::setPosition(float position) { // Turns
    leftKrakenMotor.SetControl(positionVoltage.WithPosition(units::turn_t(position)).WithSlot(0));
    rightKrakenMotor.SetControl(positionVoltage.WithPosition(units::turn_t(position)).WithSlot(0));
}

void Climber::setVelocity(float velocity) { // Turns per second
    leftKrakenMotor.SetControl(velocityVoltage.WithVelocity(units::turns_per_second_t(velocity)).WithSlot(0));
    rightKrakenMotor.SetControl(velocityVoltage.WithVelocity(units::turns_per_second_t(velocity)).WithSlot(0));
}

void Climber::setServoPosition(float position) { // Degrees
    leftServo.SetAngle(position);
    rightServo.SetAngle(position);
}