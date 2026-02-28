#include "subsystems/Feeder.h"

void Feeder::stop() {
    // Stop the feeder motor
    FeederMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
}

void Feeder::init() {
    // Configure PID constants for Feeder Motor (Velocity Control)
    FeederConfig.Slot0.kP = ShooterConstants::FeederP;
    FeederConfig.Slot0.kI = ShooterConstants::FeederI;
    FeederConfig.Slot0.kD = ShooterConstants::FeederD;
    FeederConfig.Slot0.kV = ShooterConstants::FeederV;

    FeederConfig.CurrentLimits.SupplyCurrentLimit = 10_A; 
    FeederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    FeederConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

    // Apply configuration
    FeederMotor.GetConfigurator().Apply(FeederConfig);
}

void Feeder::setFeederSpeed(float rotationsPerMinute) {
    // Set feeder velocity
    FeederMotor.SetControl(ctre::phoenix6::controls::VelocityDutyCycle{units::angular_velocity::turns_per_second_t{rotationsPerMinute / 60.0}});
}