// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
  FeederConfig.CurrentLimits.StatorCurrentLimit = 40_A;
  FeederConfig.CurrentLimits.StatorCurrentLimitEnable = true;

  FeederConfig.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Coast;

  // Apply configuration
  FeederMotor.GetConfigurator().Apply(FeederConfig);
}

void Feeder::setFeederSpeed(double rpm) {
  // Set feeder velocity
  FeederMotor.SetControl(velocityVoltage
                             .WithVelocity(units::turns_per_second_t{
                                 std::clamp(rpm / 60.0, -100.0, 100.0)})
                             .WithSlot(0));
}

frc2::CommandPtr Feeder::RunFeeder(Feeder* feeder, double rpm) {
  return frc2::cmd::StartEnd([feeder, rpm] { feeder->setFeederSpeed(rpm); },
                             [feeder] { feeder->FeederMotor.Set(0); },
                             {feeder});
}
