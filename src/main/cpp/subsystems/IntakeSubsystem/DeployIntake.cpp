// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake/DeployIntake.h"
#include "frc/smartdashboard/SmartDashboard.h"

void DeployIntake::init() {
  hopperConfig.Slot0.kP = 0.5;
  hopperConfig.Slot0.kI = 0.0;
  hopperConfig.Slot0.kD = 0.0;

  hopperConfig.MotorOutput.Inverted =
      ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  hopperConfig.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;
  hopperMotor.GetConfigurator().Apply(hopperConfig);

  pivotConfig.Slot0.kP = 0.5;
  pivotConfig.Slot0.kI = 0.0;
  pivotConfig.Slot0.kD = 0.0;

  pivotConfig.MotorOutput.Inverted =
      ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  pivotConfig.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;
  pivotMotor.GetConfigurator().Apply(hopperConfig);

  pivotMotor.SetPosition(0_tr);
}

frc2::CommandPtr DeployIntake::DeployIntakeCommand(DeployIntake* intake) {
  return frc2::cmd::StartEnd([intake] { intake->deployIntake(); },
                             [intake] { intake->retractIntake(); }, {intake});
}

void DeployIntake::deployHopper() {
  hopperMotor.SetControl(
      ctre::phoenix6::controls::PositionVoltage{hopperDeployedPos}.WithSlot(0));
}

void DeployIntake::retractHopper() {
  hopperMotor.SetControl(
      ctre::phoenix6::controls::PositionVoltage{hopperStowPos}.WithSlot(0));
}

void DeployIntake::deployIntake() {
  pivotMotor.SetControl(
      ctre::phoenix6::controls::PositionVoltage{deployedPos}.WithSlot(0));
}

void DeployIntake::retractIntake() {
  pivotMotor.SetControl(
      ctre::phoenix6::controls::PositionVoltage{-1 * deployedPos}.WithSlot(0));
}

void DeployIntake::zeroPivot(double zeroAmt) {
  pivotMotor.SetPosition(units::turn_t(zeroAmt));
}
