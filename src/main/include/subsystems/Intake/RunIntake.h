// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configuration.hpp>
#include <ctre/phoenix6/CANcoder.hpp>

#include "Constants.h"
#include <array>
#include <units/angle.h>

#include "frc2/command/StartEndCommand.h"
#include "frc2/command/SubsystemBase.h"
#include "frc2/command/Requirements.h"
#include "frc2/command/Commands.h"
#include "frc2/command/CommandPtr.h"

class RunIntake : public frc2::SubsystemBase {
 public:
  void init();
  frc2::CommandPtr IntakeCommand(RunIntake* intake, double speed);
  void setMotorSpeed(double speed);
  void stop();

 private:
  ctre::phoenix6::hardware::TalonFX rollerMotor{IntakeConstants::intakeRollerID,
                                                "Drivetrain"};
  ctre::phoenix6::configs::TalonFXConfiguration rollerConfig{};
};
