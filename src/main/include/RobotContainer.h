// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandPS5Controller.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/Climber.h"

#include "frc/filter/SlewRateLimiter.h"
#include "frc/MathUtil.h"
#include "frc/smartdashboard/SmartDashboard.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

  DriveSubsystem drivetrain;
  Climber climber;

 private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc2::CommandPS5Controller driverCtr{0}; //drive controller should be first controller that is plugged in

  // The robot's subsystems are defined here...


  frc::SlewRateLimiter<units::scalar> xLimiter{ControllerConstants::slewRate / 1_s};
  frc::SlewRateLimiter<units::scalar> yLimiter{ControllerConstants::slewRate / 1_s};
  frc::SlewRateLimiter<units::scalar> rotLimiter{ControllerConstants::slewRate / 1_s};

  void ConfigureBindings();
};
