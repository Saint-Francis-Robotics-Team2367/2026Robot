// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include "frc2/command/button/RobotModeTriggers.h"

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"
#include "frc/DriverStation.h"
#include "pathplanner/lib/controllers/PathFollowingController.h"
#include "pathplanner/lib/controllers/PPHolonomicDriveController.h"

//basically initializes robot
RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();
  drivetrain.initModules();
  drivetrain.initGyro();
  drivetrain.resetOdometry(frc::Pose2d{0_m, 0_m, 0_rad});

  pathplanner::RobotConfig robotConfig = pathplanner::RobotConfig::fromGUISettings();

  pathplanner::AutoBuilder::configure(
    [this]() {return drivetrain.getPose();}, 
    [this](const frc::Pose2d& pose) {drivetrain.resetOdometry(pose);},
    [this]() {return drivetrain.getRobotRelativeSpeeds();},
    [this](auto speeds, auto feedforwards) {drivetrain.Drive(speeds.vx.value(), speeds.vy.value(), speeds.omega.value(), drivetrain.gyroConnected());},
    std::make_shared<pathplanner::PPHolonomicDriveController> (
      pathplanner::PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
      pathplanner::PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
    ),
    robotConfig,
    []() { return frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed;},
    &drivetrain
  );
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here
  drivetrain.SetDefaultCommand(
      drivetrain.Run(
        [this]() {
          double x = frc::ApplyDeadband(driverCtr.GetLeftX(), ControllerConstants::deadband);
          double y = frc::ApplyDeadband(driverCtr.GetLeftY(), ControllerConstants::deadband);
          double rot = frc::ApplyDeadband(driverCtr.GetRightX(), ControllerConstants::deadband);

          x = xLimiter.Calculate(x);
          y = yLimiter.Calculate(y);
          rot = rotLimiter.Calculate(rot);

          double vx = x * ModuleConstants::moduleMaxMPS;
          double vy = y * ModuleConstants::moduleMaxMPS;
          rot = rot * ModuleConstants::moduleMaxRot * 2;

          frc::SmartDashboard::PutNumber("vx", vx);
          frc::SmartDashboard::PutNumber("vy", vy);
          frc::SmartDashboard::PutNumber("rot", rot);

          drivetrain.Drive(-vx, vy, -rot, drivetrain.gyroConnected());
        }
      )
  );

  //resets gyro on DPad Up
  driverCtr.POVUp().OnTrue(
    drivetrain.RunOnce(
      [this] {drivetrain.resetGyro();}
    )
  );

  //stops modules if disabled
  frc2::RobotModeTriggers::Disabled().WhileTrue(
    drivetrain.RunOnce(
      [this] {
        drivetrain.stopAllModules();
      }
    ).IgnoringDisable(true)
  );
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return autos::FollowPath(&drivetrain, "A to S1");
}
