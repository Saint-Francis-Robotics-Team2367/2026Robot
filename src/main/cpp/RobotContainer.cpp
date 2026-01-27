// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include "frc2/command/button/RobotModeTriggers.h"

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();
  drivetrain.initModules();
  drivetrain.initGyro();
  drivetrain.resetOdometry(frc::Pose2d{0_m, 0_m, 0_rad});
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

  driverCtr.POVUp().OnTrue(
    drivetrain.RunOnce(
      [this] {drivetrain.resetGyro();}
    )
  );

  frc2::RobotModeTriggers::Disabled().WhileTrue(
    drivetrain.RunOnce(
      [this] {
        drivetrain.stopAllModules();
      }
    ).IgnoringDisable(true)
  );
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  // return autos::ExampleAuto(&m_subsystem);
}
