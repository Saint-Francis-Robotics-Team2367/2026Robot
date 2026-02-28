// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <thread>
#include <chrono>

#include <frc2/command/button/Trigger.h>
#include "frc2/command/button/RobotModeTriggers.h"

#include "Shooter.h"


//basically initializes robot
RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  // Initialize Shooter
  ConfigureBindings();
  HoodedShooter.init(); // Initalize Shooter motors and encoders
  // HoodedShooter.zeroHood(); // Set initial hood position to 0
}


void RobotContainer::ConfigureBindings() {
  frc::SmartDashboard::PutString("Shooter Status", "Idle");

  driverCtr.Circle().ToggleOnTrue(
    frc2::cmd::StartEnd(
      // ON
      [this] {
        frc::SmartDashboard::PutString("Shooter Status", "Shooting");
        HoodedShooter.applyHoodBrake(); 
        BallFeeder.setFeederSpeed(-2250);
        HoodedShooter.setFlywheelSpeed(-2250);

      },
      // OFF
      [this] {
        frc::SmartDashboard::PutString("Shooter Status", "Idle");
        HoodedShooter.setFlywheelSpeed(0);
        BallFeeder.setFeederSpeed(0);
        HoodedShooter.releaseHoodBrake(); 
      },
      { &HoodedShooter } 
    )
  );

  driverCtr.Triangle().OnTrue(
    frc2::cmd::RunOnce(
      [this] {
        frc::SmartDashboard::PutString("Shooter Status", "Aligning");
        HoodedShooter.setHoodPosition(2250, 132, 186);
      },
      { &HoodedShooter } 
    )
  );

  driverCtr.Square().OnTrue(
    frc2::cmd::RunOnce(
      [this] {
        frc::SmartDashboard::PutString("Shooter Status", "Zeroing Hood");
        HoodedShooter.zeroHood();
      },
      { &HoodedShooter }
    )
  );

  driverCtr.R1().OnTrue(
    frc2::cmd::Run(
      [this]{
        HoodedShooter.calculateAngle(1.08);
      },
    
    { &HoodedShooter }
    )
  );

}

// frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
//   // An example command will be run in autonomous
//   //return autos::ExampleAuto(&m_subsystem);
// }
