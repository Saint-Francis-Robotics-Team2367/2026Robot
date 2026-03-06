// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <thread>
#include <chrono>

#include <frc2/command/button/Trigger.h>
#include "frc2/command/button/RobotModeTriggers.h"


#include "subsystems/Turret.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include <frc2/command/CommandPtr.h>


//basically initializes robot
RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  // Initialize Shooter
  ConfigureBindings();
  HoodedShooter.init(); // Initalize Shooter motors and encoders
  BallFeeder.init(); // Initialize Feeder motors and encoders
  BallIndexer.init(); // Initialize Indexer motors and encoders
  

  drivetrain.initModules();
  drivetrain.initGyro();
  drivetrain.resetOdometry(frc::Pose2d{0_m, 0_m, 0_rad});
}


void RobotContainer::ConfigureBindings() {
  frc::SmartDashboard::PutString("Shooter Status", "Idle");

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
  
  // driverCtr.Circle().ToggleOnTrue(
  //   frc2::cmd::StartEnd(
  //     // ON
  //     [this] {
  //       frc::SmartDashboard::PutString("Shooter Status", "Shooting");
  //       HoodedShooter.applyHoodBrake(); 
  //       BallFeeder.setFeederSpeed(-2250);
  //       HoodedShooter.setFlywheelSpeed(HoodedShooter.findOptimalRPM(132, 186));

  //     },
  //     // OFF
  //     [this] {
  //       frc::SmartDashboard::PutString("Shooter Status", "Idle");
  //       HoodedShooter.setFlywheelSpeed(0);
  //       BallFeeder.setFeederSpeed(0);
  //       HoodedShooter.releaseHoodBrake(); 
  //     },
  //     { &HoodedShooter, &BallFeeder } 
  //   )
  // );

  // driverCtr.Triangle().OnTrue(
  //   frc2::cmd::RunOnce(
  //     [this] {
  //       frc::SmartDashboard::PutString("Shooter Status", "Aligning");
  //       HoodedShooter.setHoodPosition(HoodedShooter.findOptimalRPM(132, 186), 132, 186);
  //     },
  //     { &HoodedShooter} 
  //   )
  // );

  // driverCtr.Square().OnTrue(
  //   frc2::cmd::RunOnce(
  //     [this] {
  //       frc::SmartDashboard::PutString("Shooter Status", "Zeroing Hood");
  //       HoodedShooter.zeroHood();
  //     },
  //     { &HoodedShooter } 
  //   )
  // );

  // driverCtr.POVUp().OnTrue(
  //   drivetrain.RunOnce(
  //     [this] {drivetrain.resetGyro();}
  //   )
  // );

  // driverCtr.POVDown().OnTrue(
  //   frc2::cmd::StartEnd(
  //     // ON
  //     [this] {
  //       frc::SmartDashboard::PutString("Shooter Status", "Shooting");
  //       BallFeeder.setFeederSpeed(HoodedShooter.findOptimalRPM(132, 186));
  //     },
  //     // OFF
  //     [this] {
  //       frc::SmartDashboard::PutString("Shooter Status", "Idle");
  //       BallFeeder.setFeederSpeed(0);
  //     },
  //     {&BallFeeder, &HoodedShooter} 
  //   )
  // );

  driverCtr.Triangle().ToggleOnTrue(
    BallFeeder.RunFeeder(&BallFeeder, -1800)
  );

  driverCtr.Circle().ToggleOnTrue(
    BallIndexer.RunIndexer(&BallIndexer, -3000)
  );

  driverCtr.Cross().ToggleOnTrue(
    mRunIntake.IntakeCommand(&mRunIntake, 3000)
  );

  driverCtr.Square().ToggleOnTrue(
    mDeployIntake.DeployIntakeCommand(&mDeployIntake)
  );

  driverCtr.POVLeft().OnTrue(
    frc2::cmd::RunOnce(
      [this] {mDeployIntake.zeroPivot();}
    )
  );
}


//************* TURRET TEST COMMANDS **************

/*
driverCtr.POVUp().ToggleOnTrue(
    m_turret.RunOnce(
        [this] { m_turret.addToSetpoint(45);}   
    )
);

driverCtr.POVDown().ToggleOnTrue(
    m_turret.RunOnce(
        [this] { m_turret.addToSetpoint(-45); }
    )
);


driverCtr.Triangle().OnTrue(
    m_turret.RunOnce(
        [this] { m_turret.setAngle(180); }    
    )
);

driverCtr.Circle().OnTrue(
    m_turret.RunOnce(
        [this] { m_turret.setAngle(0); }    
    )
);

driverCtr.Square().OnTrue(
    m_turret.RunOnce(
        [this] { m_turret.setAngle(m_turret.getSetpoint()); }    
    )
);
}
*/


// frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
//   // An example command will be run in autonomous
//   //return autos::ExampleAuto(&m_subsystem);
// }
