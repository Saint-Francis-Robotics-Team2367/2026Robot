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
  QuestNav::getInstance().init();
  drivetrain.resetOdometry(frc::Pose2d{0_m, 0_m, 0_rad});

  autoTargeting = true;
}


void RobotContainer::ConfigureBindings() {
  // ******************** Trigger Functions ********************
  frc2::Trigger rightStickYMoving{
    [this] {
      return std::abs(frc::ApplyDeadband(codriverCtr.GetRightY(), ControllerConstants::deadband)) > 0.0;
    }
  };

  frc2::Trigger leftStickXMoving{
    [this] {
      return std::abs(frc::ApplyDeadband(codriverCtr.GetLeftX(), ControllerConstants::deadband)) > 0.0;
    }
  };

  frc2::Trigger turretAutoTargetingOn{
    [this] {
      return autoTargeting;
    }
  };

  frc2::Trigger IndexerStall{
    [this] {
      return BallIndexer.IndexerStall();
    }
  };

  // ******************** DEFAULT COMMANDS ********************
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

        drivetrain.Drive(vx, -vy, -rot, drivetrain.gyroConnected());
      }
    )
  );

  // HoodedShooter.SetDefaultCommand(
  //   HoodedShooter.Run(
  //     [this]() {HoodedShooter.setFlywheelSpeed(-500);}
  //   )
  // );

  turretAutoTargetingOn.WhileTrue(
    frc2::cmd::Run(
      [this] {
        m_turret.autoMoveToTarget();
      }
    )
  );

  // Detect Indexer Stall
  IndexerStall.OnTrue(
    frc2::cmd::Sequence(
      frc2::cmd::RunOnce(
        [this] {
          frc::SmartDashboard::PutBoolean("Indexer Stall", true);
        }
      ),
      frc2::cmd::Parallel(
        BallIndexer.RunIndexer(&BallIndexer, 3000),
        BallFeeder.RunFeeder(&BallFeeder, 3000)
      ).WithTimeout(2_s),
      frc2::cmd::RunOnce(
        [this] {
          frc::SmartDashboard::PutBoolean("Indexer Stall", false);
        }
      ),
      frc2::cmd::Wait(0.25_s)
    )
  );

  // ******************** Driver Controls ********************
  // Zero Gyro
  driverCtr.POVUp().OnTrue(
    frc2::cmd::RunOnce(
      [this] {QuestNav::getInstance().ZeroGyro();}
    )
  );

  // Run Intake
  driverCtr.R1().ToggleOnTrue(
    mRunIntake.IntakeCommand(&mRunIntake, 4000)
  );

  // Outtake Intake
  driverCtr.L1().ToggleOnTrue(
    mRunIntake.IntakeCommand(&mRunIntake, -4000)
  );

  // ******************** Co-Driver Controls ********************
  // Reverse Indexer and Feeder
  codriverCtr.L2().WhileTrue(
    frc2::cmd::Parallel(
      BallIndexer.RunIndexer(&BallIndexer, -3000),
      BallFeeder.RunFeeder(&BallFeeder, -3000)
    )
  );

  codriverCtr.L1().WhileTrue(
    frc2::cmd::Parallel(
      BallIndexer.RunIndexer(&BallIndexer, 3000),
      BallFeeder.RunFeeder(&BallFeeder, 3000)
    )
  );

  // Apply Hood Brake
  /*
  codriverCtr.Triangle().OnTrue(
    HoodedShooter.RunOnce(
      [this] {HoodedShooter.applyHoodBrake();}
    )
  );
  */

  codriverCtr.R2().ToggleOnTrue(
    frc2::cmd::Sequence(
      // Step 1: Set hood position
      HoodedShooter.RunOnce(
        [this] {
          HoodedShooter.setHoodPosition(HoodedShooter.findOptimalRPM(122, 135), 122, 135);
        }
      ),
      // Step 2: Spin up flywheel and wait 4 seconds, then feed while flywheel keeps spinning
      frc2::cmd::Parallel(
        HoodedShooter.Run(
          [this] {
            HoodedShooter.setFlywheelSpeed(-(HoodedShooter.findOptimalRPM(122, 135))); // QuestNav::getInstance().getPose2d().X().value() * ShooterConstants::meterToInches, QuestNav::getInstance().getPose2d().Y().value() * ShooterConstants::meterToInches
          }
        ),
        frc2::cmd::Sequence(
          frc2::cmd::WaitUntil(
            [this] {
              return (HoodedShooter.getShooterVelocity() > (0.85 * (1/ShooterConstants::SHOOTEREFFICIENCY) * HoodedShooter.findOptimalRPM(122, 135)));
            }
          ),
          // Step 3: Run indexer and feeder while flywheel is still spinning
          frc2::cmd::RunOnce([this] {
            frc::SmartDashboard::PutString("Ran", "RAN INDEXER AND FEEDER");
          }),
          frc2::cmd::Parallel(
            BallIndexer.RunIndexer(&BallIndexer, -3000),
            BallFeeder.RunFeeder(&BallFeeder, -3000)
          )
        )
      )
    )
  );

  codriverCtr.Triangle().OnTrue(
    frc2::cmd::Sequence(
      HoodedShooter.RunOnce(
        [this] {
          HoodedShooter.setFlywheelSpeed(0);
        }
      ),
      HoodedShooter.RunOnce(
        [this] {
          HoodedShooter.moveHoodToZero();
        }
      )
    )
  );

  // Zero Hood Position
  (codriverCtr.R1() && codriverCtr.Triangle()).OnTrue(
    HoodedShooter.RunOnce(
      [this] {HoodedShooter.ZeroHood();}
    )
  );

  // Zero Turret Position
  (codriverCtr.R1() && codriverCtr.Circle()).OnTrue(
    m_turret.RunOnce(
      [this] {m_turret.ZeroTurret();}
    )
  );

  // Co-Driver Manual Turret Movement (NEEDS TO BE TESTED)
  (codriverCtr.R1() && leftStickXMoving).WhileTrue(
    m_turret.Run(
      [this] {
        // Make separate turret slew rate limiter if needed
        double leftX = frc::ApplyDeadband(codriverCtr.GetLeftX(), ControllerConstants::deadband);
        leftX = xLimiter.Calculate(leftX);

        frc::SmartDashboard::PutNumber("Turret Controller Left X", leftX);

        m_turret.setAngle(m_turret.getCurrentMotorAngle() + TurretConstants::turretTurnRatio * leftX * 10.0);
      }
    )
  );

  // Co-Driver Manual Hood Movement (NEEDS TO BE TESTED)
  (codriverCtr.R1() && rightStickYMoving).WhileTrue(
    HoodedShooter.Run(
      [this] {
        // Make separate turret slew rate limiter if needed
        double rightY = frc::ApplyDeadband(codriverCtr.GetRightY(), ControllerConstants::deadband);
        rightY = yLimiter.Calculate(rightY);

        frc::SmartDashboard::PutNumber("Shooter Hood Right Y", rightY);

        HoodedShooter.setManualHoodPosition(HoodedShooter.findHoodAngle() + ShooterConstants::shooterTurnRatio * rightY * 10.0);
      }
    )
  );

  // Enable Auto Targetting
  codriverCtr.POVUp().OnTrue(
    frc2::cmd::RunOnce(
      [this] {
        autoTargeting = !autoTargeting;
      }
    )
  );

  // ******************** Robot Disabling ********************
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
  //return autos::ExampleAuto(&m_subsystem);
}