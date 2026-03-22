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
  m_turret.init();

  drivetrain.initModules();
  drivetrain.initGyro();
  QuestNav::getInstance().init();
  drivetrain.resetOdometry(frc::Pose2d{0_m, 0_m, 0_rad});

  allianceChooser.SetDefaultOption("Blue Alliance", blueAlliance);
  allianceChooser.AddOption("Red Alliance", redAlliance);
  frc::SmartDashboard::PutData("Alliance Color", &allianceChooser);

  positionChooser.SetDefaultOption("Bottom Trench", bottomTrench);
  positionChooser.AddOption("Bottom Bump", bottomBump);
  positionChooser.AddOption("Front Hub", frontHub);
  positionChooser.AddOption("Top Bump", topBump);
  positionChooser.AddOption("Top Trench", topTrench);
  frc::SmartDashboard::PutData("Field Position", &positionChooser);
  
}


void RobotContainer::InitializeStartPose() {

  fieldPosition = positionChooser.GetSelected();
  allianceColor = allianceChooser.GetSelected();

  if (allianceColor == "Red Alliance") {
    startXOffset = PoseConstants::alliancePoseOffset;
    hubPoseX = PoseConstants::RedhubX;
  } else {
    startXOffset = 0.0;
    hubPoseX = PoseConstants::BluehubX;
  }

  double startX = 4.028694 + startXOffset;

  if (fieldPosition == "Top Trench") {
    startPose = frc::Pose2d{units::inch_t(startX),
                            units::meter_t(7.430262),  // meters
                            frc::Rotation2d{0_rad}};
  }
  else if (fieldPosition == "Top Bump") {
    startPose = frc::Pose2d{units::inch_t(startX),
                            units::meter_t(5.47497),  // meters
                            frc::Rotation2d{0_rad}};
  }
  else if (fieldPosition == "Front Hub") {
    startPose = frc::Pose2d{units::inch_t(startX),
                            units::meter_t(4.025 ),
                            frc::Rotation2d{0_rad}};
  }
  else if (fieldPosition == "Bottom Bump") {
    startPose = frc::Pose2d{units::inch_t(startX),
                            units::meter_t(2.59461),  // meters
                            frc::Rotation2d{0_rad}};
  }
  else if (fieldPosition == "Bottom Trench") {
    startPose = frc::Pose2d{units::inch_t(startX),
                            units::meter_t(0.639318),  // meters
                            frc::Rotation2d{0_rad}};
  }

  drivetrain.resetOdometry(startPose);
}

void RobotContainer::ConfigureBindings() {
  // ******************** Trigger Functions ********************
  frc2::Trigger rightStickYMoving(
    [&] {
      return std::abs(frc::ApplyDeadband(codriverCtr.GetRightY(), ControllerConstants::deadband)) > 0.0;
    }
  );

  frc2::Trigger leftStickXMoving(
    [&] {
      return std::abs(frc::ApplyDeadband(codriverCtr.GetLeftX(), ControllerConstants::deadband)) > 0.0;
    }
  );

  frc2::Trigger turretAutoTargetingOn(
    [&] {
      return autoTargeting;
    }
  );

  frc2::Trigger turretNoAprilTagDetected(
    [&] {
      return (noTagVisibleCounter > TurretConstants::turretNoTagResetThreshold);
    }
  );

  frc2::Trigger tagVisible(
    [&] {
      return turretCam.hasTarget;
    }
  );

  frc2::Trigger IndexerStall(
    [&] {
      return BallIndexer.IndexerStall();
    }
  );

  frc2::Trigger scorePossible{
    [&] {
      return drivetrain.getPose().Y().value() < 4.625594;
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

        drivetrain.Drive(vx, -vy, rot, drivetrain.gyroConnected());
      }
    )
  );

  // HoodedShooter.SetDefaultCommand(
  //   HoodedShooter.Run(
  //     [this]() {HoodedShooter.setFlywheelSpeed(-500);}
  //   )
  // );

  (turretAutoTargetingOn).WhileTrue(
    frc2::cmd::Run(
      [this] {
        if (turretCam.hasTarget)
        {
          double tx = std::clamp(turretCam.tx + m_turret.getCurrentMotorAngle(), -50.0, 50.0);
          double tolerance = std::sin(turretCam.tx * (std::numbers::pi / 180.0)) * turretCam.distanceToTag;
          tolerance = frc::ApplyDeadband(tolerance, TurretConstants::turretDeadband);
          m_turret.setAngle(tx);
          noTagVisibleCounter = 0;
        }
        else
        {
          noTagVisibleCounter++;
        }
      }
    )
  );

  (!turretAutoTargetingOn || turretNoAprilTagDetected).OnTrue(
    frc2::cmd::RunOnce(
      [this] {
        m_turret.setAngle(0);
      }
    )
  );

  // m_turret.SetDefaultCommand(
  //   m_turret.Run(
  //     [this]() {
  //       if (turretCam.isHub()) {
  //         double tx = std::clamp(turretCam.tx, -55.0, 55.0);
  //         m_turret.setAngle(tx);
  //       }
  //     }
  //   )
  // );

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
          HoodedShooter.distanceToTag = std::cos((turretCam.ty + 15.0) * (std::numbers::pi / 180.0)) * turretCam.distanceToTag;
          HoodedShooter.xOffset = std::sin(turretCam.tx * (std::numbers::pi / 180.0)) * HoodedShooter.distanceToTag;
          HoodedShooter.yOffset = std::cos(turretCam.tx * (std::numbers::pi / 180.0)) * HoodedShooter.distanceToTag;
          HoodedShooter.optimalRPM = HoodedShooter.findOptimalRPM();
          HoodedShooter.setHoodPosition(HoodedShooter.optimalRPM, HoodedShooter.xOffset, HoodedShooter.yOffset);

          frc::SmartDashboard::PutNumber("Shooter Distance (dx)", HoodedShooter.xOffset);
          frc::SmartDashboard::PutNumber("Shooter Distance (dy)", HoodedShooter.yOffset);

        }
      ),
      // Step 2: Spin up flywheel and wait 4 seconds, then feed while flywheel keeps spinning
      frc2::cmd::Parallel(
        frc2::cmd::StartEnd(
          [this] {
            HoodedShooter.setFlywheelSpeed(-HoodedShooter.optimalRPM);
          },
          [this] {
            HoodedShooter.ShooterMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
            HoodedShooter.moveHoodToZero();
          }
        ),
        frc2::cmd::Sequence(
          frc2::cmd::WaitUntil(
            [this] {
              return turretCam.hasTarget && 
                (HoodedShooter.getShooterVelocity() > 
                  (0.95 * (1/ShooterConstants::SHOOTEREFFICIENCY) * 
                  HoodedShooter.optimalRPM));
            }
          ),
          // Step 3: Run indexer and feeder while flywheel is still spinning
          frc2::cmd::RunOnce([this] {
            frc::SmartDashboard::PutString("Ran", "RAN INDEXER AND FEEDER");
          }),
          frc2::cmd::Parallel(
            frc2::cmd::Run(
              [this] {
                frc::SmartDashboard::PutNumber("Feeder Ideal RPM", HoodedShooter.optimalRPM * (1/ShooterConstants::SHOOTEREFFICIENCY));
                frc::SmartDashboard::PutNumber("Feeder RPM", BallFeeder.getFeederSpeed());
              }
            ),
            BallIndexer.RunIndexer(&BallIndexer, -3000),
            frc2::cmd::StartEnd(
              [this]
              {
                BallFeeder.setFeederSpeed( -HoodedShooter.optimalRPM * (1/ShooterConstants::SHOOTEREFFICIENCY));
              },
              [this]
              {
                BallFeeder.stopMotor();
              }
            )
          )
        )
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

  // Co-Driver Manual Hood Movement (NEEDS TO BE TESTED)
  (codriverCtr.R1() && rightStickYMoving).WhileTrue(
    HoodedShooter.Run(
      [this] {
        // Make separate turret slew rate limiter if needed
        double rightY = frc::ApplyDeadband(codriverCtr.GetRightY(), ControllerConstants::deadband);
        rightY = yLimiter.Calculate(rightY);

        frc::SmartDashboard::PutNumber("Shooter Hood Right Y", rightY);

        if (rightY > 0.5) HoodedShooter.setManualHoodPosition(HoodedShooter.findHoodAngle()+1.0);
        else if (rightY < -0.5) HoodedShooter.setManualHoodPosition(HoodedShooter.findHoodAngle() - 1.0);
      }
    )
  );

  // Enable/disable automatic turret targeting (QuestNav + hub equations)
  codriverCtr.POVUp().OnTrue(
    frc2::cmd::RunOnce(
      [this] {
        autoTargeting = !autoTargeting;
      }
    )
  );

  // Rotate Turret Left
  codriverCtr.POVLeft().WhileTrue(
    m_turret.StartEnd(
      [this] { m_turret.setSpeed(-0.1); },
      [this] { m_turret.stop(); }
    )
  );

  // Rotate Turret Right
  codriverCtr.POVRight().WhileTrue(
    m_turret.StartEnd(
      [this] { m_turret.setSpeed(0.1); },
      [this] { m_turret.stop(); }
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