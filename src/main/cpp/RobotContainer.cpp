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
  mSpindexer.init(); // Initialize Indexer motors and encoders
  m_turret.init();
  // Turret::init() zeroes the motor position, so the boot reference angle is 0 by
  // definition. Don't read it back off the encoder here — SetPosition is async over
  // CAN and an immediate read can still return the pre-zero value.
  turretBootAngle = 0.0;
  manualTurretAngle = turretBootAngle;
  mDeployIntake.init();
  mRunIntake.init();

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

  frc2::Trigger SpindexerStall(
    [&] {
      return mSpindexer.SpindexerStall();
    }
  );

  frc2::Trigger scorePossible{
    [&] {
      return drivetrain.getPose().Y().value() < 4.625594;
    }
  };

  // True while the generic (no-Limelight) shoot sequence is running. Hands the
  // co-driver D-pad over to hood/turret trim and holds off vision turret tracking.
  frc2::Trigger manualShootActive(
    [&] {
      return manualShooting;
    }
  );

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

        drivetrain.Drive(-vx, vy, rot, drivetrain.gyroConnected());
      }
    )
  );

  (turretAutoTargetingOn && !manualShootActive).WhileTrue(
    frc2::cmd::Run(
      [this] {
        if (turretCam.hasTarget)
        {
          double tx = std::clamp(turretCam.tx + m_turret.getCurrentMotorAngle(), -92.5, 92.5);
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

  // Don't yank the turret back to zero mid-manual-shot: losing the tag is expected
  // there, since the manual sequence doesn't use the camera at all.
  ((!turretAutoTargetingOn || turretNoAprilTagDetected) && !manualShootActive).OnTrue(
    frc2::cmd::RunOnce(
      [this] {
        m_turret.setAngle(0);
      }
    )
  );

  // (turretAutoTargetingOn).WhileTrue(
  //   frc2::cmd::Run(
  //     [this] {
  //       double x = drivetrain.getPose().X().value();
  //       double y = drivetrain.getPose().Y().value();
  //       double yOffset = PoseConstants::BluehubX - y; 
  //       double xOffset = PoseConstants::hubPoseY - x;
  //       double turnAmt = std::atan(yOffset / xOffset) * (180.0 / std::numbers::pi);
  //       double tx = std::clamp(turnAmt + m_turret.getCurrentMotorAngle(), -50.0, 50.0);
  //       tx = frc::ApplyDeadband(tx, TurretConstants::turretDeadband);
  //       frc::SmartDashboard::PutNumber("turret angle", tx);
  //       m_turret.setAngle(tx);
  //     }
  //   )
  // );

  // Detect Indexer Stall
  // SpindexerStall.OnTrue(
  //   frc2::cmd::Sequence(
  //     frc2::cmd::RunOnce(
  //       [this] {
  //         frc::SmartDashboard::PutBoolean("Spindexer Stall", true);
  //       }
  //     ),
  //     frc2::cmd::Sequence(
  //       mSpindexer.RunSpindexer(&mSpindexer, -2200),
  //       frc2::cmd::Wait(0.75_s),
  //       mSpindexer.RunSpindexer(&mSpindexer, 2200),
  //       frc2::cmd::Wait(0.75_s)
  //     ).WithTimeout(2.5_s),
  //     frc2::cmd::RunOnce(
  //       [this] {
  //         frc::SmartDashboard::PutBoolean("Spindexer Stall", false);
  //       }
  //     ),
  //     frc2::cmd::Wait(0.25_s)
  //   )
  // );

  // ******************** Driver Controls ********************
  // Zero Gyro
  driverCtr.Cross().OnTrue(
    frc2::cmd::RunOnce(
      [this] {QuestNav::getInstance().ZeroGyro();}
    )
  );

  // Generic shoot sequence — no Limelight required. Hood/turret trimmed on the
  // co-driver D-pad while this runs.
  driverCtr.Square().ToggleOnTrue(
    ManualShootCommand()
  );

  // Run Intake
  driverCtr.R1().ToggleOnTrue(
    mRunIntake.IntakeCommand(&mRunIntake, 3000)
  );

  // Outtake Intake
  driverCtr.L1().ToggleOnTrue(
    mRunIntake.IntakeCommand(&mRunIntake, -3000)
  );

  driverCtr.R2().ToggleOnTrue(
    mDeployIntake.masterIntakeCommand(&mDeployIntake, false)
  );

  // ******************** Co-Driver Controls ********************
  // Reverse Indexer and Feeder
  codriverCtr.L2().WhileTrue(
    frc2::cmd::Parallel(
      mSpindexer.RunSpindexer(&mSpindexer, -1 * spindexerSpeed)
    )
  );

  (codriverCtr.POVDown() && !manualShootActive).OnTrue(
    frc2::cmd::RunOnce(
      [this] {
        settings = settings % 6;
        if (settings == 1) {
          spindexerSpeed = 2500;
        }
        else if (settings == 2) {
          spindexerSpeed = 3000;
        }
        else if (settings == 3) {
          spindexerSpeed = 3500;
        }
        else if (settings == 4) {
          spindexerSpeed = 4000;
        }
        else if (settings == 5) {
          spindexerSpeed = 4500;
        }
        else if (settings == 6) {
          spindexerSpeed = 5000;
        }
        settings++;
      }
    )
  );

  codriverCtr.L1().WhileTrue(
    frc2::cmd::Parallel(
      mSpindexer.RunSpindexer(&mSpindexer, spindexerSpeed)
    )
  );

  (codriverCtr.R2()).ToggleOnTrue(
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
            mSpindexer.RunSpindexer(&mSpindexer, 6250),
            mDeployIntake.masterIntakeCommand(&mDeployIntake, true)
          )
        )
      )
    )
  );

  // Square: Turret 45° left, Hood 45°, Flywheel 1000 RPM
  codriverCtr.Square().ToggleOnTrue(
    frc2::cmd::Sequence(
      frc2::cmd::Parallel(
        m_turret.RunOnce([this] { m_turret.setAngle(-45); }),
        HoodedShooter.RunOnce([this] { HoodedShooter.setManualHoodPosition(45); })
      ),
      frc2::cmd::Parallel(
        frc2::cmd::StartEnd(
          [this] { HoodedShooter.setFlywheelSpeed(1000); },
          [this] { HoodedShooter.ShooterMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0}); }
        ),
        mSpindexer.RunSpindexer(&mSpindexer, 6250)
      )
    )
  );

  // Manual Shoot: Hood 45°, Flywheel 2000 RPM
  codriverCtr.Cross().ToggleOnTrue(
    frc2::cmd::Parallel(
      HoodedShooter.RunOnce([this] { HoodedShooter.setManualHoodPosition(45); }),
      frc2::cmd::StartEnd(
        [this] { HoodedShooter.setFlywheelSpeed(2000); },
        [this] { HoodedShooter.ShooterMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0}); },
        {&HoodedShooter}
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
  (codriverCtr.POVUp() && !manualShootActive).OnTrue(
    frc2::cmd::RunOnce(
      [this] {
        autoTargeting = !autoTargeting;
      }
    )
  );

  // Rotate Turret Left
  (codriverCtr.POVLeft() && !manualShootActive).WhileTrue(
    m_turret.StartEnd(
      [this] { m_turret.setSpeed(-0.1); },
      [this] { m_turret.stop(); }
    )
  );

  // Rotate Turret Right
  (codriverCtr.POVRight() && !manualShootActive).WhileTrue(
    m_turret.StartEnd(
      [this] { m_turret.setSpeed(0.1); },
      [this] { m_turret.stop(); }
    )
  );

  // ******************** Manual Shoot D-pad Trim ********************
  // Active only while the generic shoot sequence runs, so the bindings above keep
  // their normal jobs the rest of the time.

  // Hood Up
  (codriverCtr.POVUp() && manualShootActive).WhileTrue(
    HoodedShooter.Run(
      [this] { TrimManualHood(true); }
    )
  );

  // Hood Down
  (codriverCtr.POVDown() && manualShootActive).WhileTrue(
    HoodedShooter.Run(
      [this] { TrimManualHood(false); }
    )
  );

  // Turret Left
  (codriverCtr.POVLeft() && manualShootActive).WhileTrue(
    m_turret.Run(
      [this] { TrimManualTurret(-TurretConstants::manualTurretStep); }
    )
  );

  // Turret Right
  (codriverCtr.POVRight() && manualShootActive).WhileTrue(
    m_turret.Run(
      [this] { TrimManualTurret(TurretConstants::manualTurretStep); }
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

// Trim the manual hood setpoint one step. `up` is the direction the operator asked
// for; ManualShootConstants::hoodUpIncreasesAngle maps that onto the angle sign,
// since the hood angle convention hasn't been confirmed on hardware yet.
void RobotContainer::TrimManualHood(bool up) {
  double step = ManualShootConstants::hoodStep;
  if (!ManualShootConstants::hoodUpIncreasesAngle) step = -step;
  if (!up) step = -step;

  manualHoodAngle = std::clamp(
    manualHoodAngle + step,
    ManualShootConstants::hoodMinAngle,
    ManualShootConstants::hoodMaxAngle
  );

  HoodedShooter.setManualHoodPosition(manualHoodAngle);
  frc::SmartDashboard::PutNumber("Manual Hood Angle", manualHoodAngle);
}

// Trim the manual turret setpoint, clamped to +/- manualTurretRange around wherever
// the turret sat at boot.
void RobotContainer::TrimManualTurret(double deltaDegrees) {
  manualTurretAngle = std::clamp(
    manualTurretAngle + deltaDegrees,
    turretBootAngle - TurretConstants::manualTurretRange,
    turretBootAngle + TurretConstants::manualTurretRange
  );

  m_turret.setAngle(manualTurretAngle);
  frc::SmartDashboard::PutNumber("Manual Turret Angle", manualTurretAngle);
}

// Generic shoot sequence with no Limelight involvement. Mirrors the structure of the
// co-driver R2 auto-shot, minus the vision math: fixed RPM, fixed starting hood angle,
// and the D-pad trims hood/turret live while it runs.
frc2::CommandPtr RobotContainer::ManualShootCommand() {
  return frc2::cmd::Parallel(
    // The manualShooting flag is set and cleared by this one StartEnd so the pair is
    // symmetric. Setting it in a preceding RunOnce would leave it latched true if the
    // command were interrupted before the clearing command ever started, which would
    // disable the normal D-pad bindings and vision tracking until a reboot.
    frc2::cmd::StartEnd(
      [this] {
        manualShooting = true;
        manualHoodAngle = ManualShootConstants::startingHoodAngle;
        manualTurretAngle = turretBootAngle;

        HoodedShooter.setManualHoodPosition(manualHoodAngle);
        m_turret.setAngle(manualTurretAngle);
        HoodedShooter.setFlywheelSpeed(
          ManualShootConstants::flywheelDirection * ManualShootConstants::flywheelRPM);

        frc::SmartDashboard::PutBoolean("Manual Shooting", true);
        frc::SmartDashboard::PutNumber("Manual Hood Angle", manualHoodAngle);
        frc::SmartDashboard::PutNumber("Manual Turret Angle", manualTurretAngle);
      },
      [this] {
        HoodedShooter.ShooterMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
        HoodedShooter.moveHoodToZero();
        manualShooting = false;
        frc::SmartDashboard::PutBoolean("Manual Shooting", false);
      }
    ),
    frc2::cmd::Sequence(
      // Wait for 95% of target RPM before feeding, so the first ball isn't shot
      // into a flywheel that's still spinning up. getShooterVelocity() is absolute,
      // so this holds regardless of the commanded direction.
      frc2::cmd::WaitUntil(
        [this] {
          return HoodedShooter.getShooterVelocity() >
            (ManualShootConstants::flywheelReadyFraction *
              (1 / ShooterConstants::SHOOTEREFFICIENCY) *
              ManualShootConstants::flywheelRPM);
        }
      ),
      // Feed. Spindexer runs until the operator toggles the sequence off.
      mSpindexer.RunSpindexer(&mSpindexer, ManualShootConstants::spindexerRPM)
    )
  );
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  //return autos::ExampleAuto(&m_subsystem);
}