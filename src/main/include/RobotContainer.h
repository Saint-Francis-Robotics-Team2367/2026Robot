// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include "frc2/command/Commands.h"
#include <frc2/command/button/CommandPS5Controller.h>
#include "frc/smartdashboard/SendableChooser.h"

#include "Constants.h"

#include "subsystems/Shooter.h"
#include "subsystems/Spindexer.h"
#include "subsystems/Intake/RunIntake.h"
#include "subsystems/Intake/DeployIntake.h"

#include "subsystems/DriveSubsystem.h"

#include "frc/filter/SlewRateLimiter.h"
#include "frc/MathUtil.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "subsystems/Turret.h"
#include "subsystems/vision/PhotonVision.h"
#include "subsystems/vision/Limelight.h"

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
  Limelight turretCam{drivetrain};
  Turret m_turret = Turret(drivetrain);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc2::CommandPS5Controller driverCtr{0}; //drive controller should be first controller that is plugged in
  frc2::CommandPS5Controller codriverCtr{1};

  // The robot's subsystems are defined here...
  Shooter HoodedShooter;
  Spindexer mSpindexer;
  RunIntake mRunIntake;
  DeployIntake mDeployIntake;

  bool autoTargeting = false;

  // ******************** Manual (no-Limelight) shoot state ********************
  // True while the generic shoot sequence is running. Gates the co-driver D-pad
  // over to hood/turret trim and holds off the vision-based turret tracking.
  bool manualShooting = false;

  // Turret angle at boot, captured after Turret::init() zeroes the motor. The
  // manual sequence clamps the turret to +/- manualTurretRange around this.
  double turretBootAngle = 0.0;

  // Live setpoints trimmed by the D-pad during the manual sequence.
  double manualHoodAngle = ManualShootConstants::startingHoodAngle;
  double manualTurretAngle = 0.0;

  frc::SendableChooser<std::string> positionChooser;
  const std::string topTrench = "Top Trench";
  const std::string topBump = "Top Bump";
  const std::string frontHub = "Front Hub";
  const std::string bottomBump = "Bottom Bump";
  const std::string bottomTrench = "Bottom Trench";

  frc::SendableChooser<std::string> allianceChooser;
  const std::string blueAlliance = "Blue Alliance";
  const std::string redAlliance = "Red Alliance";

  frc::Pose2d startPose{0_m, 0_m, 0_rad};
  std::string allianceColor;
  std::string fieldPosition;

  double startXOffset = 0.0;
  double hubPoseX = 0.0;
  int noTagVisibleCounter = 0;
  int settings = 1;
  double spindexerSpeed = 0.0;

  frc::SlewRateLimiter<units::scalar> xLimiter{ControllerConstants::slewRate / 1_s};
  frc::SlewRateLimiter<units::scalar> yLimiter{ControllerConstants::slewRate / 1_s};
  frc::SlewRateLimiter<units::scalar> rotLimiter{ControllerConstants::slewRate / 1_s};

  void ConfigureBindings();
  void InitializeStartPose();

  // Generic shoot sequence that does not depend on the Limelight: spin the
  // flywheel to a fixed RPM, wait for 95% of it, then feed. Hood and turret are
  // trimmed by the co-driver D-pad while it runs.
  frc2::CommandPtr ManualShootCommand();

  void TrimManualHood(bool up);
  void TrimManualTurret(double deltaDegrees);
};
