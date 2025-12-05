
#pragma once

#include <frc/TimedRobot.h>
#include <frc/PS5Controller.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "util/ControlUtil.h"
#include "swerve/SwerveHeadingController.h"
#include "sensors/Limelight.h"
#include "util/SlewRateLimiter.h"
#include "control/PowerModule.h"
#include "SwerveDrive.h"

#include "swerve/SwerveAlign.h"
#include <ctre/phoenix6/CANBus.hpp>
#include "Trajectory.h"

#include <ctre/phoenix6/Pigeon2.hpp>
#include "sensors/Pigeon.h"
#include "sensors/LED.h"
#include "sensors/PhotonVision.h"

class Robot : public frc::TimedRobot
{
public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  void SimulationInit() override;
  void SimulationPeriodic() override;

  // Modules/Devices
  frc::PS5Controller ctr = frc::PS5Controller(0);

  pathplanner::RobotConfig pathConfig = pathplanner::RobotConfig::fromGUISettings();
  frc::SendableChooser<Trajectory::autos> mChooser;

  // Vision
  PhotonVision cameraFront = PhotonVision("cameraFront");
  PhotonVision cameraBack = PhotonVision{"cameraBack"};

  // For Auto Align
  SwerveAlign align;
  float setpointX = 0;
  float setpointY = 0;

  // Pigeon
  Pigeon pigeon{0};

  SwerveDrive mDrive = SwerveDrive(pigeon);

  Trajectory mTrajectory = Trajectory(mDrive, mHeadingController, cameraFront, cameraBack, align, pigeon, pathConfig);

  //CANivore
  // ctre::phoenix6::CANBus canbus{"Drivetrain"};
  // ctre::phoenix6::CANBus::CANBusStatus canInfo = canbus.GetStatus();
  // float busUtil = canInfo.BusUtilization;

  // Teleop Controls
  float ctrPercent = 1.0;
  float boostPercent = 0.9;
  double ctrPercentAim = 0.3;
  bool scoreAmp = false;
  bool liftElev = false;
  bool cleanDriveAccum = true;
  double speedLimiter = 1.0;

  // Controllers
  SwerveHeadingController mHeadingController = SwerveHeadingController(-4.0, 4.0);
  SlewRateLimiter xStickLimiter = SlewRateLimiter(ctrSlewRate);
  SlewRateLimiter yStickLimiter = SlewRateLimiter(ctrSlewRate);

  //autochooser
  frc::SendableChooser<std::string> positionChooser;
  const std::string kSimpleAuto = "0";
  const std::string kAutoStartDefault = "1";
  const std::string kAutoStartB = "2";
  const std::string kAutoStartC = "3";

  frc::SendableChooser<std::string> reefChooser;
  const::std::string kAutoReefDefault = "A";
  const::std::string kAutoReefB = "B";
  const::std::string kAutoReefC = "C";
  const::std::string kAutoReefD = "D";
  const::std::string kAutoReefE = "E";
  const::std::string kAutoReefF = "F";
  const std::string kOneCoral = "0";

  int coralLevel = 0;
  std::string elevatorLevel = "Start";
  std::string coralSide = "right";
  bool scorecoral = true;

  // Alliance Color and Chooser
  frc::SendableChooser<std::string> allianceChooser;
  const std::string redAlliance = "RED";
  const std::string blueAlliance = "BLUE";
  bool allianceIsRed = false;

  Led mLED{9, 43};
};
