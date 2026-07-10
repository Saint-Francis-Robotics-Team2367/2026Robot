// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <frc/Timer.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>

using namespace pathplanner;

DriveSubsystem::DriveSubsystem() {}

frc::Rotation2d DriveSubsystem::getHeading() {
  if (frc::RobotBase::IsSimulation()) {
    return m_simPose.Rotation();
  }
  return QuestNav::getInstance().getRotation2d();
}

void DriveSubsystem::integrateSimPose(const frc::ChassisSpeeds& speeds) {
  units::second_t now = frc::Timer::GetFPGATimestamp();
  units::second_t dt = now - m_lastSimTimestamp;
  m_lastSimTimestamp = now;

  if (dt <= 0_s || dt > 0.1_s) {
    dt = 20_ms;
  }

  m_lastSpeeds = speeds;
  m_simPose = m_simPose.Exp(frc::Twist2d{speeds.vx * dt, speeds.vy * dt, speeds.omega * dt});
}

//fieldRelative always true in swervedrive
void DriveSubsystem::Drive(double vx, double vy, double rot, bool fieldRelative) {
  frc::ChassisSpeeds speeds;
  
  if (fieldRelative) {
    speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        units::meters_per_second_t(vx), units::meters_per_second_t(vy),
        units::radians_per_second_t(rot), getHeading());
  }
  else {
    speeds = frc::ChassisSpeeds{units::meters_per_second_t(vx), units::meters_per_second_t(vy), units::radians_per_second_t(rot)};
  }

  if (frc::RobotBase::IsSimulation()) {
    integrateSimPose(speeds);
  }

  auto states = kinematics.ToSwerveModuleStates(speeds);
  kinematics.DesaturateWheelSpeeds(&states, units::meters_per_second_t(ModuleConstants::moduleMaxMPS));

  frontLeft.setDesiredState(states[1]);
  frontRight.setDesiredState(states[3]);
  backLeft.setDesiredState(states[0]);
  backRight.setDesiredState(states[2]);
}

void DriveSubsystem::driveRobotRelative(frc::ChassisSpeeds speeds) {
  Drive(speeds.vx.value(), speeds.vy.value(), speeds.omega.value(), false);
}

frc::ChassisSpeeds DriveSubsystem::getRobotRelativeSpeeds() {
  if (frc::RobotBase::IsSimulation()) {
    return m_lastSpeeds;
  }
  return kinematics.ToChassisSpeeds(
      frontLeft.getState(),
      frontRight.getState(),
      backLeft.getState(),
      backRight.getState());
}

void DriveSubsystem::configurePathPlanner() {
  RobotConfig config = RobotConfig::fromGUISettings();

  AutoBuilder::configure(
      [this] { return getPose(); },
      [this](frc::Pose2d pose) { resetOdometry(pose); },
      [this] { return getRobotRelativeSpeeds(); },
      [this](frc::ChassisSpeeds speeds) { driveRobotRelative(speeds); },
      std::make_shared<PPHolonomicDriveController>(
          PIDConstants(5.0, 0.0, 0.0),
          PIDConstants(5.0, 0.0, 0.0)),
      config,
      [] {
        auto alliance = frc::DriverStation::GetAlliance();
        return alliance && *alliance == frc::DriverStation::Alliance::kRed;
      },
      this);
}

void DriveSubsystem::updateOdometry() {
  if (frc::RobotBase::IsSimulation()) {
    return;
  }

  odometry.Update(
    getHeading(), 
    { 
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
    });
}

//resets origin
void DriveSubsystem::resetOdometry(frc::Pose2d pose) {
  m_simPose = pose;
  m_lastSpeeds = {};
  m_lastSimTimestamp = frc::Timer::GetFPGATimestamp();

  odometry.ResetPosition(
    getHeading(),
    {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
    },
    pose
  );
  if (!frc::RobotBase::IsSimulation()) {
    QuestNav::getInstance().CalibrateToFieldPose(pose);
  }
}

frc::SwerveDrivePoseEstimator<4> DriveSubsystem::getPoseEstimator() {
  return odometry;
}

//gets robot position
frc::Pose2d DriveSubsystem::getPose() {
  if (frc::RobotBase::IsSimulation()) {
    return m_simPose;
  }
  return odometry.GetEstimatedPosition();
}

void DriveSubsystem::AddVisionMeasurement(frc::Pose2d pose, units::second_t timestamp) {
  odometry.AddVisionMeasurement(pose, timestamp);
}

//initializes swerve modules
void DriveSubsystem::initModules() {
  frontLeft.initHardware();
  frontRight.initHardware();
  backLeft.initHardware();
  backRight.initHardware();

  frontLeft.zeroModule();
  frontRight.zeroModule();
  backLeft.zeroModule();
  backRight.zeroModule();

  // backRight.invertModule(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive, false, true);
  // backLeft.invertModule(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive, false, true);
  frontLeft.invertModule(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive, false, true);
  frontRight.invertModule(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive, false, true);
}

void DriveSubsystem::resetGyro() {
  if (frc::RobotBase::IsSimulation()) {
    m_simPose = frc::Pose2d{m_simPose.Translation(), frc::Rotation2d{}};
    return;
  }
  QuestNav::getInstance().ZeroGyro();
}

bool DriveSubsystem::gyroConnected() {
  if (frc::RobotBase::IsSimulation()) {
    return true;
  }
  return QuestNav::getInstance().isConnected();
}

void DriveSubsystem::stopAllModules() {
  frontLeft.stopModule();
  frontRight.stopModule();
  backLeft.stopModule();
  backRight.stopModule();
  m_lastSpeeds = {};
}

//initializes gyro and sets current gyro situation to zero
void DriveSubsystem::initGyro() {
}
