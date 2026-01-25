// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

DriveSubsystem::DriveSubsystem() {
  // Implementation of subsystem constructor goes here.
}

void DriveSubsystem::Drive(double vx, double vy, double rot, bool fieldRelative) {
  frc::ChassisSpeeds speeds;
  
  if (fieldRelative) {
    speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(units::meters_per_second_t(vx), units::meters_per_second_t(vy), units::radians_per_second_t(rot), pigeon.GetRotation2d());
  }
  else {
    speeds = frc::ChassisSpeeds{units::meters_per_second_t(vx), units::meters_per_second_t(vy), units::radians_per_second_t(rot)};
  }

  auto states = kinematics.ToSwerveModuleStates(speeds);
  kinematics.DesaturateWheelSpeeds(&states, units::meters_per_second_t(ModuleConstants::moduleMaxMPS));

  frontLeft.setDesiredState(states[0]);
  frontRight.setDesiredState(states[1]);
  backLeft.setDesiredState(states[2]);
  backRight.setDesiredState(states[3]);
}

void DriveSubsystem::updateOdometry() {
  odometry.Update(
    pigeon.GetRotation2d(), 
    { 
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
    });
}

void DriveSubsystem::resetOdometry(frc::Pose2d pose) {
  odometry.ResetPosition(
    pigeon.GetRotation2d(),
    {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
    },
    pose
  );
}

frc::Pose2d DriveSubsystem::getPose() {
  return odometry.GetEstimatedPosition();
}

void DriveSubsystem::initModules() {
  frontLeft.initHardware();
  frontRight.initHardware();
  backLeft.initHardware();
  backRight.initHardware();

  frontLeft.setMotor(SwerveModule::control::POSITION, SwerveModule::MotorType::STEER, 0.0 - frontLeft.moduleOffset);
  frontRight.setMotor(SwerveModule::control::POSITION, SwerveModule::MotorType::STEER, 0.0 - frontRight.moduleOffset);
  backLeft.setMotor(SwerveModule::control::POSITION, SwerveModule::MotorType::STEER, 0.0 - backLeft.moduleOffset);
  backRight.setMotor(SwerveModule::control::POSITION, SwerveModule::MotorType::STEER, 0.0 - backRight.moduleOffset);
}

void DriveSubsystem::resetGyro() {
  pigeon.Reset();
}

bool DriveSubsystem::gyroConnected() {
  pigeon.IsConnected();
}

void DriveSubsystem::stopAllModules() {
  frontLeft.stopModule();
  frontRight.stopModule();
  backLeft.stopModule();
  backRight.stopModule();
}

void DriveSubsystem::initGyro() {
  ctre::phoenix6::configs::Pigeon2Configuration pigeonConfigs{};

  pigeonConfigs.MountPose.MountPosePitch = units::degree_t(0.0);
  pigeonConfigs.MountPose.MountPoseRoll = units::degree_t(0.0);
  pigeonConfigs.MountPose.MountPoseYaw = units::degree_t(0.0);

  pigeonConfigs.Pigeon2Features.EnableCompass = true;
  pigeonConfigs.FutureProofConfigs = false;

  pigeon.GetConfigurator().Apply(pigeonConfigs);

  pigeon.Reset();
  pigeon.ClearStickyFaults();
}