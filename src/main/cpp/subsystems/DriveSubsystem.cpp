// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"
#include "frc/smartdashboard/SmartDashboard.h"

DriveSubsystem::DriveSubsystem() {}

//fieldRelative always true in swervedrive
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

  frontLeft.setDesiredState(states[1]);
  frontRight.setDesiredState(states[3]);
  backLeft.setDesiredState(states[0]);
  backRight.setDesiredState(states[2]);
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

//resets origin
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

//gets robot position
frc::Pose2d DriveSubsystem::getPose() {
  return odometry.GetEstimatedPosition();
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

  frontLeft.invertModule(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive, false, true);
  backRight.invertModule(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive, false, true);
}

void DriveSubsystem::resetGyro() {
  pigeon.Reset();
}

bool DriveSubsystem::gyroConnected() {
  return pigeon.IsConnected();
}

void DriveSubsystem::stopAllModules() {
  frontLeft.stopModule();
  frontRight.stopModule();
  backLeft.stopModule();
  backRight.stopModule();
}

//initializes gyro and sets current gyro situation to zero
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

frc::ChassisSpeeds DriveSubsystem::getRobotRelativeSpeeds() {
    frc::SwerveModuleState fl = frontLeft.getState();
    frc::SwerveModuleState fr = frontRight.getState();
    frc::SwerveModuleState bl = backLeft.getState();
    frc::SwerveModuleState br = backRight.getState();
    return kinematics.ToChassisSpeeds(fl, fr, bl, br);
}