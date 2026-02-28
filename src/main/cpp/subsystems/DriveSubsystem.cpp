// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"
#include "frc/smartdashboard/SmartDashboard.h"

DriveSubsystem::DriveSubsystem() {}

//fieldRelative always true in swervedrive
void DriveSubsystem::Drive(double vx, double vy, double rot, bool fieldRelative, GyroType gyro) {
  frc::ChassisSpeeds speeds;
  frc::Rotation2d rot2d;

  if (gyro == GyroType::QuestNav) {
    rot2d = QuestNav::getInstance().getRotation2d();
  }
  else if (gyro == GyroType::Pigeon) {
    rot2d = pigeon.GetRotation2d();
  }
  
  if (fieldRelative) {
    speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(units::meters_per_second_t(vx), units::meters_per_second_t(vy), units::radians_per_second_t(rot), rot2d);
  }
  else {
    speeds = frc::ChassisSpeeds{units::meters_per_second_t(vx), units::meters_per_second_t(vy), units::radians_per_second_t(rot)};
    frc::SmartDashboard::PutBoolean("ROBOT CENTRIC?", !fieldRelative);
  }

  auto states = kinematics.ToSwerveModuleStates(speeds);
  kinematics.DesaturateWheelSpeeds(&states, units::meters_per_second_t(ModuleConstants::moduleMaxMPS));

  frontLeft.setDesiredState(states[1]);
  frontRight.setDesiredState(states[3]);
  backLeft.setDesiredState(states[0]);
  backRight.setDesiredState(states[2]);
}

void DriveSubsystem::updateOdometry(GyroType gyro) {
  frc::Rotation2d rot2d;

  switch (gyro) {
    case GyroType::QuestNav:
      rot2d = QuestNav::getInstance().getRotation2d();
    case GyroType::Pigeon:
      rot2d = pigeon.GetRotation2d();
  };

  odometry.Update(
    rot2d, 
    { 
      getModulePositions()
    }
  );
}

//resets origin
void DriveSubsystem::resetOdometry(frc::Pose2d pose, GyroType gyro) {
  frc::Rotation2d rot2d;

  switch (gyro) {
    case GyroType::QuestNav:
      rot2d = QuestNav::getInstance().getRotation2d();
    case GyroType::Pigeon:
      rot2d = pigeon.GetRotation2d();
  };

  odometry.ResetPosition(
    rot2d,
    {
      getModulePositions()
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

  // frontLeft.invertModule(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive, false, true);
  backRight.invertModule(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive, false, true);
}

void DriveSubsystem::resetGyro(GyroType gyro) {
  switch (gyro) {
    case GyroType::QuestNav:
      QuestNav::getInstance().ZeroGyro();

    case GyroType::Pigeon:
      pigeon.Reset();
  };
}

bool DriveSubsystem::gyroConnected(GyroType gyro) {
  if (gyro == GyroType::QuestNav) {
    return QuestNav::getInstance().isConnected();
  }
  else if (gyro == GyroType::Pigeon) {
    return pigeon.IsConnected();
  }
  else {
    return false;
  }
}

void DriveSubsystem::stopAllModules() {
  frontLeft.stopModule();
  frontRight.stopModule();
  backLeft.stopModule();
  backRight.stopModule();
}

wpi::array<frc::SwerveModulePosition, 4> DriveSubsystem::getModulePositions() {
  return wpi::array<frc::SwerveModulePosition, 4> {
    frontLeft.getPosition(),
    frontRight.getPosition(),
    backLeft.getPosition(),
    backRight.getPosition()
  };
}

void DriveSubsystem::initGyro() {
  ctre::phoenix6::configs::Pigeon2Configuration pigeonConfigs{};
  pigeon.ClearStickyFaults();

  pigeonConfigs.MountPose.MountPosePitch = units::degree_t(0.0);
  pigeonConfigs.MountPose.MountPoseRoll = units::degree_t(0.0);
  pigeonConfigs.MountPose.MountPoseYaw = units::degree_t(0.0);

  pigeonConfigs.Pigeon2Features.EnableCompass = true;
  pigeonConfigs.FutureProofConfigs = false;

  pigeon.GetConfigurator().Apply(pigeonConfigs);

  pigeon.Reset();
  pigeon.ClearStickyFaults();
}

frc::Rotation2d DriveSubsystem::getActiveGyroRotation(GyroType gyro) {
  if (gyro == GyroType::QuestNav) {
    units::radian_t questCorrection = QuestNav::getInstance().getPose2d().Rotation().Radians() + questOffset;
    return frc::Rotation2d(questCorrection);
  }
  else if (gyro == GyroType::Pigeon) {
    return frc::Rotation2d(units::radian_t(std::fmod(pigeon.GetYaw().GetValueAsDouble(), 360.0) * (std::numbers::pi / 180.0)) + pigeonOffset);
  }
}

void DriveSubsystem::syncAndSwitchToPigeon() {
  frc::Pose2d currentPose = getPose();
  double rawPigeonYawDeg = std::fmod(pigeon.GetYaw().GetValueAsDouble(), 360.0); 
  double rawPigeonYawRad = rawPigeonYawDeg * (std::numbers::pi / 180.0);
  pigeonOffset = units::radian_t(currentPose.Rotation().Radians().value() - rawPigeonYawRad);

  pigeon.SetYaw(pigeonOffset);

  odometry.ResetPosition(
    getActiveGyroRotation(GyroType::Pigeon),
    getModulePositions(),
    currentPose
  );
}

void DriveSubsystem::syncAndSwitchToQuest() {
  frc::Pose2d currentPose = getPose();
  units::radian_t questHeading = QuestNav::getInstance().getPose2d().Rotation().Radians();
  units::radian_t maintainedHeading = currentPose.Rotation().Radians();
  questOffset = maintainedHeading - questHeading;

  QuestNav::getInstance().ZeroGyro(questOffset.value());

  odometry.ResetPosition(
    getActiveGyroRotation(GyroType::QuestNav),
    getModulePositions(),
    currentPose
  );
}

ctre::phoenix6::hardware::Pigeon2& DriveSubsystem::getPigeon() {
  return pigeon;
}