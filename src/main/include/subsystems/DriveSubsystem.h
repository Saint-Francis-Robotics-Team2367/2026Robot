// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "frc/estimator/SwerveDrivePoseEstimator.h"

#include "frc/geometry/Translation2d.h"
#include "frc/geometry/Pose2d.h"

#include "subsystems/SwerveModule.h"

#include "subsystems/vision/QuestNav.h"
#include "ctre/phoenix6/Pigeon2.hpp"

class DriveSubsystem : public frc2::SubsystemBase {
public:
  enum GyroType {
    QuestNav,
    Pigeon
  };

  DriveSubsystem();

  void Drive(double vx, double vy, double rot, bool fieldRelative, GyroType gyro);
  void initModules();
  void updateOdometry(GyroType gyro);
  void resetOdometry(frc::Pose2d pose, GyroType gyro);
  frc::Pose2d getPose();
  void resetGyro(GyroType gyro);
  bool gyroConnected(GyroType gyro);
  void stopAllModules();
  void initGyro();
  frc::Rotation2d getActiveGyroRotation(GyroType gyro);
  void syncAndSwitchToPigeon();
  void syncAndSwitchToQuest();
  wpi::array<frc::SwerveModulePosition, 4> getModulePositions();
  ctre::phoenix6::hardware::Pigeon2& getPigeon();

private:
  ctre::phoenix6::hardware::Pigeon2 pigeon{HardwareIDs::pigeonID};
  units::radian_t questOffset = 0_rad;
  units::radian_t pigeonOffset = 0_rad;

  //module objects
  SwerveModule frontLeft{HardwareIDs::FLdriveID, HardwareIDs::FLsteerID, HardwareIDs::FLencoderID};
  SwerveModule frontRight{HardwareIDs::FRdriveID, HardwareIDs::FRsteerID, HardwareIDs::FRencoderID};
  SwerveModule backLeft{HardwareIDs::BLdriveID, HardwareIDs::BLsteerID, HardwareIDs::BLencoderID};
  SwerveModule backRight{HardwareIDs::BRdriveID, HardwareIDs::BRsteerID, HardwareIDs::BRencoderID};

  //CHANGE THESE IF ROBOT DIMENSIONS CHANGE; positions of swerve modules relative to robot
  frc::SwerveDriveKinematics<4> kinematics {
    frc::Translation2d{0.381_m, 0.381_m}, 
    frc::Translation2d{0.381_m, -0.381_m},
    frc::Translation2d{-0.381_m, 0.381_m},
    frc::Translation2d{-0.381_m, -0.381_m} 
  };

  frc::SwerveDrivePoseEstimator<4> odometry{
        kinematics, 
        QuestNav::getInstance().getRotation2d(), 
        { 
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        },
        frc::Pose2d{0_m, 0_m, 0_deg}  //needs to be reset if starting on bump 
  };
};
