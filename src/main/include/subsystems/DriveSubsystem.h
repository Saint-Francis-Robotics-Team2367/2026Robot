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
#include "ctre/phoenix6/Pigeon2.hpp"

class DriveSubsystem : public frc2::SubsystemBase {
public:
  DriveSubsystem();

  void Drive(double vx, double vy, double rot, bool fieldRelative);
  void initModules();
  void updateOdometry();
  void resetOdometry(frc::Pose2d pose);
  frc::Pose2d getPose();
  void resetGyro();
  bool gyroConnected();
  void stopAllModules();
  void initGyro();

private:
  ctre::phoenix6::hardware::Pigeon2 pigeon{HardwareIDs::pigeonID};

  SwerveModule frontLeft{HardwareIDs::FLdriveID, HardwareIDs::FLsteerID, HardwareIDs::FLencoderID};
  SwerveModule frontRight{HardwareIDs::FRdriveID, HardwareIDs::FRsteerID, HardwareIDs::FRencoderID};
  SwerveModule backLeft{HardwareIDs::BLdriveID, HardwareIDs::BLsteerID, HardwareIDs::BLencoderID};
  SwerveModule backRight{HardwareIDs::BRdriveID, HardwareIDs::BRsteerID, HardwareIDs::BRencoderID};

  frc::SwerveDriveKinematics<4> kinematics {
    frc::Translation2d{0.381_m, 0.381_m}, 
    frc::Translation2d{0.381_m, -0.381_m},
    frc::Translation2d{-0.381_m, 0.381_m},
    frc::Translation2d{-0.381_m, -0.381_m} 
  };

  frc::SwerveDrivePoseEstimator<4> odometry{
        kinematics, 
        pigeon.GetRotation2d(), 
        { 
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        },
        frc::Pose2d{0_m, 0_m, 0_deg} 
  };
};
