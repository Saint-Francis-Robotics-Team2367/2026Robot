// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include "frc/kinematics/ChassisSpeeds.h"
#include <units/time.h>
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "frc/estimator/SwerveDrivePoseEstimator.h"

#include "frc/geometry/Translation2d.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"

#include "subsystems/SwerveModule.h"

class DriveSubsystem : public frc2::SubsystemBase {
public:
  DriveSubsystem();

  void Drive(double vx, double vy, double rot, bool fieldRelative);
  void driveRobotRelative(frc::ChassisSpeeds speeds);
  void initModules();
  void configurePathPlanner();
  void updateOdometry();
  void resetOdometry(frc::Pose2d pose);
  frc::SwerveDrivePoseEstimator<4> getPoseEstimator();
  
  frc::Pose2d getPose();
  frc::ChassisSpeeds getRobotRelativeSpeeds();
  frc::Rotation2d getHeading();
  void resetGyro();
  bool gyroConnected();
  void stopAllModules();
  void initGyro();
  void AddVisionMeasurement(frc::Pose2d pose, units::second_t timestamp);

private:
  void integrateSimPose(const frc::ChassisSpeeds& speeds);

  //module objects
  SwerveModule frontLeft{HardwareIDs::FLdriveID, HardwareIDs::FLsteerID, HardwareIDs::FLencoderID, "Drivetrain"};
  SwerveModule frontRight{HardwareIDs::FRdriveID, HardwareIDs::FRsteerID, HardwareIDs::FRencoderID, "Drivetrain"};
  SwerveModule backLeft{HardwareIDs::BLdriveID, HardwareIDs::BLsteerID, HardwareIDs::BLencoderID, "Drivetrain"};
  SwerveModule backRight{HardwareIDs::BRdriveID, HardwareIDs::BRsteerID, HardwareIDs::BRencoderID, "Drivetrain"};

    //CHANGE THESE IF ROBOT DIMENSIONS CHANGE; positions of swerve modules relative to robot
  frc::SwerveDriveKinematics<4> kinematics {
    frc::Translation2d{0.381_m, 0.381_m}, 
    frc::Translation2d{0.381_m, -0.381_m},
    frc::Translation2d{-0.381_m, 0.381_m},
    frc::Translation2d{-0.381_m, -0.381_m} 
  };

  // Desktop sim has no QuestNav / real encoders — integrate commanded speeds instead.
  frc::Pose2d m_simPose{0_m, 0_m, 0_deg};
  frc::ChassisSpeeds m_lastSpeeds{};
  units::second_t m_lastSimTimestamp{0_s};

public:
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