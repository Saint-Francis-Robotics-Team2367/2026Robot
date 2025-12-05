// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <string>
#include <cameraserver/CameraServer.h>

void Robot::RobotInit()
{
  mDrive.initModules();
  pigeon.init();

  // frc::CameraServer::StartAutomaticCapture();

  // Choosers
  allianceChooser.SetDefaultOption("Red Alliance", redAlliance);
  allianceChooser.AddOption("Blue Alliance", blueAlliance);
  frc::SmartDashboard::PutData("Alliance Color", &allianceChooser);

  // Determines alliance color
  std::string allianceColor = allianceChooser.GetSelected();
  if (allianceColor == "RED") {
    allianceIsRed = true;
  }
  else {
    allianceIsRed = false;
  }

  positionChooser.SetDefaultOption(kAutoStartDefault, kAutoStartDefault);
  positionChooser.AddOption(kAutoStartB, kAutoStartB);
  positionChooser.AddOption(kAutoStartC, kAutoStartC);
  positionChooser.AddOption(kSimpleAuto, kSimpleAuto);
  frc::SmartDashboard::PutData("Auto Start Position", &positionChooser);

  reefChooser.SetDefaultOption(kAutoReefDefault, kAutoReefDefault);
  reefChooser.AddOption(kAutoReefB, kAutoReefB);
  reefChooser.AddOption(kAutoReefC, kAutoReefC);
  reefChooser.AddOption(kAutoReefD, kAutoReefD);
  reefChooser.AddOption(kAutoReefE, kAutoReefE);
  reefChooser.AddOption(kAutoReefF, kAutoReefF);
  reefChooser.AddOption(kOneCoral, kOneCoral);
  frc::SmartDashboard::PutData("Auto Reef Position", &reefChooser);

}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit()
{
  mDrive.state = DriveState::Auto;
  mDrive.enableModules();
  pigeon.pigeon.Reset();

  align.forwardPID.Reset();
  align.strafePID.Reset();
  mDrive.resetPoseEstimator(frc::Translation2d(0_m, 0_m), frc::Rotation2d(0_rad));

  std::string start_pos = positionChooser.GetSelected();
  std::string reef_pos = reefChooser.GetSelected();
  std::string allianceColor = allianceChooser.GetSelected();

  // Auto path choosing
  if(start_pos=="1" && reef_pos=="A") {
    mTrajectory.followPath(Trajectory::auto_1A, allianceIsRed);
  }
  else if(start_pos=="1" && reef_pos=="B") {
    mTrajectory.followPath(Trajectory::auto_1B, allianceIsRed);
  }
  else if(start_pos=="1" && reef_pos=="C") {
    mTrajectory.followPath(Trajectory::auto_1C, allianceIsRed);
  }
  else if(start_pos=="1" && reef_pos=="D") {
    mTrajectory.followPath(Trajectory::auto_1D, allianceIsRed);
  }
  else if(start_pos=="1" && reef_pos=="E") {
    mTrajectory.followPath(Trajectory::auto_1E, allianceIsRed);
  }
  else if(start_pos=="1" && reef_pos=="F") {
    mTrajectory.followPath(Trajectory::auto_1F, allianceIsRed);
  }
  else if(start_pos=="2" && reef_pos=="A") {
    mTrajectory.followPath(Trajectory::auto_2A, allianceIsRed);
  }
  else if(start_pos=="2" && reef_pos=="B") {
    mTrajectory.followPath(Trajectory::auto_2B, allianceIsRed);
  }
  else if(start_pos=="2" && reef_pos=="C") {
    mTrajectory.followPath(Trajectory::auto_2C, allianceIsRed);
  }
  else if(start_pos=="2" && reef_pos=="D") {
    mTrajectory.followPath(Trajectory::auto_2D, allianceIsRed);
  }
  else if(start_pos=="2" && reef_pos=="E") {
    mTrajectory.followPath(Trajectory::auto_2E, allianceIsRed);
  }
  else if(start_pos=="2" && reef_pos=="F") {
    mTrajectory.followPath(Trajectory::auto_2F, allianceIsRed);
  }
  else if(start_pos=="3" && reef_pos=="A") {
    mTrajectory.followPath(Trajectory::auto_3A, allianceIsRed);
  }
  else if(start_pos=="3" && reef_pos=="B") {
    mTrajectory.followPath(Trajectory::auto_3B, allianceIsRed);
  }
  else if(start_pos=="3" && reef_pos=="C") {
    mTrajectory.followPath(Trajectory::auto_3C, allianceIsRed);
  }
  else if(start_pos=="3" && reef_pos=="D") {
    mTrajectory.followPath(Trajectory::auto_3D, allianceIsRed);
  }
  else if(start_pos=="3" && reef_pos=="E") {
    mTrajectory.followPath(Trajectory::auto_3E, allianceIsRed);
  }
  else if(start_pos=="3" && reef_pos=="F") {
    mTrajectory.followPath(Trajectory::auto_3F, allianceIsRed);
  }
  else if (start_pos == "0" && reef_pos == "0") {
    mTrajectory.followPath(Trajectory::MOVE_STRAIGHT, allianceIsRed);
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit()
{
  mDrive.state = DriveState::Teleop;

  mDrive.enableModules();
  // pigeon.pigeon.SetYaw(units::degree_t(fmod(pigeon.getBoundedAngleCCW().getDegrees() + 180, 360)));
  mDrive.resetOdometry(frc::Translation2d(0_m, 0_m), frc::Rotation2d(0_rad));

  mHeadingController.setHeadingControllerState(SwerveHeadingController::OFF);
  xStickLimiter.reset(0.0);
  yStickLimiter.reset(0.0);
}
void Robot::TeleopPeriodic()
{
  if (coralLevel == 1) {
    speedLimiter = 0.8;
  }
  else if (coralLevel ==2 ) {
    speedLimiter = 0.7;
  }
  else if (coralLevel == 3) {
    speedLimiter = 0.5;
  }
  else if (coralLevel == 4) {
    speedLimiter = 0.3;
  }
  else if (coralLevel==5) {
    speedLimiter = 0.9;
  }

  bool fieldOriented = pigeon.pigeon.IsConnected();

  double vx = 0;
  double vy = 0;

  // Controller inputs
  double leftX = ControlUtil::deadZonePower(ctr.GetLeftX(), ctrDeadzone, 1);
  double leftY = ControlUtil::deadZonePower(-ctr.GetLeftY(), ctrDeadzone, 1);
  leftX = xStickLimiter.calculate(leftX) * speedLimiter;
  leftY = yStickLimiter.calculate(leftY) * speedLimiter;
  double rightX = ControlUtil::deadZoneQuadratic(ctr.GetRightX(), ctrDeadzone);
  double rot = 0;

  // Driver
  int dPad = ctr.GetPOV();
  bool alignPV = ctr.GetR2Button();
  bool resetGyro = (ctr.GetPOV() == 0);
  
  // Driving Modes
  double offSet = 0;
  double targetDistance = 0; // CHECK THIS
  double zeroSetpoint = 0;

  // frc::Color detectedColor = color.GetColor();
  // bool red = (detectedColor.red < 0.29) && (detectedColor.red > 0.255);
  // bool green = (detectedColor.green < 0.495) && (detectedColor.green > 0.47);
  // bool blue = (detectedColor.blue < 0.27) && (detectedColor.blue > 0.23);
  // bool coralIn = red && green && blue;

  // frc::SmartDashboard::PutBoolean("coralIN", coralIn);

  // if (coralIn) {
  //   mLED.Set_Color(frc::Color::kGreen);
  // }

  if(dPad==90) {
    coralSide = "right";
  }
  else if(dPad==270) {
    coralSide = "left";
  }
  
  if (ctr.GetR2ButtonPressed()) {
    align.forwardPID.Reset();
    align.strafePID.Reset();
  }
  else if (ctr.GetR1ButtonPressed() && cameraBack.camera.GetLatestResult().HasTargets()) {
    align.forwardPID.Reset();
    align.strafePID.Reset();
    float offset = cameraBack.getStrafeDistancetoTarget();
    float distanceToTag = cameraBack.getDistanceToTarget();
    float angle = cameraBack.getAngleSetpoint();
    setpointX = mDrive.getOdometryPose().X().value() + (distanceToTag * 39.37);
    setpointY = mDrive.getOdometryPose().Y().value() - (offset * 39.37);
  }
  else if (ctr.GetR1Button()) {
    ChassisSpeeds speed = align.driveToSetpointX(setpointX, mDrive, pigeon);
    ChassisSpeeds speeds = align.driveToSetpointY(setpointY, mDrive, pigeon);
    vy = speed.vyMetersPerSecond;
    vx = speeds.vyMetersPerSecond;
    rot = 0;
    // mHeadingController.setHeadingControllerState(SwerveHeadingController::ALIGN);
    // mHeadingController.setSetpoint(cameraBack.getAngleSetpoint());
    // rot = mHeadingController.calculate(pigeon.getBoundedAngleCW().getDegrees());
  }
  else if (alignPV) { // Alignment Mode
    if (cameraBack.isTargetDetected() && coralSide == "left") {
        targetDistance = 0.275;
        offSet = 0.078;

        ChassisSpeeds speeds = align.autoAlignPV(cameraBack, targetDistance, offSet);
        vx = speeds.vxMetersPerSecond;
        vy = speeds.vyMetersPerSecond;
        fieldOriented = false;

        mHeadingController.setHeadingControllerState(SwerveHeadingController::ALIGN);
        mHeadingController.setSetpoint(cameraBack.getAngleSetpoint());
        rot = mHeadingController.calculate(pigeon.getBoundedAngleCW().getDegrees());
      }
      else if (cameraFront.isTargetDetected() && coralSide == "right") {
        targetDistance = 0.275;
        offSet = -0.07;

        ChassisSpeeds speeds = align.autoAlignPV(cameraFront, targetDistance, offSet);
        vx = speeds.vxMetersPerSecond;
        vy = speeds.vyMetersPerSecond;
        fieldOriented = false;

        mHeadingController.setHeadingControllerState(SwerveHeadingController::ALIGN);
        mHeadingController.setSetpoint(cameraFront.getAngleSetpoint());
        rot = mHeadingController.calculate(pigeon.getBoundedAngleCW().getDegrees());
      }
  }
  else // Normal driving mode
  {
    mHeadingController.setHeadingControllerState(SwerveHeadingController::OFF);
    vx = leftX * moduleMaxFPS;
    vy = leftY * moduleMaxFPS;
    rot = rightX * moduleMaxRot * 2;
  }

  // Gyro Resets
  if (resetGyro) {
    pigeon.pigeon.Reset();
  }

  // Drive function
  mDrive.Drive(
      ChassisSpeeds(vx, vy, rot),
      pigeon.getBoundedAngleCCW(),
      fieldOriented,
      cleanDriveAccum);
  mDrive.updateOdometry();

  frc::SmartDashboard::PutNumber("Odometry X", mDrive.getOdometryPose().X().value());
  frc::SmartDashboard::PutNumber("Odometry Y", mDrive.getOdometryPose().Y().value());
  frc::SmartDashboard::PutNumber("vx", vx);
  frc::SmartDashboard::PutNumber("vy", vy);
  frc::SmartDashboard::PutNumber("setpoint X", setpointX);
  frc::SmartDashboard::PutNumber("setpoint y", setpointY);
  frc::SmartDashboard::PutNumber("distance to tag", cameraBack.getDistanceToTarget());
  frc::SmartDashboard::PutNumber("offset", cameraBack.getStrafeDistancetoTarget());
}

void Robot::DisabledInit()
{
  mDrive.state = DriveState::Disabled;
  mDrive.disableModules();
}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif