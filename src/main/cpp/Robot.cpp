// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "RobotContainer.h"

#include "cameraserver/CameraServer.h"
#include <frc2/command/CommandScheduler.h>
#include <limits>


Robot::Robot() {}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  // frc::CameraServer::StartAutomaticCapture();
  frc2::CommandScheduler::GetInstance().Run(); //runs command-based queue
  QuestNav::getInstance().periodic();
  // Advance odometry before vision so SwerveDrivePoseEstimator state matches wheel/gyro, then fuse.
  m_container.drivetrain.updateOdometry();
  m_container.turretCam.periodic();
  frc::SmartDashboard::PutNumber("Robot Pose X", m_container.drivetrain.getPose().X().value());
  frc::SmartDashboard::PutNumber("Robot Pose Y", m_container.drivetrain.getPose().Y().value());
  frc::SmartDashboard::PutNumber("Quest Heading", QuestNav::getInstance().getPose2d().Rotation().Degrees().value());
  frc::SmartDashboard::PutBoolean("Auto Target", m_container.autoTargeting);
  frc::SmartDashboard::PutNumber("Distance to Tag", m_container.turretCam.distanceToTag);
  frc::SmartDashboard::PutNumber("Strafe Distance to Tag", m_container.turretCam.strafeDistanceToTag);
  frc::SmartDashboard::PutNumber("tx", m_container.turretCam.tx);
  frc::SmartDashboard::PutNumber("ty", m_container.turretCam.ty);
  frc::SmartDashboard::PutNumber("Turret Angle", m_container.m_turret.getCurrentMotorAngle());
  frc::SmartDashboard::PutBoolean("Has Target", m_container.turretCam.hasTarget);
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  // m_autonomousCommand = m_container.GetAutonomousCommand();

  // if (m_autonomousCommand) {
  //   frc2::CommandScheduler::GetInstance().Schedule(m_autonomousCommand.value());
  // }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  m_container.InitializeStartPose();       // sets pose from dashboard selector (fallback)
  // m_container.autoTargeting = true;

  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  // if (m_autonomousCommand) {
  //   m_autonomousCommand->Cancel();
  // }
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
  //frc::SmartDashboard::PutNumber("encoder angle", m_turret.getCurrentAngle());
}
/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

/**
 * This function is called once when the robot is first started up.
 */
void Robot::SimulationInit() {}

/**
 * This function is called periodically whilst in simulation.
 */
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
