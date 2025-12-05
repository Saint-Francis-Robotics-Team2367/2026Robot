#pragma once

#include "sensors/Limelight.h"
#include "ChassisSpeeds.h"
#include "SwerveHeadingController.h"
#include "SwerveDrive.h"
#include "sensors/PhotonVision.h"

class SwerveAlign {
public: 
    frc::PIDController forwardPID{9.1, 0, 0.2}; // 8.55
    frc::PIDController strafePID{9.1, 0, 0.2};
    
    double forwardSpeed = 0;
    double strafeSpeed = 0;
    double targetDistance = 0;
    double targetOffset = 0;

public:

    double currentX = 0;
    double currentY = 0;
    double prevErrorX = 0;
    double prevErrorY = 0;

    bool isAligned(PhotonVision& cameraFront) {

        if (cameraFront.isTargetDetected()) {
            if (fabs(cameraFront.getStrafeDistancetoTarget()) < 0.05 && fabs(cameraFront.getDistanceToTarget()) < 0.6) {
                return true;
            }
        }
        return false;
    }

    ChassisSpeeds autoAlign(Limelight& limelight, double setpointDistance, double offsetSetpoint) { // distance in meters
        ChassisSpeeds speeds;
        double offset = limelight.getTargetPoseRobotSpace().x;
        double distanceToTag = limelight.getDistanceToWall();
        targetDistance = setpointDistance;
        targetOffset = offsetSetpoint;
        forwardPID.SetTolerance(0.05, 0.01);
        strafePID.SetTolerance(0.05, 0.01);
        if (!limelight.isTargetDetected()) {
            speeds = ChassisSpeeds::fromRobotRelativeSpeeds(0, 0, 0);
        }
        else if (!forwardPID.AtSetpoint() || !strafePID.AtSetpoint()) {
            double forwardSpeed = forwardPID.Calculate(distanceToTag, setpointDistance);
            double strafeSpeed = strafePID.Calculate(offset, offsetSetpoint);
            strafeSpeed = std::clamp(strafeSpeed, -4.0, 4.0);
            forwardSpeed = std::clamp(forwardSpeed, -4.0, 4.0);
            speeds = ChassisSpeeds::fromRobotRelativeSpeeds(strafeSpeed, -forwardSpeed, 0);
        }
        else {
            speeds = ChassisSpeeds::fromRobotRelativeSpeeds(0, 0, 0);
        }
        return speeds;
    }

    ChassisSpeeds autoAlignPV(PhotonVision& photon, double setpointDistance, double offsetSetpoint) {
        ChassisSpeeds speeds;
        auto result = photon.camera.GetLatestResult();
        if (!result.HasTargets()) {
            return ChassisSpeeds::fromRobotRelativeSpeeds(0, 0, 0);
        }

        frc::SmartDashboard::PutBoolean("result target", result.HasTargets());
        
        photon::PhotonTrackedTarget target = result.GetBestTarget();

        double offset = photon.getStrafeDistancetoTarget();
        double distanceToTag = photon.getDistanceToTarget(); // FIX THIS: WHAT UNIT?
        targetDistance = setpointDistance;
        targetOffset = offsetSetpoint;
        forwardPID.SetTolerance(0.05, 0.01);
        strafePID.SetTolerance(0.025, 0.01);
        
        if (!forwardPID.AtSetpoint() || !strafePID.AtSetpoint()) {
            double forwardSpeed = forwardPID.Calculate(distanceToTag, setpointDistance);
            double strafeSpeed = strafePID.Calculate(offset, offsetSetpoint);
            speeds = ChassisSpeeds::fromRobotRelativeSpeeds(strafeSpeed, -forwardSpeed, 0);
        } else {
            speeds = ChassisSpeeds::fromRobotRelativeSpeeds(0, 0, 0);
        }
        return speeds;
    }

    ChassisSpeeds driveToSetpointX(double setpointX, SwerveDrive& drive, Pigeon &pigeon) { 
        // Variables
        ChassisSpeeds speeds;
        double currentX = drive.getOdometryPose().X().value();

        //Tolerance
        strafePID.SetTolerance(1.5, 0.01);

        if (!strafePID.AtSetpoint()) {
            double strafeSpeed = strafePID.Calculate(currentX, setpointX);
            // if (prevErrorX < (setpointX-currentX)) {
            //     strafeSpeed = -fabs(strafeSpeed);
            // }
            // else {
            //     strafeSpeed = fabs(strafeSpeed);
            // }
            strafeSpeed = std::clamp(strafeSpeed, -10.0, 10.0);
            speeds = ChassisSpeeds::fromFieldRelativeSpeeds(0, strafeSpeed, 0, pigeon.getBoundedAngleCW());
        }
        else {
            // strafeSpeed = 0;
            speeds = ChassisSpeeds::fromFieldRelativeSpeeds(0, 0, 0, 0);
        }
        // prevErrorX = setpointX-currentX;
        return speeds;
    }

    ChassisSpeeds driveToSetpointY(double setpointY, SwerveDrive& drive, Pigeon &pigeon) { 
        // Variables
        ChassisSpeeds speeds;
        double currentY = drive.getOdometryPose().Y().value();

        // Tolerance
        forwardPID.SetTolerance(14.5, 0.01);

        if (!forwardPID.AtSetpoint()) {
            double forwardSpeed = forwardPID.Calculate(currentY, setpointY);
            // if (prevErrorY < (setpointY - currentY)) {
            //     forwardSpeed = -fabs(forwardSpeed);
            // }
            // else {
            //     forwardSpeed = fabs(forwardSpeed);
            // }
            forwardSpeed = std::clamp(forwardSpeed, -10.0, 10.0);
            speeds = ChassisSpeeds::fromFieldRelativeSpeeds(0, forwardSpeed, 0, pigeon.getBoundedAngleCW());
        }
        else {
            // forwardSpeed = 0;
            speeds = ChassisSpeeds(0, 0, 0);
        }
        // prevErrorY = setpointY - currentY;
        return speeds;
    }

    ChassisSpeeds driveToSetpoint(float setpointX, float setpointY, SwerveDrive &drive, Pigeon &pigeon) {
        ChassisSpeeds speeds;
        forwardPID.SetTolerance(0.1, 0.01);
        strafePID.SetTolerance(0.1, 0.01);
        float currentX = drive.getOdometryPose().X().value();
        float currentY = drive.getOdometryPose().Y().value();

        if (!forwardPID.AtSetpoint() || !strafePID.AtSetpoint()) {
            double forwardSpeed = forwardPID.Calculate(currentY, setpointY);
            double strafeSpeed = strafePID.Calculate(currentX, setpointX);
            speeds = ChassisSpeeds::fromRobotRelativeSpeeds(strafeSpeed, -forwardSpeed, 0); //CHECK THIS
        }
        else {
            speeds = ChassisSpeeds(0, 0, 0);
        }
        return speeds;
    }
};