#include "SwerveDrive.h"
#include <cmath>
#include <string>
#include <chrono>
#include <thread>
#include <algorithm>

void SwerveDrive::Drive(ChassisSpeeds desiredSpeeds, Rotation2d fieldRelativeGyro, bool useFieldOriented, bool cleanAccum) {
    double desiredVx = desiredSpeeds.vxMetersPerSecond;
    double desiredVy = desiredSpeeds.vyMetersPerSecond;

    if (useFieldOriented) {
        desiredSpeeds = ChassisSpeeds::fromFieldRelativeSpeeds(desiredVx, desiredVy, desiredSpeeds.omegaRadiansPerSecond, fieldRelativeGyro);
        frc::SmartDashboard::PutBoolean("BOTCENTRIC!", false);
    } else {
        frc::SmartDashboard::PutBoolean("BOTCENTRIC!", true);
    }

    desiredVx = desiredSpeeds.vxMetersPerSecond;
    desiredVy = desiredSpeeds.vyMetersPerSecond;

    if (cleanAccum && fabs(desiredVx) < (moduleMaxFPS * 0.1) && fabs(desiredVy) < (moduleMaxFPS * 0.1)) {
        // zeroAccumulation();
        frc::SmartDashboard::PutNumber("CleanedAccum", true);
    } else {
        frc::SmartDashboard::PutNumber("CleanedAccum", false);
    }

    if (fabs(desiredVx) < kEpsilon && fabs(desiredVy) < kEpsilon && fabs(desiredSpeeds.omegaRadiansPerSecond) < kEpsilon) {
        // SwerveModuleState FLBRstop = SwerveModuleState(0.0, PI / 4);
        // SwerveModuleState FRBLstop = SwerveModuleState(0.0, 7 * PI / 4);

        mFrontLeft.setDriveVelocitySetpoint(0.0);
        mFrontRight.setDriveVelocitySetpoint(0.0);
        mBackLeft.setDriveVelocitySetpoint(0.0);
        mBackRight.setDriveVelocitySetpoint(0.0);
        return;
    }

    std::vector<SwerveModuleState> moduleStates = m_kinematics.toSwerveStates(desiredSpeeds);
    moduleStates = m_kinematics.desaturateWheelSpeeds(moduleStates, moduleMaxFPS);
    /**
     * Kinematics class returns module orientations in polar degrees
     * This means that 0 degrees is "to the right"
     * We want 0 degrees to be forward
     * Kinematics class also returns module speeds in ft/sec
     * We need to convert back FPS to RPM for the PIDs, so we use our conversion factors
     * FPS * 60 = FPM(feet per min)
     * FPM / (circum) = wheel rotations per minute(WPM)
     * WPM * (6.12 motor rot / 1 wheel rot) = RPM
     */

    for (int i = 0; i < 4; i++) {
        double speed = moduleStates[i].getSpeedFPS();
        speed = ((speed * 60) / wheelCircumFeet) * moduleDriveRatio;
        // SwerveModuleState temp = SwerveModuleState(speed, moduleStates[i].getRot2d().getRadians());
        moduleStates[i].setSpeedFPS(speed);
        // moduleStates[i] = temp;

        // frc::SmartDashboard::PutNumber(std::to_string(i) + "vel", speed);
        // frc::SmartDashboard::PutNumber(std::to_string(i) + "angle", moduleStates[i].getRot2d().getDegrees());
    }

    // Order of kinematics output is always BL, FL, BR, FR
    mFrontLeft.setModuleState(moduleStates[1], true);
    mFrontRight.setModuleState(moduleStates[3], true);
    mBackLeft.setModuleState(moduleStates[0], true);
    mBackRight.setModuleState(moduleStates[2], true);

    frc::SmartDashboard::PutNumber("desired speeds vx", desiredSpeeds.vxMetersPerSecond);
    frc::SmartDashboard::PutNumber("desired speeds vy", desiredSpeeds.vyMetersPerSecond);
}

/**
 * Initialize every motor(encoders, factory reset, current limits, etc)
 * Initialize each motor thread, which should start the threads
 */
void SwerveDrive::initModules() {
    mFrontLeft.initMotors();
    mFrontLeft.driveMotor.setInvert(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive);
    mFrontRight.initMotors();
    mFrontRight.driveMotor.setInvert(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive);
    mBackLeft.initMotors();
    //mBackLeft.driveMotor.setInvert(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive);
    mBackRight.initMotors();
    //mBackRight.driveMotor.setInvert(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive);

    modulePIDThread = std::thread(&SwerveDrive::runModules, this);
}

/**
 * Do not call this code outside of initModules's thread
 */
void SwerveDrive::runModules() {
    while (true) {
        mFrontLeft.run();
        mFrontRight.run();
        mBackLeft.run();
        mBackRight.run();
        std::this_thread::sleep_for(std::chrono::milliseconds(12));
    }
}

/**
 * Set every module's threads to active mode
 * So the PIDs start running
 */
void SwerveDrive::enableModules() {
    mFrontLeft.startModule();
    mBackLeft.startModule();
    mBackRight.startModule();
    mFrontRight.startModule();
}

/**
 * Disable every module's thread
 * Threads still exist, just on standby while loop
 */
bool SwerveDrive::disableModules() {
    mFrontLeft.stopModule();
    mBackLeft.stopModule();
    mBackRight.stopModule();
    mFrontRight.stopModule();
    return true;
}

/**
 * Resets odometry position
 * (used in auto config)
 */
void SwerveDrive::resetOdometry(frc::Translation2d trans, frc::Rotation2d angle) {
    m_odometry.ResetPosition(
        pigeon.getRotation2d(),
        {mBackLeft.getModulePosition(),
         mFrontLeft.getModulePosition(),
         mFrontRight.getModulePosition(),
         mBackRight.getModulePosition()},
        frc::Pose2d{trans, angle});
}

frc::Pose2d SwerveDrive::getOdometryPose() { // gets odometry pose in feet
    return m_odometry.GetPose();
}

/**
 * Updates odometry with current module positions
 */
void SwerveDrive::updateOdometry() {
    m_odometry.Update(
        -pigeon.getRotation2d(),
        {mBackLeft.getModulePosition(),
         mFrontLeft.getModulePosition(),
         mFrontRight.getModulePosition(),
         mBackRight.getModulePosition()});
}

/**
 * Swerve Drive Pose Esimator Resets
 * Better than odometry for vision, etc.
 */

void SwerveDrive::resetPoseEstimator(frc::Translation2d trans, frc::Rotation2d angle) {
    mSwervePose.ResetPosition(
        pigeon.getRotation2d(),
        {mBackLeft.getModulePosition(),
         mFrontLeft.getModulePosition(),
         mFrontRight.getModulePosition(),
         mBackRight.getModulePosition()},
        frc::Pose2d{trans, angle});
}

frc::Pose2d SwerveDrive::GetPoseEstimatorPose() { // In feet 
    frc::Pose2d pose = mSwervePose.GetEstimatedPosition(); // Get Estimated Position is in meters
    return frc::Pose2d{units::foot_t(pose.X().value() * 3.281), units::foot_t(pose.Y().value() * 3.281), pose.Rotation()};
}

void SwerveDrive::updatePoseEstimator(PhotonVision &camera, bool vision, units::second_t timestamp) {
    mSwervePose.UpdateWithTime(timestamp, 
        -pigeon.getRotation2d(),
        {mBackLeft.getModulePosition(),
         mFrontLeft.getModulePosition(),
         mFrontRight.getModulePosition(),
         mBackRight.getModulePosition()});

    if (camera.isTargetDetected() && vision) {
        frc::Pose2d robotPose = camera.returnPoseEstimate();
        mSwervePose.AddVisionMeasurement(robotPose, timestamp);
    }
}

// Auto Rotation Section
float SwerveDrive::roundToTwoDecimals(float num) {
    return std::round(num * 100.0) / 100.0;
}

void SwerveDrive::autoRot() {
    // Gets encoder positions of all the modules
    float backLeftpos = mBackLeft.getModulePosition().angle.Degrees().value();
    float backRightpos = mBackRight.getModulePosition().angle.Degrees().value();
    float frontLeftpos = mFrontLeft.getModulePosition().angle.Degrees().value();
    float frontRightpos = mFrontRight.getModulePosition().angle.Degrees().value();

    /*
    Transfers the encoder positions to a vector then rounds it to the nearest two decimal points
    */
    std::vector<float> modulePos {roundToTwoDecimals(backLeftpos), 
                            roundToTwoDecimals(backRightpos), 
                            roundToTwoDecimals(frontLeftpos), 
                            roundToTwoDecimals(frontRightpos)};

    // Using a set in order to take out duplicates
    std::set<float> moduleSet(modulePos.begin(), modulePos.end());

    // Since sets sort from least to greatest, lowerDrivePos gets the lowest val and higherDrivePos gets the highest val
    float lowerDrivePos = *moduleSet.begin();
    float higherDrivePos = *std::prev(moduleSet.end());

    // See how many times lowerDrivePos and higherDrivePos occur in the vector
    int lowerDrivePosOccur = std::count(modulePos.begin(), modulePos.end(), lowerDrivePos);
    int higherDrivePosOccur = std::count(modulePos.begin(), modulePos.end(), higherDrivePos);

    frc::SmartDashboard::PutNumber("lowerDrivePos", lowerDrivePos);
    frc::SmartDashboard::PutNumber("higherDrivePos", higherDrivePos);
    frc::SmartDashboard::PutNumber("lowerDrivePosOccur", lowerDrivePosOccur);
    frc::SmartDashboard::PutNumber("higherDrivePosOccur", higherDrivePosOccur);

    // See if there's an odd one out, if not, the wheels are in the right pos
    if (lowerDrivePosOccur == 1) {
        index = std::distance(modulePos.begin(), std::find(modulePos.begin(), modulePos.end(), lowerDrivePos));
    }
    else if (higherDrivePosOccur == 1) {
        index = std::distance(modulePos.begin(), std::find(modulePos.begin(), modulePos.end(), higherDrivePos));
    }
    else if (lowerDrivePosOccur == 4 || higherDrivePosOccur == 4) {
        goodWheelPos = true;
    }

    frc::SmartDashboard::PutNumber("swerve index rot", index);

    // Change angle setpoint is the wheels aren't in the right position
    if (goodWheelPos == false) {
        if (index == 0) {
            mBackLeft.setSteerAngleSetpoint(backRightpos);
        }
        else if (index == 1) {
            mBackRight.setSteerAngleSetpoint(frontRightpos);
        }
        else if (index == 2) {
            mFrontLeft.setSteerAngleSetpoint(backLeftpos);
        }
        else if (index == 3) {
            mFrontRight.setSteerAngleSetpoint(frontLeftpos);
        }
        goodWheelPos = true;
    }
}

/**
 * Uses gyro widget
 * Flips angle gyro if module has negative velocity
 */
// void SwerveDrive::displayDriveTelemetry() {
// }

// void SwerveDrive::zeroAccumulation() {
//     mFrontLeft.m_pidController.SetIAccum(0.0);
//     mFrontRight.m_pidController.SetIAccum(0.0);
//     mBackRight.m_pidController.SetIAccum(0.0);
//     mBackLeft.m_pidController.SetIAccum(0.0);
// }