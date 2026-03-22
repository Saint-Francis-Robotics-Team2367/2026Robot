#include "LimelightHelpers.h"
#include "subsystems/DriveSubsystem.h"

class Limelight {
private:
    DriveSubsystem &mDrive;
    std::string LimelightName = "";

public:
    double tx = 0;
    double ty = 0;
    double ta = 0;
    bool hasTarget = false;
    double heartbeat = 0.0;
    double distanceToTag = 0.0;
    double strafeDistanceToTag = 0.0;

    Limelight(DriveSubsystem &mDriveInput, std::string name = "") : mDrive(mDriveInput) {
        LimelightName = name;
        LimelightHelpers::setPipelineIndex(LimelightName, 0);
        LimelightHelpers::setCameraPose_RobotSpace(
            LimelightName,
            0.305, 
            0.0,
            0.36,
            0.0,
            15,
            0.0
        );
        LimelightHelpers::SetRobotOrientation(LimelightName, QuestNav::getInstance().getRotation2d().Degrees().value(), 0.0, 0.0, 0.0, 0.0, 0.0);
        LimelightHelpers::SetFiducialIDFiltersOverride(LimelightName, std::vector<int>{2, 4, 5, 10, 24, 26, 27, 20, 8}); // Remove ID 8 to blacklist 
        LimelightHelpers::SetFiducialDownscalingOverride(LimelightName, 2.0);
    }

    void periodic() {
        LimelightHelpers::PoseEstimate limelightMeasurement = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(LimelightName);

        mDrive.getPoseEstimator().SetVisionMeasurementStdDevs({0.5, 0.5, 9999999}); // Ignore Megatag Gyro Input
        mDrive.getPoseEstimator().AddVisionMeasurement(
            limelightMeasurement.pose,
            limelightMeasurement.timestampSeconds
        );

        tx = LimelightHelpers::getTX(LimelightName);
        ty = LimelightHelpers::getTY(LimelightName);
        ta = LimelightHelpers::getTA(LimelightName);
        hasTarget = LimelightHelpers::getTV(LimelightName);
        heartbeat = LimelightHelpers::getHeartbeat();

        std::vector<double> pose = LimelightHelpers::getTargetPose_RobotSpace();
        distanceToTag = pose[2];
        strafeDistanceToTag = pose[0];
    }

    bool isHub() {
        if (!hasTarget) {
            return false;
        }
        int tagID = LimelightHelpers::getFiducialID();
        return (tagID >= 2 && tagID <= 11) || (tagID >= 18 && tagID <= 27);
    }
};