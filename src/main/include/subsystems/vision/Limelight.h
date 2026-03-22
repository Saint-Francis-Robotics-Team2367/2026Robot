#include "LimelightHelpers.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/vision/QuestNav.h"

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
        // MegaTag2 fuses tag geometry with robot heading; Limelight docs require this every frame
        // before reading botpose_orb_wpiblue.
        LimelightHelpers::SetRobotOrientation(
            LimelightName,
            QuestNav::getInstance().getRotation2d().Degrees().value(),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0);

        tx = LimelightHelpers::getTX(LimelightName);
        ty = LimelightHelpers::getTY(LimelightName);
        ta = LimelightHelpers::getTA(LimelightName);
        hasTarget = LimelightHelpers::getTV(LimelightName);
        
        LimelightHelpers::PoseEstimate limelightMeasurement =
            LimelightHelpers::getBotPoseEstimate_wpiRed_MegaTag2(LimelightName);

        if (hasTarget && limelightMeasurement.tagCount >= 2) {
            limelightMeasurement =
                LimelightHelpers::getBotPoseEstimate_wpiRed_MegaTag2(LimelightName);
            mDrive.odometry.SetVisionMeasurementStdDevs({0.5, 0.5, 9999999}); // Ignore Megatag Gyro Input
            mDrive.odometry.AddVisionMeasurement(
                limelightMeasurement.pose,
                limelightMeasurement.timestampSeconds);
        }
        heartbeat = LimelightHelpers::getHeartbeat();

        std::vector<double> pose =
            LimelightHelpers::getTargetPose_RobotSpace(LimelightName);
        if (pose.size() >= 3) {
            distanceToTag = pose[2];
            strafeDistanceToTag = pose[0];
        } else {
            distanceToTag = 0.0;
            strafeDistanceToTag = 0.0;
        }
    }

    bool isHub() {
        if (!hasTarget) {
            return false;
        }
        int tagID = LimelightHelpers::getFiducialID();
        return (tagID >= 2 && tagID <= 11) || (tagID >= 18 && tagID <= 27);
    }
};