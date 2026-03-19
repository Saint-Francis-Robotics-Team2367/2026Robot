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

    Limelight::Limelight(DriveSubsystem &mDriveInput, std::string name = "") : mDrive(mDriveInput) {
        LimelightName = name;
    }

    void init() {
        LimelightHelpers::setPipelineIndex(LimelightName, 0);
        LimelightHelpers::setCameraPose_RobotSpace(
            LimelightName,
            0.5, // tune these numbers
            0.5,
            0.0,
            0.0,
            0.0,
            0.0
        );

        LimelightHelpers::SetFiducialIDFiltersOverride(LimelightName, std::vector<int>{2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27});
    }

    void periodic() {
        LimelightHelpers::PoseEstimate limelightMeasurement = LimelightHelpers::getBotPoseEstimate_wpiBlue(LimelightName);
        if (limelightMeasurement.tagCount >= 2) {
            mDrive.getPoseEstimator().SetVisionMeasurementStdDevs({0.7, 0.7, 9999999}); // Ignore Megatag Gyro Input
            mDrive.getPoseEstimator().AddVisionMeasurement(
                limelightMeasurement.pose,
                limelightMeasurement.timestampSeconds
            );
        }

        tx = LimelightHelpers::getTX(LimelightName);
        ty = LimelightHelpers::getTY(LimelightName);
        ta = LimelightHelpers::getTA(LimelightName);
        hasTarget = LimelightHelpers::getTV(LimelightName);
        heartbeat = LimelightHelpers::getHeartbeat();

        std::vector<double> pose = LimelightHelpers::getTargetPose_RobotSpace();
        distanceToTag = pose[2];
        strafeDistanceToTag = pose[0];
    }
};