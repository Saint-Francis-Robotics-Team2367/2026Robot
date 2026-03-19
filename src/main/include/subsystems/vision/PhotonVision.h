#pragma once

#include <frc/controller/PIDController.h>
#include "Constants.h"
#include <vector>

#include "photon/PhotonCamera.h"
#include "photon/targeting/PhotonTrackedTarget.h"
#include "photon/estimation/VisionEstimation.h"
#include "photon/PhotonPoseEstimator.h"
#include "photon/targeting/PhotonPipelineResult.h"

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>

class PhotonVision {
private:
    double targetDistanceMeters;
    double targetYaw;
    double targetFiducial;
    double targetxMeters;
    
    double cameraPitchRadians;
    double cameraHeightMeters;
    double cameraPositionOffsetxMeters;
    double cameraPositionOffsetyMeters;

    std::vector<std::vector<int>> tagInfo {{1, 234}, {2, 126}, {3, 90}, {4, 0}, {5, 0},
        {10, 180}, {11, 120}, {6, 60}, {7, 0}, {8, 300}, {9, 240},
        {12, 126}, {13, 234}, {14, 0}, {15, 0}, {16, 90},
        {17, 300}, {18, 0}, {19, 60}, {20, 120}, {21, 180}, {22, 240}};

public:
    frc::AprilTagFieldLayout apriltagField = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::kDefaultField);
    photon::PhotonPoseEstimator poseEstimator{
      apriltagField, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
      frc::Transform3d(0.0_m, 0.0_m, 0.0_m, frc::Rotation3d(0.0_deg, 0.0_deg, 0.0_deg))};

    enum TagType {HUB, OUTPOST, DEPOT, TRENCH};

    photon::PhotonCamera camera;
    
    PhotonVision(std::string name)
        : camera(name),
          cameraPitchRadians(0.0),
          cameraHeightMeters(1.0), // FIX THIS
          cameraPositionOffsetxMeters(0.0),
          cameraPositionOffsetyMeters(0.0) {}

    void init() {
        camera.SetPipelineIndex(0);
    }

    bool isTargetDetected() {
        return camera.GetLatestResult().HasTargets();
    }

    int getTagID() {
        return camera.GetLatestResult().GetBestTarget().GetFiducialId();
    }

    TagType getTagType() {
        if(camera.GetLatestResult().HasTargets()) {
            int tagID = getTagID();
            if ((tagID >= 2 && tagID <= 11) || (tagID >= 18 && tagID <= 27)) {
                return HUB;
            } else if (tagID == 15 || tagID == 16 || tagID == 31 || tagID == 32) {
                return DEPOT;
            } else if (tagID == 13 || tagID == 14 || tagID == 29 || tagID == 30) {
                return OUTPOST;
            } else if (tagID == 6 || tagID == 7 || tagID == 1 || tagID == 12 || tagID == 17 || tagID == 28 || tagID == 22 || tagID == 23) {
                return TRENCH;
            }
        };
    }

    double getAngleSetpoint() {
        if (isTargetDetected()) {
            int tagID = getTagID();
            
            for (int i = 0; i < tagInfo.size(); i++) {
                if (tagInfo[i][0] == tagID) {
                    return tagInfo[i][1];
                }
            }
            return 0;
        }   
    }

    bool isHub() {
        return (getTagType() == TagType::HUB);
    }

    bool isDepot() {
        return (getTagType() == TagType::DEPOT);
    }

    double getDistanceToTarget() {
        return camera.GetLatestResult().GetBestTarget().GetBestCameraToTarget().X().value();
    }

    double getStrafeDistancetoTarget() {
        return camera.GetLatestResult().GetBestTarget().GetBestCameraToTarget().Y().value();
    }

    double getYaw() {
        return camera.GetLatestResult().GetBestTarget().GetYaw();
    }

    frc::Pose2d returnPoseEstimate() {
        std::optional<photon::EstimatedRobotPose> visionCache;

        for (const auto& result : camera.GetAllUnreadResults()) {
            if (camera.GetLatestResult().HasTargets()) {
                visionCache = poseEstimator.Update(result);
                photon::PhotonPipelineResult latestResult = result;
            }
        }
        return visionCache->estimatedPose.ToPose2d();
    }
};