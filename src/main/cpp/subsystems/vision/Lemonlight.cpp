#include "subsystems/vision/Lemonlight.h"
#include "LimelightHelpers.h"
#include "visionKonstants.h"
#include "subsystems/DriveSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>

Lemonlight::Lemonlight(DriveSubsystem& drive)
    : m_drive(drive),
      m_fieldLayout(frc::LoadAprilTagLayoutField(frc::AprilTagField::k2025Reefscape))
{
    if (VisionConstants::usePhotonVision) {
        m_photonCamera = std::make_unique<photon::PhotonCamera>(
            std::string(VisionConstants::photonCameraName));
        m_poseEstimator.emplace(
            m_fieldLayout,
            photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
            VisionConstants::robotToCamera);
    } else {
        LimelightHelpers::setPipelineIndex(m_limelightName, 0);
    }
}

void Lemonlight::Periodic() {
    if (VisionConstants::usePhotonVision) {
        UpdateDrivePose();
    }

    // Update SmartDashboard with useful information
    bool hasTarget = HasTarget();
    frc::SmartDashboard::PutBoolean("Lemonlight/HasTarget", hasTarget);
    
    if (hasTarget) {
        int targetId = GetPrimaryTagID();
        frc::SmartDashboard::PutNumber("Lemonlight/TargetID", targetId);
        
        auto dist = GetDistanceToTag();
        if (dist) {
            frc::SmartDashboard::PutNumber("Lemonlight/DistToTag", dist.value().value());
        }
        
        auto error = GetHeadingErrorToTag();
        if (error) {
            frc::SmartDashboard::PutNumber("Lemonlight/ErrorToTag", error.value().value());
        }
        
        // Example: If seeing tag 7, also log distance to SpeakerCenter offset
        if (targetId == 7) {
            auto distOffset = GetDistanceToOffset(7, "SpeakerCenter");
            if (distOffset) {
                frc::SmartDashboard::PutNumber("Lemonlight/DistToSpeakerCenter", distOffset.value().value());
            }
            
            auto errorOffset = GetHeadingErrorToOffset(7, "SpeakerCenter");
            if (errorOffset) {
                frc::SmartDashboard::PutNumber("Lemonlight/ErrorToSpeakerCenter", errorOffset.value().value());
            }
        }
    }
}

bool Lemonlight::HasTarget() {
    if (VisionConstants::usePhotonVision) {
        return m_photonCamera->GetLatestResult().HasTargets();
    } else {
        return LimelightHelpers::getTV(m_limelightName);
    }
}

int Lemonlight::GetPrimaryTagID() {
    if (VisionConstants::usePhotonVision) {
        auto result = m_photonCamera->GetLatestResult();
        if (result.HasTargets()) {
            return result.GetBestTarget().GetFiducialId();
        }
        return -1;
    } else {
        return static_cast<int>(LimelightHelpers::getFiducialID(m_limelightName));
    }
}

std::optional<frc::Pose3d> Lemonlight::GetTargetPoseCameraSpace() {
    if (VisionConstants::usePhotonVision) {
        auto result = m_photonCamera->GetLatestResult();
        if (!result.HasTargets()) return std::nullopt;
        // Transform3d from camera to target is exactly the pose of the target in camera space
        return frc::Pose3d() + result.GetBestTarget().GetBestCameraToTarget();
    } else {
        if (!LimelightHelpers::getTV(m_limelightName)) return std::nullopt;
        return LimelightHelpers::getTargetPose3d_CameraSpace(m_limelightName);
    }
}

std::optional<units::meter_t> Lemonlight::GetDistanceToOffset(int tagID, const std::string& offsetName) {
    if (GetPrimaryTagID() != tagID) return std::nullopt;

    auto it = VisionConstants::tagOffsets.find(tagID);
    if (it == VisionConstants::tagOffsets.end()) return std::nullopt;

    auto offsetIt = it->second.find(offsetName);
    if (offsetIt == it->second.end()) return std::nullopt;

    frc::Transform3d offsetTransform = offsetIt->second;

    auto tagPoseOpt = GetTargetPoseCameraSpace();
    if (!tagPoseOpt) return std::nullopt;
    frc::Pose3d tagPose_CameraSpace = tagPoseOpt.value();

    frc::Pose3d offsetPose_CameraSpace = tagPose_CameraSpace + offsetTransform;

    return offsetPose_CameraSpace.Translation().Norm();
}

std::optional<units::degree_t> Lemonlight::GetHeadingErrorToOffset(int tagID, const std::string& offsetName) {
    if (GetPrimaryTagID() != tagID) return std::nullopt;

    auto it = VisionConstants::tagOffsets.find(tagID);
    if (it == VisionConstants::tagOffsets.end()) return std::nullopt;

    auto offsetIt = it->second.find(offsetName);
    if (offsetIt == it->second.end()) return std::nullopt;

    frc::Transform3d offsetTransform = offsetIt->second;

    auto tagPoseOpt = GetTargetPoseCameraSpace();
    if (!tagPoseOpt) return std::nullopt;
    frc::Pose3d tagPose_CameraSpace = tagPoseOpt.value();

    frc::Pose3d offsetPose_CameraSpace = tagPose_CameraSpace + offsetTransform;

    double x = offsetPose_CameraSpace.X().value();
    double y = offsetPose_CameraSpace.Y().value();
    
    return units::radian_t{std::atan2(y, x)};
}

std::optional<units::degree_t> Lemonlight::GetTargetTurretHeading(units::degree_t currentTurretHeading, int tagID, const std::string& offsetName) {
    auto error = GetHeadingErrorToOffset(tagID, offsetName);
    if (!error) return std::nullopt;

    // Turret heading + angle error in camera space = absolute heading to aim at
    return currentTurretHeading + error.value();
}

std::optional<units::meter_t> Lemonlight::GetDistanceToTag() {
    auto tagPoseOpt = GetTargetPoseCameraSpace();
    if (!tagPoseOpt) return std::nullopt;
    return tagPoseOpt.value().Translation().Norm();
}

std::optional<units::degree_t> Lemonlight::GetHeadingErrorToTag() {
    auto tagPoseOpt = GetTargetPoseCameraSpace();
    if (!tagPoseOpt) return std::nullopt;

    double x = tagPoseOpt.value().X().value();
    double y = tagPoseOpt.value().Y().value();

    return units::radian_t{std::atan2(y, x)};
}

std::optional<frc::Pose2d> Lemonlight::GetEstimatedFieldPose() {
    if (!VisionConstants::usePhotonVision || !m_photonCamera || !m_poseEstimator)
        return std::nullopt;

    auto result = m_photonCamera->GetLatestResult();
    if (!result.HasTargets()) return std::nullopt;

    auto estimatedPose = m_poseEstimator->EstimateCoprocMultiTagPose(result);
    if (!estimatedPose.has_value()) {
        estimatedPose = m_poseEstimator->EstimateLowestAmbiguityPose(result);
    }

    if (estimatedPose.has_value()) {
        return estimatedPose->estimatedPose.ToPose2d();
    }
    return std::nullopt;
}

void Lemonlight::UpdateDrivePose() {
    if (!m_photonCamera || !m_poseEstimator) return;

    for (const auto& result : m_photonCamera->GetAllUnreadResults()) {
        // Try coprocessor multi-tag first (most accurate), fall back to lowest ambiguity
        auto estimatedPose = m_poseEstimator->EstimateCoprocMultiTagPose(result);
        if (!estimatedPose.has_value()) {
            estimatedPose = m_poseEstimator->EstimateLowestAmbiguityPose(result);
        }

        if (estimatedPose.has_value()) {
            m_drive.AddVisionMeasurement(
                estimatedPose->estimatedPose.ToPose2d(),
                estimatedPose->timestamp);

            frc::SmartDashboard::PutNumber("Lemonlight/VisionPoseX",
                estimatedPose->estimatedPose.X().value());
            frc::SmartDashboard::PutNumber("Lemonlight/VisionPoseY",
                estimatedPose->estimatedPose.Y().value());
        }
    }
}
