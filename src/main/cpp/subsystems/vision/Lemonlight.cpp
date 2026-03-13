#include "subsystems/vision/Lemonlight.h"
#include "LimelightHelpers.h"
#include "visionKonstants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>

Lemonlight::Lemonlight() {
    // Initialization code
    // Assuming pipeline 0 is configured for AprilTags
    LimelightHelpers::setPipelineIndex(m_limelightName, 0); 
}

void Lemonlight::Periodic() {
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
    return LimelightHelpers::getTV(m_limelightName);
}

int Lemonlight::GetPrimaryTagID() {
    return static_cast<int>(LimelightHelpers::getFiducialID(m_limelightName));
}

std::optional<units::meter_t> Lemonlight::GetDistanceToOffset(int tagID, const std::string& offsetName) {
    if (!HasTarget() || GetPrimaryTagID() != tagID) return std::nullopt;

    auto it = VisionConstants::tagOffsets.find(tagID);
    if (it == VisionConstants::tagOffsets.end()) return std::nullopt;

    auto offsetIt = it->second.find(offsetName);
    if (offsetIt == it->second.end()) return std::nullopt;

    // Get the transform for this offset relative to the tag
    frc::Transform3d offsetTransform = offsetIt->second;

    // Get tag pose in camera space
    frc::Pose3d tagPose_CameraSpace = LimelightHelpers::getTargetPose3d_CameraSpace(m_limelightName);

    // Apply the offset transform to the tag pose to get the offset pose in camera space
    frc::Pose3d offsetPose_CameraSpace = tagPose_CameraSpace + offsetTransform;

    // Distance is the 3D norm of the translation
    return offsetPose_CameraSpace.Translation().Norm();
}

std::optional<units::degree_t> Lemonlight::GetHeadingErrorToOffset(int tagID, const std::string& offsetName) {
    if (!HasTarget() || GetPrimaryTagID() != tagID) return std::nullopt;

    auto it = VisionConstants::tagOffsets.find(tagID);
    if (it == VisionConstants::tagOffsets.end()) return std::nullopt;

    auto offsetIt = it->second.find(offsetName);
    if (offsetIt == it->second.end()) return std::nullopt;

    frc::Transform3d offsetTransform = offsetIt->second;
    frc::Pose3d tagPose_CameraSpace = LimelightHelpers::getTargetPose3d_CameraSpace(m_limelightName);
    frc::Pose3d offsetPose_CameraSpace = tagPose_CameraSpace + offsetTransform;

    // X is forward out of the camera, Y is left. Angle to face the point is atan2(Y, X)
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
    if (!HasTarget()) return std::nullopt;
    frc::Pose3d tagPose_CameraSpace = LimelightHelpers::getTargetPose3d_CameraSpace(m_limelightName);
    return tagPose_CameraSpace.Translation().Norm();
}

std::optional<units::degree_t> Lemonlight::GetHeadingErrorToTag() {
    if (!HasTarget()) return std::nullopt;
    frc::Pose3d tagPose_CameraSpace = LimelightHelpers::getTargetPose3d_CameraSpace(m_limelightName);
    
    // X is forward, Y is left.
    double x = tagPose_CameraSpace.X().value();
    double y = tagPose_CameraSpace.Y().value();
    
    return units::radian_t{std::atan2(y, x)};
}
