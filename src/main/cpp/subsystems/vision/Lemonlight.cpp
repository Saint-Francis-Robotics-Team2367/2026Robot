// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/vision/Lemonlight.h"
#include "LimelightHelpers.h"
#include "visionKonstants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>

Lemonlight::Lemonlight() {
  if (VisionConstants::usePhotonVision) {
    m_photonCamera = std::make_unique<photon::PhotonCamera>(m_limelightName);
  } else {
    LimelightHelpers::setPipelineIndex(m_limelightName, 0);
  }
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
      frc::SmartDashboard::PutNumber("Lemonlight/DistToTag",
                                     dist.value().value());
    }

    auto error = GetHeadingErrorToTag();
    if (error) {
      frc::SmartDashboard::PutNumber("Lemonlight/ErrorToTag",
                                     error.value().value());
    }

    // Example: If seeing tag 7, also log distance to SpeakerCenter offset
    if (targetId == 7) {
      auto distOffset = GetDistanceToOffset(7, "SpeakerCenter");
      if (distOffset) {
        frc::SmartDashboard::PutNumber("Lemonlight/DistToSpeakerCenter",
                                       distOffset.value().value());
      }

      auto errorOffset = GetHeadingErrorToOffset(7, "SpeakerCenter");
      if (errorOffset) {
        frc::SmartDashboard::PutNumber("Lemonlight/ErrorToSpeakerCenter",
                                       errorOffset.value().value());
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
    if (!result.HasTargets())
      return std::nullopt;
    // Transform3d from camera to target is exactly the pose of the target in
    // camera space
    return frc::Pose3d() + result.GetBestTarget().GetBestCameraToTarget();
  } else {
    if (!LimelightHelpers::getTV(m_limelightName))
      return std::nullopt;
    return LimelightHelpers::getTargetPose3d_CameraSpace(m_limelightName);
  }
}

std::optional<units::meter_t> Lemonlight::GetDistanceToOffset(
    int tagID, const std::string& offsetName) {
  if (GetPrimaryTagID() != tagID)
    return std::nullopt;

  auto it = VisionConstants::tagOffsets.find(tagID);
  if (it == VisionConstants::tagOffsets.end())
    return std::nullopt;

  auto offsetIt = it->second.find(offsetName);
  if (offsetIt == it->second.end())
    return std::nullopt;

  frc::Transform3d offsetTransform = offsetIt->second;

  auto tagPoseOpt = GetTargetPoseCameraSpace();
  if (!tagPoseOpt)
    return std::nullopt;
  frc::Pose3d tagPose_CameraSpace = tagPoseOpt.value();

  frc::Pose3d offsetPose_CameraSpace = tagPose_CameraSpace + offsetTransform;

  return offsetPose_CameraSpace.Translation().Norm();
}

std::optional<units::degree_t> Lemonlight::GetHeadingErrorToOffset(
    int tagID, const std::string& offsetName) {
  if (GetPrimaryTagID() != tagID)
    return std::nullopt;

  auto it = VisionConstants::tagOffsets.find(tagID);
  if (it == VisionConstants::tagOffsets.end())
    return std::nullopt;

  auto offsetIt = it->second.find(offsetName);
  if (offsetIt == it->second.end())
    return std::nullopt;

  frc::Transform3d offsetTransform = offsetIt->second;

  auto tagPoseOpt = GetTargetPoseCameraSpace();
  if (!tagPoseOpt)
    return std::nullopt;
  frc::Pose3d tagPose_CameraSpace = tagPoseOpt.value();

  frc::Pose3d offsetPose_CameraSpace = tagPose_CameraSpace + offsetTransform;

  double x = offsetPose_CameraSpace.X().value();
  double y = offsetPose_CameraSpace.Y().value();

  return units::radian_t{std::atan2(y, x)};
}

std::optional<units::degree_t> Lemonlight::GetTargetTurretHeading(
    units::degree_t currentTurretHeading, int tagID,
    const std::string& offsetName) {
  auto error = GetHeadingErrorToOffset(tagID, offsetName);
  if (!error)
    return std::nullopt;

  // Turret heading + angle error in camera space = absolute heading to aim at
  return currentTurretHeading + error.value();
}

std::optional<units::meter_t> Lemonlight::GetDistanceToTag() {
  auto tagPoseOpt = GetTargetPoseCameraSpace();
  if (!tagPoseOpt)
    return std::nullopt;
  return tagPoseOpt.value().Translation().Norm();
}

std::optional<units::degree_t> Lemonlight::GetHeadingErrorToTag() {
  auto tagPoseOpt = GetTargetPoseCameraSpace();
  if (!tagPoseOpt)
    return std::nullopt;

  double x = tagPoseOpt.value().X().value();
  double y = tagPoseOpt.value().Y().value();

  return units::radian_t{std::atan2(y, x)};
}
