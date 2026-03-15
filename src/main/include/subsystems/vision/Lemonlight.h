#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Pose3d.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <units/length.h>
#include <units/angle.h>
#include <string>
#include <optional>
#include <memory>
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>

class DriveSubsystem;

class Lemonlight : public frc2::SubsystemBase {
 public:
  explicit Lemonlight(DriveSubsystem& drive);

  void Periodic() override;

  bool HasTarget();
  int GetPrimaryTagID();

  // Calculate distance in meters to an offset point relative to an AprilTag
  std::optional<units::meter_t> GetDistanceToOffset(int tagID, const std::string& offsetName);

  // Calculate the heading error (yaw) in degrees to the offset point.
  // Because the Limelight is ON the turret, this error is how many degrees 
  // the turret must turn from its CURRENT position to face the offset point.
  std::optional<units::degree_t> GetHeadingErrorToOffset(int tagID, const std::string& offsetName);

  // Feed the current turret heading, and get the target turret heading to aim at the offset point.
  std::optional<units::degree_t> GetTargetTurretHeading(units::degree_t currentTurretHeading, int tagID, const std::string& offsetName);

  // Returns a field-space robot pose estimate from the latest AprilTag result.
  // Returns nullopt if PhotonVision is disabled or no tags are visible.
  std::optional<frc::Pose2d> GetEstimatedFieldPose();

  // For debugging, get the raw distance to the tag itself
  std::optional<units::meter_t> GetDistanceToTag();

  // For debugging, get the raw heading error to face the tag itself
  std::optional<units::degree_t> GetHeadingErrorToTag();

 private:
  DriveSubsystem& m_drive;

  // Limelight hostname (used by LimelightHelpers for the non-PhotonVision path)
  std::string m_limelightName = "lemonlight";

  std::unique_ptr<photon::PhotonCamera> m_photonCamera;
  frc::AprilTagFieldLayout m_fieldLayout;
  std::optional<photon::PhotonPoseEstimator> m_poseEstimator;

  // Helper method to retrieve the target pose dynamically based on the current vision provider
  std::optional<frc::Pose3d> GetTargetPoseCameraSpace();

  // Feed latest PhotonVision pose estimates into the drive pose estimator
  void UpdateDrivePose();
};
