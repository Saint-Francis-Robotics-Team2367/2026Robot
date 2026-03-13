#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Pose3d.h>
#include <units/length.h>
#include <units/angle.h>
#include <string>
#include <optional>

class Lemonlight : public frc2::SubsystemBase {
 public:
  Lemonlight();

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

  // For debugging, get the raw distance to the tag itself
  std::optional<units::meter_t> GetDistanceToTag();

  // For debugging, get the raw heading error to face the tag itself
  std::optional<units::degree_t> GetHeadingErrorToTag();

 private:
  std::string m_limelightName = "lemonlight";
};
