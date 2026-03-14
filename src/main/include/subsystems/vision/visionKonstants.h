#pragma once

#include <array>
#include <optional>
#include <string_view>

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/geometry/Rotation3d.h>
#include <units/length.h>

/**
 * VisionKonstants
 *
 * Define named aiming/scoring targets as offsets from AprilTag fiducial IDs.
 * Instead of targeting the tag itself, you target a named point in space that
 * is a fixed XYZ offset away from the tag center.
 *
 * Offset convention (WPILib tag-face frame):
 *   x (length) — positive = away from the tag face (toward the field)
 *   y (depth)  — positive = left when standing in front of the tag
 *   z (height) — positive = upward from tag center
 *
 * To add a target:
 *   1. Add a constexpr VisionTarget below with the tag ID and XYZ offset.
 *   2. Add it to kAllTargets so findByName / findByTagId can return it.
 *
 * Usage example (from a command or subsystem):
 *   auto target = VisionKonstants::findByName("HubCenter");
 *   if (target && photonResult.HasTargets()) {
 *       auto camToTag = photonResult.GetBestTarget().GetBestCameraToTarget();
 *       auto camToTarget = target->cameraToTarget(camToTag);
 *       units::meter_t dist = camToTarget.Translation().Norm();
 *   }
 */

namespace VisionKonstants {

// ---------------------------------------------------------------------------
// VisionTarget — a named point offset from an AprilTag center.
// All offsets are in meters using WPILib units.
// ---------------------------------------------------------------------------
struct VisionTarget {
    std::string_view name;  ///< Human-readable label (e.g. "HubCenter")
    int tagId;              ///< AprilTag fiducial ID this target is anchored to

    units::meter_t x;  ///< length  — in front of (+) / behind (-) the tag face
    units::meter_t y;  ///< depth   — left (+) / right (-) when facing the tag
    units::meter_t z;  ///< height  — above (+) / below (-) the tag center

    // -----------------------------------------------------------------------
    // Transform helpers
    // -----------------------------------------------------------------------

    /** Transform from this tag's origin to the target point (no rotation). */
    frc::Transform3d tagToTarget() const {
        return frc::Transform3d{
            frc::Translation3d{x, y, z},
            frc::Rotation3d{}
        };
    }

    /**
     * Transform from camera to the target point, given the camera-to-tag
     * transform reported by PhotonVision (e.g. GetBestCameraToTarget()).
     *
     * @param cameraToTag  Transform3d from camera origin to the AprilTag.
     * @return             Transform3d from camera origin to the target point.
     */
    frc::Transform3d cameraToTarget(const frc::Transform3d& cameraToTag) const {
        frc::Pose3d origin{};
        frc::Pose3d tagInCam   = origin.TransformBy(cameraToTag);
        frc::Pose3d targetInCam = tagInCam.TransformBy(tagToTarget());
        return frc::Transform3d{origin, targetInCam};
    }

    /**
     * XYZ translation (meters) from the camera to the target point.
     *   .X() = left/right,  .Y() = up/down,  .Z() = forward depth
     * (Axes are in camera frame — match your PhotonVision coordinate output.)
     *
     * @param cameraToTag  Transform3d from camera origin to the AprilTag.
     */
    frc::Translation3d cameraToTargetTranslation(const frc::Transform3d& cameraToTag) const {
        return cameraToTarget(cameraToTag).Translation();
    }

    /**
     * Transform from the robot center to the target point, given the
     * robot-to-tag transform (derived from field pose math or odometry).
     *
     * @param robotToTag  Transform3d from robot center to the AprilTag.
     * @return            Transform3d from robot center to the target point.
     */
    frc::Transform3d robotToTarget(const frc::Transform3d& robotToTag) const {
        frc::Pose3d origin{};
        frc::Pose3d tagInRobot    = origin.TransformBy(robotToTag);
        frc::Pose3d targetInRobot = tagInRobot.TransformBy(tagToTarget());
        return frc::Transform3d{origin, targetInRobot};
    }

    /**
     * XYZ translation (meters) from the robot center to the target point.
     *
     * @param robotToTag  Transform3d from robot center to the AprilTag.
     */
    frc::Translation3d robotToTargetTranslation(const frc::Transform3d& robotToTag) const {
        return robotToTarget(robotToTag).Translation();
    }
};

// ---------------------------------------------------------------------------
// Named targets
//
// Edit these for your field geometry. Tag IDs and offsets are placeholders —
// measure physical positions and update before competition.
//
// FRC 2026 tag IDs: check the field manual or your FRC2026_WELDED.fmap file.
//
// x = meters forward from tag face
// y = meters left when facing the tag
// z = meters up from tag center
// ---------------------------------------------------------------------------

/// High-goal hub center — above the hub tag, centered on the opening.
constexpr VisionTarget kHubCenter {
    "HubCenter",
    /*tagId=*/ 4,
    /*x=*/  0.0_m,   // on the tag plane (no forward offset into the hub)
    /*y=*/  0.0_m,   // centered laterally
    /*z=*/  0.5_m    // TODO: measure — 0.5 m above tag center as placeholder
};

/// Loading / human-player station intake target.
constexpr VisionTarget kLoadingStation {
    "LoadingStation",
    /*tagId=*/ 1,
    /*x=*/  0.3_m,   // TODO: 30 cm in front of the tag — adjust to your intake reach
    /*y=*/  0.0_m,
    /*z=*/  0.0_m
};

// Add more targets here following the same pattern, e.g.:
// constexpr VisionTarget kAmpTarget {
//     "AmpTarget", /*tagId=*/ 5,
//     /*x=*/ 0.0_m, /*y=*/ 0.0_m, /*z=*/ 0.6_m
// };

// ---------------------------------------------------------------------------
// Registry — add every VisionTarget here for look-up by name / tag ID.
// Update the array size when you add or remove entries.
// ---------------------------------------------------------------------------
constexpr std::array<VisionTarget, 2> kAllTargets = {
    kHubCenter,
    kLoadingStation,
};

// ---------------------------------------------------------------------------
// Look-up helpers
// ---------------------------------------------------------------------------

/**
 * Find a registered target by its human-readable name.
 * Returns std::nullopt if no target with that name exists.
 *
 * Example:
 *   auto t = VisionKonstants::findByName("HubCenter");
 */
inline std::optional<VisionTarget> findByName(std::string_view name) {
    for (const auto& t : kAllTargets) {
        if (t.name == name) return t;
    }
    return std::nullopt;
}

/**
 * Find the first registered target anchored to the given AprilTag ID.
 * Returns std::nullopt if no target is registered for that tag.
 *
 * Example:
 *   auto t = VisionKonstants::findByTagId(4);
 */
inline std::optional<VisionTarget> findByTagId(int tagId) {
    for (const auto& t : kAllTargets) {
        if (t.tagId == tagId) return t;
    }
    return std::nullopt;
}

} // namespace VisionKonstants
