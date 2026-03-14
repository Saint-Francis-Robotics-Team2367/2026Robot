#pragma once

#include <unordered_map>
#include <string>
#include <string_view>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/geometry/Rotation3d.h>
#include <units/length.h>
#include <units/angle.h>

namespace VisionConstants {

    // Toggle this boolean before the match to switch between PhotonVision and Limelight libraries
    constexpr bool usePhotonVision = true;

    // Name of the PhotonVision camera configured on the Lemonlight device
    constexpr std::string_view photonCameraName = "Meyer";

    // Physical transform from robot center to the camera lens.
    // +X forward, +Y left, +Z up (robot frame). Update these to match actual mounting.
    inline const frc::Transform3d robotToCamera{
        frc::Translation3d{0.0_m, 0.0_m, 0.0_m},
        frc::Rotation3d{0_rad, 0_rad, 0_rad}
    };

    // Define offset points relative to specific AprilTags (fiducial IDs)
    // Map format: <Tag ID, Map<Offset Name, Transform3d from Tag Center>>
    // +X is forward (out of the tag face), +Y is left relative to the tag face, +Z is up relative to the tag face.
    inline const std::unordered_map<int, std::unordered_map<std::string, frc::Transform3d>> tagOffsets = {
        { 
            7, { // Example for Tag ID 7
                {"SpeakerCenter", frc::Transform3d{frc::Translation3d{0.0_m, 0.5_m, 0.0_m}, frc::Rotation3d{}}},
                {"AmpScoring", frc::Transform3d{frc::Translation3d{-0.2_m, 0.0_m, 0.0_m}, frc::Rotation3d{}}}
            }
        },
        {
            8, { // Example for Tag ID 8
                {"Offset1", frc::Transform3d{frc::Translation3d{1.0_m, 0.0_m, 0.0_m}, frc::Rotation3d{}}}
            }
        }
    };

}
