#pragma once

#include <optional>
#include <string>

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include <units/length.h>

#include "subsystems/vision/QuestNavCalibration.h"
#include "subsystems/vision/QuestNav.h"

/**
 * PhotonVision
 *
 * Wraps a PhotonCamera ("Meyer") running on the "Lemonlight" coprocessor and a
 * PhotonPoseEstimator backed by the FRC 2026 welded field layout.
 *
 * Primary job at robot start-up:
 *   1. Wait until PhotonVision reports a confident multi-tag estimate.
 *   2. Call QuestNavCalibration::calibrate() with that estimate and the
 *      simultaneous QuestNav pose.
 *   3. From that point forward the calibration object can convert any QuestNav
 *      pose into a field-absolute pose.
 *
 * The .fmap file must be deployed to the RoboRIO:
 *   Place FRC2026_WELDED.fmap in src/main/deploy/ so GradleRIO copies it to
 *   /home/lvuser/deploy/ on the robot.
 *
 * IMPORTANT – camera-to-robot transform:
 *   kRobotToCamera below is a placeholder.  Measure the actual Limelight3
 *   mounting position relative to the robot's center (positive X = forward,
 *   positive Y = left, positive Z = up) and update accordingly.
 */
class PhotonVision {
public:
    PhotonVision();

    /** Called once during robot init (after QuestNav::init()). */
    void init();

    /**
     * Called every 20 ms from Robot::RobotPeriodic().
     * Polls all new camera results, runs the pose estimator, and stores the
     * latest confident estimate.
     *
     * If the provided calibration object is not yet calibrated, this method
     * will attempt to calibrate it using the current QuestNav pose.
     */
    void periodic(QuestNavCalibration& calibration);

    /** Returns the most recent field-absolute 2D pose estimate, if any. */
    std::optional<frc::Pose2d> getLatestPose() const;

    /** True if the camera is connected and transmitting data. */
    bool isConnected() const;

    /** True if the last periodic call produced a confident pose estimate. */
    bool hasConfidentEstimate() const;

    // -----------------------------------------------------------------------
    // Camera-to-robot transform
    // Describes where the Limelight3 is mounted relative to the robot center.
    // Positive X = forward, positive Y = left, positive Z = up (WPILib frame).
    // Yaw = 0 means camera faces the same direction as the robot's front.
    //
    // TODO: Replace these with your measured values before competition.
    // -----------------------------------------------------------------------
    static const frc::Transform3d kRobotToCamera;

    // Minimum number of AprilTags that must be visible before we trust the
    // estimate enough to use it for calibration.
    static constexpr int kMinTagsForCalibration = 1;

    // Maximum single-tag pose ambiguity accepted for calibration (lower = more
    // confident).  Not used when multiple tags are visible.
    static constexpr double kMaxAmbiguityForCalibration = 0.15;

private:
    photon::PhotonCamera m_camera{"Meyer"};
    std::optional<photon::PhotonPoseEstimator> m_poseEstimator;
    std::optional<frc::AprilTagFieldLayout> m_fieldLayout;

    std::optional<frc::Pose2d> m_latestPose;
    bool m_hasConfidentEstimate = false;
    bool m_initialized = false;

    void tryLoadFieldLayout();
};
