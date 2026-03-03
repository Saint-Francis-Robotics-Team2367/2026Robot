#pragma once
/**
 * XNavLib - C++ Client Library for XNav Vision System
 *
 * Use this library on your roboRIO to communicate with the XNav
 * vision system over NetworkTables 4 (WPILib NT4).
 *
 * Usage:
 *   #include "XNavLib.h"
 *
 *   // In robotInit():
 *   xnav::XNav vision;
 *   vision.Init();  // connects to NT server
 *
 *   // In teleopPeriodic():
 *   if (vision.HasTarget()) {
 *     auto target = vision.GetPrimaryTarget();
 *     double dist = target.distance;
 *     double tx   = target.tx;
 *   }
 *
 *   // Field-centric robot pose:
 *   auto pose = vision.GetRobotPose();
 *   if (pose.valid) {
 *     frc::Pose2d robotPose(pose.x, pose.y, frc::Rotation2d(pose.yaw_deg));
 *   }
 *
 * See docs/nt_topics.md for the full NT topic reference.
 */

#include <string>
#include <vector>
#include <optional>
#include <memory>
#include <functional>

// WPILib headers (available in FRC toolchain)
#ifdef WPILIB_AVAILABLE
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/DoubleTopic.h>
#include <networktables/BooleanTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/StringTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/IntegerArrayTopic.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>
#endif

namespace xnav {

// ─────────────────────────────────────────────────────────────────────────────
// Data structures
// ─────────────────────────────────────────────────────────────────────────────

/** Single detected AprilTag result. */
struct TagResult {
    int    id       = -1;
    double tx       = 0.0;   ///< Horizontal angle from camera center (degrees)
    double ty       = 0.0;   ///< Vertical angle from camera center (degrees)
    double x        = 0.0;   ///< X in camera frame (meters, right is positive)
    double y        = 0.0;   ///< Y in camera frame (meters, down is positive)
    double z        = 0.0;   ///< Z in camera frame (meters, forward is positive)
    double distance = 0.0;   ///< Direct 3D distance (meters)
    double yaw      = 0.0;   ///< Tag yaw relative to camera (degrees)
    double pitch    = 0.0;   ///< Tag pitch relative to camera (degrees)
    double roll     = 0.0;   ///< Tag roll relative to camera (degrees)
};

/** Robot field-centric pose estimated from AprilTags. */
struct RobotPose {
    double x       = 0.0;    ///< X on field (meters)
    double y       = 0.0;    ///< Y on field (meters)
    double z       = 0.0;    ///< Z on field (meters, usually ~0)
    double roll    = 0.0;    ///< Roll (degrees)
    double pitch   = 0.0;    ///< Pitch (degrees)
    double yaw_deg = 0.0;    ///< Yaw / heading (degrees)
    bool   valid   = false;  ///< True if pose is available
};

/** Result for configured offset point. */
struct OffsetPoint {
    int    tag_id          = -1;
    double x               = 0.0; ///< X to offset point in camera frame (m)
    double y               = 0.0;
    double z               = 0.0;
    double direct_distance = 0.0; ///< 3D distance to point (m)
    double tx              = 0.0; ///< Horizontal angle to point (deg)
    double ty              = 0.0; ///< Vertical angle to point (deg)
    bool   valid           = false;
};

/** XNav system status. */
struct SystemStatus {
    std::string status;           ///< "running", "starting", "error"
    double      fps        = 0.0;
    double      latency_ms = 0.0;
    int         num_targets = 0;
    bool        nt_connected = false;
};

// ─────────────────────────────────────────────────────────────────────────────
// Main XNav class
// ─────────────────────────────────────────────────────────────────────────────

class XNav {
public:
    /**
     * @brief Constructor.
     * @param table_name  NT table name (default "XNav", must match XNav config)
     */
    explicit XNav(const std::string& table_name = "XNav");
    ~XNav();

    // ── Initialization ───────────────────────────────────────────────────────

    /**
     * @brief Initialize and connect to the NT server.
     * Call once in robotInit(). The library connects to the standard
     * robot network (team number auto-detection via WPILib).
     */
    void Init();

    /**
     * @brief Initialize with explicit server address.
     * @param server_address IP or hostname of the NT server (XNav device)
     */
    void Init(const std::string& server_address);

    // ── Detection results ─────────────────────────────────────────────────────

    /** @return True if at least one tag is currently detected. */
    bool HasTarget() const;

    /** @return Number of currently detected tags. */
    int GetNumTargets() const;

    /** @return IDs of all currently detected tags. */
    std::vector<int> GetTagIds() const;

    /**
     * @brief Get data for the primary (closest) detected tag.
     * Check id != -1 to confirm a target exists.
     */
    TagResult GetPrimaryTarget() const;

    /**
     * @brief Get data for a specific tag by ID.
     * @return Empty optional if tag is not currently visible.
     */
    std::optional<TagResult> GetTarget(int tag_id) const;

    /**
     * @brief Get all currently detected tags.
     */
    std::vector<TagResult> GetAllTargets() const;

    // ── Robot pose ────────────────────────────────────────────────────────────

    /**
     * @brief Get robot field-centric pose (requires .fmap to be loaded on XNav).
     * @return RobotPose with valid=true if pose is available.
     */
    RobotPose GetRobotPose() const;

    // ── Offset point ─────────────────────────────────────────────────────────

    /**
     * @brief Get distance/angles to the configured offset point.
     * Configure the offset in the XNav dashboard or via NT.
     */
    OffsetPoint GetOffsetPoint() const;

    // ── Turret control ────────────────────────────────────────────────────────

    /**
     * @brief Send turret angle to XNav for pose compensation.
     * @param angle_deg  Turret rotation in degrees (positive = CCW from above)
     */
    void SetTurretAngle(double angle_deg);

    /**
     * @brief Enable or disable turret compensation on XNav.
     */
    void SetTurretEnabled(bool enabled);

    // ── Match mode ────────────────────────────────────────────────────────────

    /**
     * @brief Enable match mode (maximum performance).
     * Squeezes every bit of performance from the hardware.
     */
    void SetMatchMode(bool enabled);

    // ── System status ─────────────────────────────────────────────────────────

    /** @return Current XNav system status. */
    SystemStatus GetStatus() const;

    /** @return True if NT connection to XNav is active. */
    bool IsConnected() const;

    // ── Callbacks ─────────────────────────────────────────────────────────────

    /**
     * @brief Register a callback invoked when new target data arrives.
     * Called from NT listener thread.
     */
    void OnNewTargets(std::function<void(const std::vector<TagResult>&)> callback);

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace xnav