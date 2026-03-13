#include "subsystems/vision/PhotonVision.h"

#include <frc/Filesystem.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <stdexcept>

// ---------------------------------------------------------------------------
// Camera-to-robot transform definition
//
// Placeholder: camera is mounted 0.30 m forward and 0.50 m above the robot
// center, pointing straight forward (no tilt, no yaw offset).
//
// Measure your actual Limelight3 position and update these values:
//   - Translation: (forward_m, left_m, up_m) from robot center
//   - Rotation3d pitch < 0 tilts camera downward (typical for tags on a wall)
// ---------------------------------------------------------------------------
const frc::Transform3d PhotonVision::kRobotToCamera{
    frc::Translation3d{0.30_m, 0.00_m, 0.50_m},
    frc::Rotation3d{0_rad, 0_rad, 0_rad}   // TODO: tune pitch/yaw if needed
};

PhotonVision::PhotonVision() {}

void PhotonVision::init() {
    tryLoadFieldLayout();
}

void PhotonVision::tryLoadFieldLayout() {
    std::string layoutPath =
        frc::filesystem::GetDeployDirectory() + "/FRC2026_WELDED.fmap";
    try {
        m_fieldLayout = frc::AprilTagFieldLayout{layoutPath};
        m_poseEstimator.emplace(
            m_fieldLayout.value(),
            photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
            kRobotToCamera
        );
        m_initialized = true;
        frc::SmartDashboard::PutBoolean("PhotonVision/LayoutLoaded", true);
    } catch (const std::exception& e) {
        m_initialized = false;
        frc::SmartDashboard::PutBoolean("PhotonVision/LayoutLoaded", false);
        frc::SmartDashboard::PutString("PhotonVision/LayoutError", e.what());
    }
}

void PhotonVision::periodic(QuestNavCalibration& calibration) {
    frc::SmartDashboard::PutBoolean("PhotonVision/Connected", m_camera.IsConnected());
    frc::SmartDashboard::PutBoolean("PhotonVision/Initialized", m_initialized);

    if (!m_initialized) {
        // Retry layout loading each cycle until it succeeds
        tryLoadFieldLayout();
        m_hasConfidentEstimate = false;
        return;
    }

    m_hasConfidentEstimate = false;
    auto& estimator = m_poseEstimator.value();

    // Process every unread result so we don't fall behind
    for (auto& result : m_camera.GetAllUnreadResults()) {
        // Try multi-tag estimate first (most accurate)
        auto visionEst = estimator.EstimateCoprocMultiTagPose(result);

        // Fall back to single-tag lowest-ambiguity if multi-tag unavailable
        if (!visionEst.has_value()) {
            visionEst = estimator.EstimateLowestAmbiguityPose(result);
        }

        if (!visionEst.has_value()) {
            continue;
        }

        const auto& est = visionEst.value();
        int numTags = static_cast<int>(est.targetsUsed.size());

        // Assess confidence: require enough tags or low ambiguity
        bool confident = false;
        if (numTags >= 2) {
            confident = true;
        } else if (numTags == 1) {
            double ambiguity = est.targetsUsed[0].GetPoseAmbiguity();
            confident = (ambiguity >= 0.0 && ambiguity <= kMaxAmbiguityForCalibration);
        }

        frc::SmartDashboard::PutNumber("PhotonVision/TagCount",  numTags);
        frc::SmartDashboard::PutBoolean("PhotonVision/Confident", confident);

        if (confident) {
            frc::Pose2d pose2d = est.estimatedPose.ToPose2d();
            m_latestPose           = pose2d;
            m_hasConfidentEstimate = true;

            frc::SmartDashboard::PutNumber("PhotonVision/PoseX", pose2d.X().value());
            frc::SmartDashboard::PutNumber("PhotonVision/PoseY", pose2d.Y().value());
            frc::SmartDashboard::PutNumber("PhotonVision/PoseHeadingDeg",
                pose2d.Rotation().Degrees().value());

            // Calibrate QuestNav on the first confident estimate
            if (!calibration.isCalibrated()) {
                frc::Pose2d questPose = QuestNav::getInstance().getPose2d();
                calibration.calibrate(pose2d, questPose);
                frc::SmartDashboard::PutBoolean("PhotonVision/Calibrated", true);
                frc::SmartDashboard::PutNumber("PhotonVision/CalibPhotonX", pose2d.X().value());
                frc::SmartDashboard::PutNumber("PhotonVision/CalibPhotonY", pose2d.Y().value());
                frc::SmartDashboard::PutNumber("PhotonVision/CalibQuestX",  questPose.X().value());
                frc::SmartDashboard::PutNumber("PhotonVision/CalibQuestY",  questPose.Y().value());
                frc::SmartDashboard::PutNumber("PhotonVision/RotOffsetDeg",
                    calibration.getRotationOffset().Degrees().value());
            }
        }
    }
}

std::optional<frc::Pose2d> PhotonVision::getLatestPose() const {
    return m_latestPose;
}

bool PhotonVision::isConnected() const {
    return true;
}

bool PhotonVision::hasConfidentEstimate() const {
    return m_hasConfidentEstimate;
}
