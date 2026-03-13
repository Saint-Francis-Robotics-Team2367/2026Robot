#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Rotation2d.h>

/**
 * QuestNavCalibration
 *
 * Stores a one-time calibration snapshot between a PhotonVision field-absolute
 * pose and the corresponding QuestNav relative pose.  After calibration, any
 * QuestNav pose can be converted to a field-absolute pose.
 *
 * Math: at calibration time both sensors observe the same physical robot pose.
 *   questDelta  = currentQuestPose.RelativeTo(questPoseAtCalib)
 *   fieldPose   = photonPoseAtCalib.TransformBy(questDelta)
 *
 * This correctly handles both translation AND rotation differences between the
 * QuestNav coordinate frame and the field coordinate frame.
 */
class QuestNavCalibration {
public:
    /**
     * Record a calibration snapshot.
     *
     * @param photonPose  Field-absolute pose from PhotonVision at calibration time.
     * @param questPose   QuestNav relative pose at the same instant.
     */
    void calibrate(frc::Pose2d photonPose, frc::Pose2d questPose) {
        m_photonPoseAtCalib = photonPose;
        m_questPoseAtCalib  = questPose;
        m_calibrated = true;
    }

    bool isCalibrated() const { return m_calibrated; }

    /**
     * Convert any QuestNav pose to a field-absolute pose using the stored
     * calibration.  Returns an identity pose if not yet calibrated.
     */
    frc::Pose2d getCorrectedPose(frc::Pose2d currentQuestPose) const {
        if (!m_calibrated) {
            return frc::Pose2d{};
        }

        // Delta expressed in the robot's local frame at calibration time
        // (RelativeTo removes the calibration-point offset and rotates the
        //  translation into the calibration heading's frame).
        frc::Pose2d questDelta = currentQuestPose.RelativeTo(m_questPoseAtCalib);

        // Apply the same delta starting from the photon calibration pose.
        // TransformBy rotates questDelta's translation by photonPoseAtCalib's
        // heading, which maps from "robot-relative at calibration" → field frame.
        return m_photonPoseAtCalib.TransformBy(
            frc::Transform2d{frc::Pose2d{}, questDelta}
        );
    }

    /** Convenience: return just the calibrated heading offset (field − quest). */
    frc::Rotation2d getRotationOffset() const {
        if (!m_calibrated) return frc::Rotation2d{};
        return m_photonPoseAtCalib.Rotation() - m_questPoseAtCalib.Rotation();
    }

    frc::Pose2d getPhotonPoseAtCalib() const { return m_photonPoseAtCalib; }
    frc::Pose2d getQuestPoseAtCalib()  const { return m_questPoseAtCalib;  }

private:
    frc::Pose2d m_photonPoseAtCalib;
    frc::Pose2d m_questPoseAtCalib;
    bool m_calibrated = false;
};
