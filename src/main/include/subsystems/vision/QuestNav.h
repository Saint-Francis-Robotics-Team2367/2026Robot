#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/RawTopic.h>
#include <frc/geometry/Pose3d.h>
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "protoh/data.pb.h"
#include "protoh/geometry3d.pb.h"
#include "Singleton.h"

class QuestNav : public Singleton<QuestNav> {
private:
    nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table = inst.GetTable("QuestNav");
    nt::PubSubOptions options = nt::PubSubOptions{};
    nt::RawSubscriber m_questNavSub;
    questnav::protos::data::ProtobufQuestNavFrameData m_frame;
    units::radian_t yawOffset = 0_rad;
public:
    frc::Pose3d robotPose;
    frc::Quaternion robotQuaternion;

    void init() {
        // Questnav set up
        options.periodic = 0.01f;
        options.sendAll = true;
        options.pollStorage = 20;
        // Subscribe to the raw bytes.
        m_questNavSub = table->GetRawTopic("frameData")
                            .Subscribe("proto:questnav.protos.data.ProtobufQuestNavFrameData", {}, options);
    }

    void periodic() {
        std::vector<uint8_t> rawBytes = m_questNavSub.Get({});

        if (rawBytes.empty()) return;

        // 2. Parse the bytes using your generated class
        if (m_frame.ParseFromArray(rawBytes.data(), rawBytes.size())) {

        // 3. Check if tracking is valid
        // Note: In your header, the accessor is 'istracking()', NOT 'is_tracking()'
        if (m_frame.istracking()) {
            
            // 4. Extract the Pose
            // The header shows 'pose3d()' returns a 'wpi::proto::ProtobufPose3d'
            auto protoPose = m_frame.pose3d();
            auto trans = protoPose.translation();
            auto protoRot = protoPose.rotation();

            // 5. Convert to FRC standard types (frc::Pose3d)
            // Note: You may need to check geometry3d.pb.h to see if quaternion fields 
            // are named (w, x, y, z) or (q_w, q_x, etc). Assuming standard names here:
            robotPose = frc::Pose3d(
                units::meter_t{trans.x()},
                units::meter_t{trans.y()},
                units::meter_t{trans.z()},
                frc::Rotation3d(
                    frc::Quaternion(protoRot.q().w(), protoRot.q().x(), protoRot.q().y(), protoRot.q().z())
                )
            );
            
            robotQuaternion = frc::Quaternion(protoRot.q().w(), protoRot.q().x(), protoRot.q().y(), protoRot.q().z());

            frc::SmartDashboard::PutNumber("Robot Pose X", robotPose.X().value());
            frc::SmartDashboard::PutNumber("Robot Pose Y", robotPose.Y().value());
        }
        }
    }

    void ZeroGyro() {
        yawOffset = robotPose.Rotation().Z(); 
    }

    frc::Rotation2d getRotation2d() {
        units::radian_t rawYaw = robotPose.Rotation().Z();
        units::radian_t correctedYaw = rawYaw - yawOffset;
        return frc::Rotation2d(correctedYaw);
    }

    frc::Rotation3d getRotation3d() {
        return frc::Rotation3d(robotQuaternion);
    }

    frc::Pose3d getPose3d() {
        return robotPose;
    }

    frc::Pose2d getPose2d() {
        return frc::Pose2d(robotPose.X(), robotPose.Y(), getRotation2d());
    }

    frc::Rotation2d getBoundedAngleCCW()
    {
        return frc::Rotation2d(robotPose.Rotation().Z());
    }

    frc::Rotation2d getBoundedAngleCW()
    {
        return frc::Rotation2d(-1*robotPose.Rotation().Z());
    }
};