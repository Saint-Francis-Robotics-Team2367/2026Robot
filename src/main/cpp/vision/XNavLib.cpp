/**
 * XNavLib.cpp - Implementation of the XNav client library.
 *
 * This implementation uses WPILib NetworkTables 4 (ntcore).
 * Compatible with FRC robots running the standard WPILib stack.
 */

#include "vision/XNavLib.h"

#include <sstream>
#include <mutex>
#include <algorithm>

#ifdef WPILIB_AVAILABLE
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/DoubleTopic.h>
#include <networktables/BooleanTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/StringTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/IntegerArrayTopic.h>
#endif

namespace xnav {

// ─────────────────────────────────────────────────────────────────────────────
// PIMPL implementation
// ─────────────────────────────────────────────────────────────────────────────

struct XNav::Impl {
    std::string table_name;
    std::function<void(const std::vector<TagResult>&)> on_new_targets;
    mutable std::mutex data_mutex;

#ifdef WPILIB_AVAILABLE
    nt::NetworkTableInstance inst;
    std::shared_ptr<nt::NetworkTable> table;

    // Subscribers
    nt::BooleanSubscriber  sub_has_target;
    nt::IntegerSubscriber  sub_num_targets;
    nt::IntegerSubscriber  sub_primary_id;
    nt::StringSubscriber   sub_status;
    nt::DoubleSubscriber   sub_fps;
    nt::DoubleSubscriber   sub_latency;
    nt::DoubleArraySubscriber sub_robot_pose;
    // Offset point
    nt::BooleanSubscriber  sub_offset_valid;
    nt::DoubleSubscriber   sub_offset_x, sub_offset_y, sub_offset_z;
    nt::DoubleSubscriber   sub_offset_dist, sub_offset_tx, sub_offset_ty;
    nt::IntegerSubscriber  sub_offset_tagid;

    // Publishers (inputs to XNav)
    nt::DoublePublisher  pub_turret_angle;
    nt::BooleanPublisher pub_turret_enabled;
    nt::BooleanPublisher pub_match_mode;

    // Per-tag subscribers (created lazily)
    struct TagSubs {
        nt::DoubleSubscriber tx, ty, x, y, z, distance, yaw, pitch, roll;
    };
    std::unordered_map<int, TagSubs> tag_subs;
    nt::IntegerArraySubscriber sub_tag_ids;

    void Init(const std::string& server) {
        inst = nt::NetworkTableInstance::GetDefault();
        table = inst.GetTable(table_name);

        sub_has_target  = table->GetBooleanTopic("hasTarget").Subscribe(false);
        sub_num_targets = table->GetIntegerTopic("numTargets").Subscribe(0);
        sub_primary_id  = table->GetIntegerTopic("primaryTagId").Subscribe(-1);
        sub_status      = table->GetStringTopic("status").Subscribe("unknown");
        sub_fps         = table->GetDoubleTopic("fps").Subscribe(0.0);
        sub_latency     = table->GetDoubleTopic("latencyMs").Subscribe(0.0);
        sub_robot_pose  = table->GetDoubleArrayTopic("robotPose").Subscribe({});
        sub_tag_ids     = table->GetIntegerArrayTopic("tagIds").Subscribe({});

        auto op = table->GetSubTable("offsetPoint");
        sub_offset_valid  = op->GetBooleanTopic("valid").Subscribe(false);
        sub_offset_x      = op->GetDoubleTopic("x").Subscribe(0.0);
        sub_offset_y      = op->GetDoubleTopic("y").Subscribe(0.0);
        sub_offset_z      = op->GetDoubleTopic("z").Subscribe(0.0);
        sub_offset_dist   = op->GetDoubleTopic("directDistance").Subscribe(0.0);
        sub_offset_tx     = op->GetDoubleTopic("tx").Subscribe(0.0);
        sub_offset_ty     = op->GetDoubleTopic("ty").Subscribe(0.0);
        sub_offset_tagid  = op->GetIntegerTopic("tag_id").Subscribe(-1);

        auto input = table->GetSubTable("input");
        pub_turret_angle   = input->GetDoubleTopic("turretAngle").Publish();
        pub_turret_enabled = input->GetBooleanTopic("turretEnabled").Publish();
        pub_match_mode     = input->GetBooleanTopic("matchMode").Publish();

        if (!server.empty()) {
            inst.SetServer(server.c_str());
        }
        inst.StartClient4("XNavLib");
    }

    TagSubs& GetTagSubs(int id) {
        auto it = tag_subs.find(id);
        if (it != tag_subs.end()) return it->second;
        auto sub = table->GetSubTable("targets/" + std::to_string(id));
        TagSubs ts;
        ts.tx       = sub->GetDoubleTopic("tx").Subscribe(0.0);
        ts.ty       = sub->GetDoubleTopic("ty").Subscribe(0.0);
        ts.x        = sub->GetDoubleTopic("x").Subscribe(0.0);
        ts.y        = sub->GetDoubleTopic("y").Subscribe(0.0);
        ts.z        = sub->GetDoubleTopic("z").Subscribe(0.0);
        ts.distance = sub->GetDoubleTopic("distance").Subscribe(0.0);
        ts.yaw      = sub->GetDoubleTopic("yaw").Subscribe(0.0);
        ts.pitch    = sub->GetDoubleTopic("pitch").Subscribe(0.0);
        ts.roll     = sub->GetDoubleTopic("roll").Subscribe(0.0);
        tag_subs[id] = std::move(ts);
        return tag_subs[id];
    }

    TagResult ReadTag(int id) {
        auto& ts = GetTagSubs(id);
        TagResult t;
        t.id       = id;
        t.tx       = ts.tx.Get();
        t.ty       = ts.ty.Get();
        t.x        = ts.x.Get();
        t.y        = ts.y.Get();
        t.z        = ts.z.Get();
        t.distance = ts.distance.Get();
        t.yaw      = ts.yaw.Get();
        t.pitch    = ts.pitch.Get();
        t.roll     = ts.roll.Get();
        return t;
    }
#else
    // Stub implementations when WPILib is not available (for unit testing on desktop)
    void Init(const std::string&) {}
    TagResult ReadTag(int id) { TagResult t; t.id = id; return t; }
#endif
};

// ─────────────────────────────────────────────────────────────────────────────
// XNav public API
// ─────────────────────────────────────────────────────────────────────────────

XNav::XNav(const std::string& table_name)
    : m_impl(std::make_unique<Impl>()) {
    m_impl->table_name = table_name;
}

XNav::~XNav() = default;

void XNav::Init() {
    m_impl->Init("");
}

void XNav::Init(const std::string& server_address) {
    m_impl->Init(server_address);
}

bool XNav::HasTarget() const {
#ifdef WPILIB_AVAILABLE
    return m_impl->sub_has_target.Get();
#else
    return false;
#endif
}

int XNav::GetNumTargets() const {
#ifdef WPILIB_AVAILABLE
    return static_cast<int>(m_impl->sub_num_targets.Get());
#else
    return 0;
#endif
}

std::vector<int> XNav::GetTagIds() const {
#ifdef WPILIB_AVAILABLE
    auto raw = m_impl->sub_tag_ids.Get();
    std::vector<int> ids;
    ids.reserve(raw.size());
    for (auto v : raw) ids.push_back(static_cast<int>(v));
    return ids;
#else
    return {};
#endif
}

TagResult XNav::GetPrimaryTarget() const {
#ifdef WPILIB_AVAILABLE
    int id = static_cast<int>(m_impl->sub_primary_id.Get(-1));
    if (id < 0) return TagResult{};
    return m_impl->ReadTag(id);
#else
    return TagResult{};
#endif
}

std::optional<TagResult> XNav::GetTarget(int tag_id) const {
#ifdef WPILIB_AVAILABLE
    auto ids = GetTagIds();
    auto it = std::find(ids.begin(), ids.end(), tag_id);
    if (it == ids.end()) return std::nullopt;
    return m_impl->ReadTag(tag_id);
#else
    return std::nullopt;
#endif
}

std::vector<TagResult> XNav::GetAllTargets() const {
    std::vector<TagResult> results;
    for (int id : GetTagIds()) {
        results.push_back(m_impl->ReadTag(id));
    }
    return results;
}

RobotPose XNav::GetRobotPose() const {
    RobotPose pose;
#ifdef WPILIB_AVAILABLE
    auto data = m_impl->sub_robot_pose.Get({});
    if (data.size() >= 6) {
        pose.x       = data[0];
        pose.y       = data[1];
        pose.z       = data[2];
        pose.roll    = data[3];
        pose.pitch   = data[4];
        pose.yaw_deg = data[5];
        pose.valid   = true;
    }
#endif
    return pose;
}

OffsetPoint XNav::GetOffsetPoint() const {
    OffsetPoint op;
#ifdef WPILIB_AVAILABLE
    op.valid           = m_impl->sub_offset_valid.Get(false);
    op.tag_id          = static_cast<int>(m_impl->sub_offset_tagid.Get(-1));
    op.x               = m_impl->sub_offset_x.Get(0.0);
    op.y               = m_impl->sub_offset_y.Get(0.0);
    op.z               = m_impl->sub_offset_z.Get(0.0);
    op.direct_distance = m_impl->sub_offset_dist.Get(0.0);
    op.tx              = m_impl->sub_offset_tx.Get(0.0);
    op.ty              = m_impl->sub_offset_ty.Get(0.0);
#endif
    return op;
}

void XNav::SetTurretAngle(double angle_deg) {
#ifdef WPILIB_AVAILABLE
    m_impl->pub_turret_angle.Set(angle_deg);
#endif
}

void XNav::SetTurretEnabled(bool enabled) {
#ifdef WPILIB_AVAILABLE
    m_impl->pub_turret_enabled.Set(enabled);
#endif
}

void XNav::SetMatchMode(bool enabled) {
#ifdef WPILIB_AVAILABLE
    m_impl->pub_match_mode.Set(enabled);
#endif
}

SystemStatus XNav::GetStatus() const {
    SystemStatus s;
#ifdef WPILIB_AVAILABLE
    s.status      = m_impl->sub_status.Get("unknown");
    s.fps         = m_impl->sub_fps.Get(0.0);
    s.latency_ms  = m_impl->sub_latency.Get(0.0);
    s.num_targets = GetNumTargets();
    s.nt_connected = IsConnected();
#endif
    return s;
}

bool XNav::IsConnected() const {
#ifdef WPILIB_AVAILABLE
    return !m_impl->inst.GetConnections().empty();
#else
    return false;
#endif
}

void XNav::OnNewTargets(std::function<void(const std::vector<TagResult>&)> callback) {
    m_impl->on_new_targets = std::move(callback);
}

} // namespace xnav