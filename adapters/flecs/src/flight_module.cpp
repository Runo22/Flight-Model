#include "fm/ecs/flight_module.hpp"

#include <utility>

namespace fm::ecs {

namespace {

constexpr double kLeaderVelocityTau = 0.3;  // s, smoothing of estimated leader velocity

const WorldFrame& frame_or_default(const FrameConfig* config) {
    static const WorldFrame default_frame = WorldFrame::enu();
    return config ? config->frame : default_frame;
}

void write_pose(const WorldFrame& f, const FlightAgent& agent, FlightPose& pose) {
    const FlightState& s = agent.state();
    pose.position = f.to_world(s.position);
    pose.rotation = f.orientation(s.attitude);
    pose.velocity = f.vector_to_world(s.velocity);
    pose.heading = rad2deg(s.attitude.heading);
    pose.pitch = rad2deg(s.attitude.pitch);
    pose.roll = rad2deg(s.attitude.roll);
    pose.airspeed = s.airspeed;
    pose.ground_speed = s.ground_speed;
    pose.vertical_speed = s.vertical_speed;
    pose.throttle = s.throttle;
    pose.mode = agent.mode();
    pose.phase = s.phase;
    pose.on_ground = s.on_ground;
}

// Leader state in ENU from whatever the leader entity provides.
std::optional<LeaderState> leader_state(const WorldFrame& f, FollowLeader& follow, double dt) {
    const flecs::entity leader = follow.leader;
    if (!leader.is_alive()) return std::nullopt;
    if (const auto* fc = compat::try_get<FlightController>(leader); fc && fc->agent) {
        return fc->agent->as_leader();
    }
    const auto* pose = compat::try_get<TrackedPose>(leader);
    if (!pose) return std::nullopt;

    const Vec3 position = f.to_enu(pose->position);
    if (follow.has_last_position && dt > 0.0) {
        const Vec3 raw = (position - follow.last_position) / dt;
        const double k = 1.0 - std::exp(-dt / kLeaderVelocityTau);
        follow.velocity += (raw - follow.velocity) * k;
    }
    follow.last_position = position;
    follow.has_last_position = true;
    return LeaderState{position, follow.velocity, pose->heading, true};
}

}  // namespace

FlightModule::FlightModule(flecs::world& world) {
    world.module<FlightModule>();

    world.component<FrameConfig>();
    world.component<FlightController>();
    world.component<TerrainHeight>();
    world.component<HeightAboveGround>();
    world.component<Wind>();
    world.component<TrackedPose>();
    world.component<FlightPose>();
    world.component<FollowLeader>();

    if (!compat::try_get_singleton<FrameConfig>(world)) world.set<FrameConfig>({});

    // Reads other entities (the leaders), so it must stay single threaded.
    world.system<FollowLeader, FlightController>("FollowSystem")
        .kind(flecs::OnUpdate)
        .run([](flecs::iter& it) {
            const WorldFrame& f =
                frame_or_default(compat::try_get_singleton<FrameConfig>(it.world()));
            while (it.next()) {
                auto follow = it.field<FollowLeader>(0);
                auto controller = it.field<FlightController>(1);
                for (auto i : it) {
                    FlightController& fc = controller[i];
                    if (!fc.agent || fc.agent->mode() != FlightMode::Formation) continue;
                    if (auto leader = leader_state(f, follow[i], it.delta_time()))
                        fc.agent->set_leader_state(*leader);
                }
            }
        });

    // Every agent only touches its own components: safe to run on worker threads.
    world.system<FlightController, FlightPose, const TerrainHeight*, const HeightAboveGround*,
                 const Wind*>("FlightSystem")
        .kind(flecs::OnUpdate)
        .multi_threaded()
        .run([](flecs::iter& it) {
            const WorldFrame& f =
                frame_or_default(compat::try_get_singleton<FrameConfig>(it.world()));
            while (it.next()) {
                auto controller = it.field<FlightController>(0);
                auto pose = it.field<FlightPose>(1);
                // Optional inputs: nullptr when this table does not have the component.
                const TerrainHeight* terrain =
                    it.is_set(2) ? &it.field<const TerrainHeight>(2)[0] : nullptr;
                const HeightAboveGround* height =
                    it.is_set(3) ? &it.field<const HeightAboveGround>(3)[0] : nullptr;
                const Wind* wind = it.is_set(4) ? &it.field<const Wind>(4)[0] : nullptr;
                for (auto i : it) {
                    FlightController& fc = controller[i];
                    if (!fc.agent) continue;
                    Environment env;
                    if (terrain) {
                        env.terrain_elevation = f.height_to_enu(terrain[i].value);
                    } else if (height) {
                        env.terrain_elevation = fc.agent->state().position.z -
                                                height[i].value / f.units_per_meter;
                    }
                    if (wind) env.wind = f.vector_to_enu(wind[i].velocity);
                    fc.agent->step(it.delta_time(), env);
                    write_pose(f, *fc.agent, pose[i]);
                }
            }
        });
}

void set_frame(flecs::world& world, const WorldFrame& frame) { world.set<FrameConfig>({frame}); }

const WorldFrame& frame(const flecs::world& world) {
    return frame_or_default(compat::try_get_singleton<FrameConfig>(world));
}

void attach(flecs::entity e, const VehicleParams& params, const InitialConditions& ic) {
    FlightController fc;
    fc.agent.emplace(params, ic);
    FlightPose pose;
    write_pose(frame(e.world()), *fc.agent, pose);
    e.set<FlightController>(std::move(fc));
    e.set<FlightPose>(pose);
}

void attach_at(flecs::entity e, const VehicleParams& params, const Vec3& world_position,
               double heading, double speed, bool on_ground) {
    InitialConditions ic;
    ic.position = frame(e.world()).to_enu(world_position);
    ic.heading = heading;
    ic.speed = speed;
    ic.on_ground = on_ground;
    attach(e, params, ic);
}

double heading_from_world_forward(const flecs::world& world, const Vec3& world_forward) {
    return heading_of(frame(world).vector_to_enu(world_forward).xy());
}

FlightAgent* agent(flecs::entity e) {
    auto* fc = compat::try_get_mut<FlightController>(e);
    return fc && fc->agent ? &*fc->agent : nullptr;
}

bool follow(flecs::entity follower, flecs::entity leader, const FormationSlot& slot) {
    FlightAgent* a = agent(follower);
    if (!a || !leader.is_alive()) return false;
    a->join_formation(slot);
    if (a->mode() != FlightMode::Formation) return false;
    FollowLeader relation;
    relation.leader = leader;
    relation.slot = slot;
    if (const auto* fc = compat::try_get<FlightController>(leader); fc && fc->agent)
        a->set_leader_state(fc->agent->as_leader());
    follower.set<FollowLeader>(relation);
    return true;
}

void stop_following(flecs::entity follower) {
    follower.remove<FollowLeader>();
    if (FlightAgent* a = agent(follower); a && a->mode() == FlightMode::Formation) a->cruise();
}

}  // namespace fm::ecs
