#include "fm/ecs/flight_module.hpp"

#include <algorithm>
#include <cstdio>
#include <utility>

namespace fm::ecs {

namespace {

constexpr double kLeaderVelocityTau = 0.3;  // s, smoothing of estimated leader velocity

const FrameConfig& config_or_default(const FrameConfig* config) {
    static const FrameConfig default_config{};
    return config ? *config : default_config;
}

const FrameConfig& config_of(const flecs::world& world) {
    return config_or_default(compat::try_get_singleton<FrameConfig>(world));
}

FlightSettings settings_of(const flecs::world& world) {
    const FlightSettings* s = compat::try_get_singleton<FlightSettings>(world);
    return s ? *s : FlightSettings{};
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

void write_geo_pose(const GeoProjection& proj, const FlightAgent& agent, GeoPose& pose) {
    const FlightState& s = agent.state();
    const GeoPoint p = proj.to_geo(s.position);
    pose.latitude = p.latitude;
    pose.longitude = p.longitude;
    pose.altitude = p.altitude;
    pose.roll = rad2deg(s.attitude.roll);
    pose.pitch = rad2deg(s.attitude.pitch);
    const double yaw = rad2deg(wrap_2pi(s.attitude.heading - proj.convergence(p)));
    pose.yaw = yaw >= 360.0 - 1e-9 ? 0.0 : yaw;  // keep [0, 360) free of rounding to 360
}

// Smoothed velocity of a leader that only reports positions.
Vec3 estimate_velocity(FollowLeader& follow, const Vec3& position, double dt) {
    if (follow.has_last_position && dt > 0.0) {
        const Vec3 raw = (position - follow.last_position) / dt;
        const double k = 1.0 - std::exp(-dt / kLeaderVelocityTau);
        follow.velocity += (raw - follow.velocity) * k;
    }
    follow.last_position = position;
    follow.has_last_position = true;
    return follow.velocity;
}

// Leader state in the local frame from whatever the leader entity provides.
std::optional<LeaderState> leader_state(const FrameConfig& cfg, FollowLeader& follow, double dt) {
    const flecs::entity leader = follow.leader;
    if (!leader.is_alive()) return std::nullopt;
    if (const auto* fc = compat::try_get<FlightController>(leader); fc && fc->agent) {
        LeaderState state = fc->agent->as_leader();
        // A paused leader stands still; followers must not chase its last velocity.
        if (leader.has<Paused>() || settings_of(leader.world()).paused) state.velocity = {};
        return state;
    }
    if (const auto* geo_pose = compat::try_get<TrackedGeoPose>(leader); geo_pose && cfg.geo) {
        const Vec3 position = cfg.geo->to_local(geo_pose->position);
        const double heading = std::isnan(geo_pose->yaw)
                                   ? kUnknown
                                   : cfg.geo->grid_heading(deg2rad(geo_pose->yaw), position);
        return LeaderState{position, estimate_velocity(follow, position, dt), heading, true};
    }
    if (const auto* pose = compat::try_get<TrackedPose>(leader)) {
        const Vec3 position = cfg.frame.to_enu(pose->position);
        return LeaderState{position, estimate_velocity(follow, position, dt), pose->heading, true};
    }
    return std::nullopt;
}

// Returns the world's projection, creating it at `origin` when missing. Returned by value
// so it also works in deferred mode, where the new singleton is not visible yet.
GeoProjection ensure_geo(flecs::world& world, const GeoPoint& origin) {
    if (const GeoProjection* proj = geo(world)) return *proj;
    set_geo_origin(world, origin.latitude, origin.longitude);
    return GeoProjection(origin.latitude, origin.longitude);
}

// Adds the agent and its outputs without reading the components back (deferred-safe).
void attach_agent(flecs::entity e, FlightAgent agent, const std::optional<GeoProjection>& proj,
                  const AgentSetup& setup) {
    if (setup) setup(agent);
    FlightPose pose;
    write_pose(frame(e.world()), agent, pose);
    GeoPose geo_pose;
    if (proj) write_geo_pose(*proj, agent, geo_pose);
    FlightController fc;
    fc.agent.emplace(std::move(agent));
    e.set<FlightController>(std::move(fc));
    e.set<FlightPose>(pose);
    if (proj) e.set<GeoPose>(geo_pose);
}

template <typename... T>
void reset_cached_ids() {
    (compat::reset_cached_id<T>(), ...);
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
    world.component<TrackedGeoPose>();
    world.component<FlightPose>();
    world.component<GeoPose>();
    world.component<FollowLeader>();
    world.component<FlightSettings>();
    world.component<Paused>();

    if (!compat::try_get_singleton<FrameConfig>(world)) world.set<FrameConfig>({});
    if (!compat::try_get_singleton<FlightSettings>(world)) world.set<FlightSettings>({});

    // Reads other entities (the leaders), so it must stay single threaded.
    world.system<FollowLeader, FlightController>("FollowSystem")
        .kind(flecs::OnUpdate)
        .without<Paused>()
        .run([](flecs::iter& it) {
            const flecs::world stage = it.world();
            const FlightSettings settings = settings_of(stage);
            if (settings.paused) {
                it.fini();
                return;
            }
            const FrameConfig& cfg = config_of(stage);
            const double dt = std::min<double>(it.delta_time(), settings.max_step);
            while (it.next()) {
                auto follow = it.field<FollowLeader>(0);
                auto controller = it.field<FlightController>(1);
                for (auto i : it) {
                    FlightController& fc = controller[i];
                    if (!fc.agent || fc.agent->mode() != FlightMode::Formation) continue;
                    if (auto leader = leader_state(cfg, follow[i], dt))
                        fc.agent->set_leader_state(*leader);
                }
            }
        });

    // Every agent only touches its own components: safe to run on worker threads.
    world.system<FlightController, FlightPose, const TerrainHeight*, const HeightAboveGround*,
                 const Wind*, GeoPose*>("FlightSystem")
        .kind(flecs::OnUpdate)
        .without<Paused>()
        .multi_threaded()
        .run([](flecs::iter& it) {
            const flecs::world stage = it.world();
            const FlightSettings settings = settings_of(stage);
            if (settings.paused) {
                it.fini();
                return;
            }
            const double dt = std::min<double>(it.delta_time(), settings.max_step);
            const FrameConfig& cfg = config_of(stage);
            const WorldFrame& f = cfg.frame;
            while (it.next()) {
                auto controller = it.field<FlightController>(0);
                auto pose = it.field<FlightPose>(1);
                // Optional terms: nullptr when this table does not have the component.
                const TerrainHeight* terrain =
                    it.is_set(2) ? &it.field<const TerrainHeight>(2)[0] : nullptr;
                const HeightAboveGround* height =
                    it.is_set(3) ? &it.field<const HeightAboveGround>(3)[0] : nullptr;
                const Wind* wind = it.is_set(4) ? &it.field<const Wind>(4)[0] : nullptr;
                GeoPose* geo_pose = it.is_set(5) && cfg.geo ? &it.field<GeoPose>(5)[0] : nullptr;
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
                    fc.agent->step(dt, env);
                    write_pose(f, *fc.agent, pose[i]);
                    if (geo_pose) write_geo_pose(*cfg.geo, *fc.agent, geo_pose[i]);
                }
            }
        });
}

flecs::entity load(flecs::world& world) {
    if (!compat::runtime_initialized()) {
        std::fprintf(stderr,
                     "fm::ecs::load: this binary has its own, uninitialized copy of Flecs. "
                     "Link it against the Flecs library the host uses (e.g. flecs.dll) "
                     "instead of a static Flecs; see README, \"Plugin DLLs\".\n");
        return flecs::entity();
    }
    const compat::RuntimeVersion v = compat::runtime_version();
    if (!v.matches) {
        std::fprintf(stderr,
                     "fm::ecs::load: compiled against Flecs %d.%d.%d but the runtime is "
                     "%d.%d.%d; rebuild against the host's Flecs headers and library.\n",
                     FLECS_VERSION_MAJOR, FLECS_VERSION_MINOR, FLECS_VERSION_PATCH, v.major,
                     v.minor, v.patch);
        return flecs::entity();
    }
    return world.import<FlightModule>();
}

void unload(flecs::world& world) {
    // Deleting the module entity deletes its children: the systems, everything created in
    // its scope and the components, which flecs removes from all entities first.
    if (flecs::entity module = module_entity(world)) module.destruct();
    // This binary may stay loaded and call load() again: forget the cached ids.
    reset_cached_ids<FlightModule, FrameConfig, FlightController, TerrainHeight,
                     HeightAboveGround, Wind, TrackedPose, TrackedGeoPose, FlightPose, GeoPose,
                     FollowLeader, FlightSettings, Paused>();
}

flecs::entity module_entity(const flecs::world& world) {
    return world.lookup("fm::ecs::FlightModule");
}

void set_paused(flecs::world& world, bool paused) {
    FlightSettings s = settings_of(world);
    s.paused = paused;
    world.set<FlightSettings>(s);
}

bool is_paused(const flecs::world& world) { return settings_of(world).paused; }

void pause(flecs::entity e) { e.add<Paused>(); }

void resume(flecs::entity e) { e.remove<Paused>(); }

bool is_paused(flecs::entity e) { return e.has<Paused>(); }

void set_max_step(flecs::world& world, double seconds) {
    FlightSettings s = settings_of(world);
    if (seconds > 0.0) s.max_step = seconds;
    world.set<FlightSettings>(s);
}

void set_frame(flecs::world& world, const WorldFrame& frame) {
    world.set<FrameConfig>({frame, std::nullopt});
}

const WorldFrame& frame(const flecs::world& world) { return config_of(world).frame; }

void attach(flecs::entity e, const VehicleParams& params, const InitialConditions& ic,
            const AgentSetup& setup) {
    const GeoProjection* proj = geo(e.world());
    attach_agent(e, FlightAgent(params, ic),
                 proj ? std::optional<GeoProjection>(*proj) : std::nullopt, setup);
}

void attach_at(flecs::entity e, const VehicleParams& params, const Vec3& world_position,
               double heading, double speed, bool on_ground, const AgentSetup& setup) {
    InitialConditions ic;
    ic.position = frame(e.world()).to_enu(world_position);
    ic.heading = heading;
    ic.speed = speed;
    ic.on_ground = on_ground;
    attach(e, params, ic, setup);
}

double heading_from_world_forward(const flecs::world& world, const Vec3& world_forward) {
    return heading_of(frame(world).vector_to_enu(world_forward).xy());
}

void set_geo_origin(flecs::world& world, double latitude, double longitude) {
    world.set<FrameConfig>({WorldFrame::enu(), GeoProjection(latitude, longitude)});
}

const GeoProjection* geo(const flecs::world& world) {
    const FrameConfig* cfg = compat::try_get_singleton<FrameConfig>(world);
    return cfg && cfg->geo ? &*cfg->geo : nullptr;
}

Vec3 to_local(flecs::world& world, const GeoPoint& p) { return ensure_geo(world, p).to_local(p); }

double local_heading(flecs::world& world, double true_heading_deg, const GeoPoint& near) {
    const GeoProjection proj = ensure_geo(world, near);
    return proj.grid_heading(deg2rad(true_heading_deg), proj.to_local(near));
}

void attach_geo(flecs::entity e, const VehicleParams& params, const GeoPoint& at, double yaw,
                double speed, bool on_ground, const AgentSetup& setup) {
    flecs::world world = e.world();
    const GeoProjection proj = ensure_geo(world, at);
    InitialConditions ic;
    ic.position = proj.to_local(at);
    ic.heading = proj.grid_heading(deg2rad(yaw), ic.position);
    ic.speed = speed;
    ic.on_ground = on_ground;
    attach_agent(e, FlightAgent(params, ic), proj, setup);
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
