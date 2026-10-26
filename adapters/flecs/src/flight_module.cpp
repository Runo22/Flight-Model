#include "fm/ecs/flight_module.hpp"

#include <algorithm>
#include <cstdio>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

namespace fm::ecs {

namespace {

constexpr double kLeaderVelocityTau = 0.3;  // s, smoothing of estimated leader velocity
constexpr double kMaxLeaderSpeed = 1500.0;  // m/s, faster position changes are jumps

const FrameConfig& config_or_default(const FrameConfig* config) {
    static const FrameConfig default_config{};
    return config ? *config : default_config;
}

const FrameConfig& config_of(const flecs::world& world) {
    return config_or_default(world.try_get<FrameConfig>());
}

FlightSettings settings_of(const flecs::world& world) {
    const FlightSettings* s = world.try_get<FlightSettings>();
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
    pose.calibrated_airspeed = s.calibrated_airspeed;
    pose.mach = s.mach;
    pose.ground_speed = s.ground_speed;
    pose.vertical_speed = s.vertical_speed;
    pose.throttle = s.throttle;
    pose.g_force = s.g_force;
    pose.angle_of_attack = rad2deg(s.angle_of_attack);
    pose.sideslip_angle = rad2deg(s.sideslip_angle);
    pose.turn_rate = rad2deg(s.turn_rate);
    pose.fuel = s.fuel;
    pose.gear_position = s.gear_position;
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

// Writes the agent's pose to the output components and remembers what was written.
void publish(const WorldFrame& f, const GeoProjection* proj, FlightController& fc,
             FlightPose& pose, GeoPose* geo_pose) {
    write_pose(f, *fc.agent, pose);
    fc.written.position = pose.position;
    fc.written.rotation = pose.rotation;
    fc.written.heading = pose.heading;
    if (proj && geo_pose) {
        write_geo_pose(*proj, *fc.agent, *geo_pose);
        fc.written.geo = *geo_pose;
        fc.written.geo_valid = true;
    }
}

bool valid_geo(const GeoPoint& p) {
    return std::abs(p.latitude) <= 90.0 && std::isfinite(p.longitude) &&
           std::isfinite(p.altitude);
}

// Within a meter of each other on the Earth's surface.
bool same_place(const GeoPoint& a, const GeoPoint& b) {
    const auto unit = [](const GeoPoint& p) {
        const double lat = deg2rad(p.latitude);
        const double lon = deg2rad(p.longitude);
        return Vec3{std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat)};
    };
    return (unit(a) - unit(b)).length() * 6.371e6 < 1.0;
}

bool relocate_geo(const GeoProjection& proj, FlightAgent& agent, const GeoPose& g) {
    const GeoPoint at{g.latitude, g.longitude, g.altitude};
    if (!valid_geo(at)) return false;
    const Vec3 local = proj.to_local(at);
    // Points the projection cannot represent (towards the far side of the Earth).
    if (!same_place(proj.to_geo(local), at)) return false;
    const double heading =
        std::isfinite(g.yaw) ? proj.grid_heading(deg2rad(g.yaw), local) : kUnknown;
    return agent.relocate(local, heading);
}

bool same_rotation(const Quat& a, const Quat& b) {
    const double d = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
    return std::abs(d) > 1.0 - 1e-9;  // within ~0.01 deg
}

// A FlightPose / GeoPose that differs from what was written last means the game moved the
// entity: the agent continues from there. Absurd poses are refused by relocate() and get
// overwritten with the agent's pose after the step.
void take_over_moves(const WorldFrame& f, const GeoProjection* proj, FlightController& fc,
                     const FlightPose& pose, const GeoPose* geo_pose) {
    const FlightController::Written& w = fc.written;
    if (proj && geo_pose && w.geo_valid && detail::geo_moved(*geo_pose, w.geo)) {
        relocate_geo(*proj, *fc.agent, *geo_pose);
        return;
    }
    const bool moved = (pose.position - w.position).length() > 0.01 * f.units_per_meter;
    const bool turned = std::abs(std::remainder(pose.heading - w.heading, 360.0)) > 0.01;
    const bool rotated = !same_rotation(pose.rotation, w.rotation);
    if (!moved && !turned && !rotated) return;
    double heading = kUnknown;
    if (turned) {
        heading = deg2rad(pose.heading);
    } else if (rotated) {
        const Vec3 forward = f.vector_to_enu(pose.rotation.rotate(f.model_forward));
        if (forward.xy().length() > 1e-6 * forward.length()) heading = heading_of(forward.xy());
    }
    fc.agent->relocate(f.to_enu(pose.position), heading);
}

// Smoothed velocity of a leader that only reports positions.
Vec3 estimate_velocity(FollowLeader& follow, const Vec3& position, double dt) {
    if (follow.has_last_position && dt > 0.0) {
        const Vec3 raw = (position - follow.last_position) / dt;
        // Faster than anything flies: the game moved the leader. Keep the estimate.
        if (raw.length() <= kMaxLeaderSpeed) {
            const double k = 1.0 - std::exp(-dt / kLeaderVelocityTau);
            follow.velocity += (raw - follow.velocity) * k;
        }
    }
    follow.last_position = position;
    follow.has_last_position = true;
    return follow.velocity;
}

// Leader state in the local frame from whatever the leader entity provides.
std::optional<LeaderState> leader_state(const FrameConfig& cfg, FollowLeader& follow, double dt) {
    const flecs::entity leader = follow.leader;
    if (!leader.is_alive()) return std::nullopt;
    if (const auto* fc = leader.try_get<FlightController>(); fc && fc->agent) {
        LeaderState state = fc->agent->as_leader();
        // A paused leader stands still; followers must not chase its last velocity.
        if (leader.has<Paused>() || settings_of(leader.world()).paused) state.velocity = {};
        return state;
    }
    if (const auto* geo_pose = leader.try_get<TrackedGeoPose>(); geo_pose && cfg.geo) {
        const Vec3 position = cfg.geo->to_local(geo_pose->position);
        const double heading = std::isnan(geo_pose->yaw)
                                   ? kUnknown
                                   : cfg.geo->grid_heading(deg2rad(geo_pose->yaw), position);
        return LeaderState{position, estimate_velocity(follow, position, dt), heading, true};
    }
    if (const auto* pose = leader.try_get<TrackedPose>()) {
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
    FlightController fc;
    fc.agent.emplace(std::move(agent));
    FlightPose pose;
    GeoPose geo_pose;
    publish(frame(e.world()), proj ? &*proj : nullptr, fc, pose, &geo_pose);
    e.set<FlightController>(std::move(fc));
    e.set<FlightPose>(pose);
    if (proj) e.set<GeoPose>(geo_pose);
}

// Where `slot` of a leader at `position` heading `heading` is.
Vec3 slot_position(const Vec3& position, double heading, const FormationSlot& slot) {
    const Vec2 forward = heading_vector(heading);
    const Vec3& o = slot.offset;
    const Vec2 at = position.xy() + right_of(forward) * o.x + forward * o.y;
    return {at.x, at.y, position.z + o.z};
}

FixedWingAgent* fixed_wing_of(flecs::entity e) {
    FlightAgent* a = e.is_alive() ? agent(e) : nullptr;
    return a ? a->fixed_wing() : nullptr;
}

// Flight takeoff: start the taxi (one after the other along a shared path), release the rows
// once everybody is lined up, rejoin the leader once airborne.
void step_takeoff(flecs::entity leader, FlightSequence& seq) {
    const FlightTakeoff& o = seq.takeoff;
    const std::vector<Vec3>& lead_taxi = seq.members.front().plan.taxi;
    const Vec2 entry = lead_taxi.empty() ? Vec2{} : lead_taxi.front().xy();
    bool all_ready = true;
    const FlightSequence::Member* ahead = nullptr;
    for (std::size_t m = 0; m < seq.members.size(); ++m) {
        FlightSequence::Member& mem = seq.members[m];
        if (mem.done) continue;
        FixedWingAgent* fw = fixed_wing_of(mem.entity);
        if (!fw) {
            mem.done = true;  // deleted, or no fixed-wing agent any more
            continue;
        }
        const FlightState& s = fw->state();
        const double radius = std::max(fw->params().taxi_turn_radius, 10.0);
        if (!mem.started) {
            // Along the leader's path: enter it once the aircraft ahead has moved on.
            bool go = true;
            if (mem.shared_taxi && ahead) {
                const FixedWingAgent* a = fixed_wing_of(ahead->entity);
                go = !a || (ahead->started && ahead->passed_entry &&
                            (a->state().position.xy() - entry).length() > 4.0 * radius);
            }
            if (go) {
                mem.started = agent(mem.entity)->takeoff(mem.plan);
                mem.done = !mem.started;
            }
            all_ready = false;
            ahead = &mem;
            continue;
        }
        ahead = &mem;
        if ((s.position.xy() - entry).length() < 2.0 * radius) mem.passed_entry = true;
        if (!mem.released) {
            if (fw->mode() != FlightMode::Takeoff) {
                mem.done = true;  // given another command
                continue;
            }
            all_ready = all_ready && s.phase == FlightPhase::LineUp && s.ground_speed < 0.3;
            if (seq.release_time >= 0.0 && seq.clock >= seq.release_time + mem.row * o.interval) {
                fw->release_takeoff();
                mem.released = true;
                if (m == 0) mem.done = true;  // the leader flies its plan
            }
            continue;
        }
        // Airborne after the takeoff: join the leader.
        if (m > 0 && fw->mode() != FlightMode::Takeoff && !s.on_ground) {
            follow(mem.entity, leader, mem.slot);
            mem.done = true;
        }
    }
    if (seq.release_time < 0.0 && all_ready) seq.release_time = seq.clock;
}

// Landing parameters of the member at position `m` of the landing order: alternating sides
// of the runway (the leader on the break side), later breaks, shorter stops.
void assign_landing(const FlightSequence& seq, std::size_t m, FlightSequence::Member& mem) {
    const FlightLanding& o = seq.land;
    const double side = seq.left_pattern ? -1.0 : 1.0;
    const Vec2 right = right_of(heading_vector(seq.runway.heading));
    const double shift = (m % 2 == 0 ? side : -side) * o.side_offset;
    const Vec2 at = seq.runway.start.xy() + right * shift;
    mem.runway = seq.runway;
    mem.runway.start.x = at.x;
    mem.runway.start.y = at.y;
    mem.landing.pattern = LandingPattern::Overhead;
    mem.landing.left_pattern = seq.left_pattern;
    mem.landing.break_distance = o.break_distance + static_cast<double>(m) * seq.break_spacing;
    mem.landing.initial_speed = seq.initial_speed;
    // Initial leg: the leader on the centerline, wingmen in echelon next to it.
    mem.landing.initial_offset = (m == 0 ? 0.0 : mem.slot.offset.x) - shift;
    mem.landing.stop_distance =
        seq.natural_stop + static_cast<double>(seq.members.size() - 1 - m) * o.stop_spacing;
}

// Flight landing. Once the leader flies along the runway on the initial leg, the wingmen in
// their echelon slots leave the formation for their own overhead approach at once; the
// others are moved to the end of the landing order and start when the aircraft before them
// has broken.
void step_landing(FlightSequence& seq) {
    using Stage = FixedWingAgent::LandingStage;
    const auto past_break = [](const FixedWingAgent* fw) {
        if (!fw || fw->mode() != FlightMode::Land) return true;
        const Stage stage = fw->landing_stage();
        return stage != Stage::Positioning && stage != Stage::Initial;
    };
    const flecs::entity leader = seq.members.front().entity;
    const FixedWingAgent* lead_fw = fixed_wing_of(leader);
    // Distance from a slot: horizontal, and height difference.
    const auto slot_error = [&](const FlightSequence::Member& mem, const FormationSlot& slot) {
        const FixedWingAgent* fw = fixed_wing_of(mem.entity);
        if (!fw || !lead_fw) return Vec2{1e9, 1e9};
        const FlightState& l = lead_fw->state();
        const Vec3 d = fw->state().position - slot_position(l.position, l.attitude.heading, slot);
        return Vec2{d.xy().length(), std::abs(d.z)};
    };
    if (!seq.ordered) {
        // Wingmen dropping back behind the echelon move on to their slot.
        for (FlightSequence::Member& mem : seq.members) {
            if (mem.in_transit && slot_error(mem, mem.transit).x < seq.land.echelon_spacing) {
                follow(mem.entity, leader, mem.slot);
                mem.in_transit = false;
            }
        }
    }
    if (!seq.ordered) {
        // The leader established on the initial: on the centerline, along the runway. The
        // order is fixed once every wingman of the formation is in its echelon slot, at the
        // latest ~15 s before the leader's break.
        bool on_initial = !lead_fw || lead_fw->mode() != FlightMode::Land;
        bool last_call = on_initial;
        if (!on_initial) {
            const Stage stage = lead_fw->landing_stage();
            const FlightState& l = lead_fw->state();
            const Vec2 dir = heading_vector(seq.runway.heading);
            const Vec2 rel = l.position.xy() - seq.runway.start.xy();
            on_initial = stage != Stage::Positioning &&
                         (stage != Stage::Initial ||
                          (std::abs(wrap_pi(l.track - seq.runway.heading)) < deg2rad(10.0) &&
                           std::abs(dot(rel, right_of(dir))) < 150.0));
            const double decision = std::max(2000.0, 15.0 * seq.initial_speed);
            last_call = stage != Stage::Positioning &&
                        (stage != Stage::Initial ||
                         dot(rel, dir) > seq.members.front().landing.break_distance - decision);
        }
        if (!on_initial) return;
        // Wingmen close to their echelon slot keep their place in the order.
        const auto in_position = [&](const FlightSequence::Member& mem) {
            const FixedWingAgent* fw = fixed_wing_of(mem.entity);
            const FollowLeader* f = mem.entity.try_get<FollowLeader>();
            if (!fw || !f || f->leader != leader || fw->mode() != FlightMode::Formation ||
                mem.in_transit)
                return false;
            const Vec2 error = slot_error(mem, mem.slot);
            return error.x < std::max(100.0, 1.5 * seq.land.echelon_spacing) && error.y < 200.0;
        };
        const auto in_formation = [&](const FlightSequence::Member& mem) {
            const FixedWingAgent* fw = fixed_wing_of(mem.entity);
            const FollowLeader* f = mem.entity.try_get<FollowLeader>();
            return fw && f && f->leader == leader && fw->mode() == FlightMode::Formation;
        };
        const bool all_in_position =
            std::all_of(seq.members.begin() + 1, seq.members.end(), [&](const auto& mem) {
                return mem.started || !in_formation(mem) || in_position(mem);
            });
        if (!all_in_position && !last_call) return;
        std::stable_partition(seq.members.begin() + 1, seq.members.end(),
                              [&](const FlightSequence::Member& mem) {
                                  return mem.started || in_position(mem);
                              });
        for (std::size_t m = 1; m < seq.members.size(); ++m) {
            FlightSequence::Member& mem = seq.members[m];
            if (mem.started) continue;
            assign_landing(seq, m, mem);
            if (in_position(mem)) {
                stop_following(mem.entity);
                mem.started = agent(mem.entity)->land(mem.runway, mem.landing);
                mem.done = !mem.started;
            }
        }
        seq.ordered = true;
    }
    for (std::size_t m = 0; m < seq.members.size(); ++m) {
        FlightSequence::Member& mem = seq.members[m];
        if (mem.done) continue;
        FixedWingAgent* fw = fixed_wing_of(mem.entity);
        if (!fw) {
            mem.done = true;
            continue;
        }
        if (!mem.started) {
            // Late: start once the aircraft before it has broken.
            const FlightSequence::Member& before = seq.members[m - 1];
            if (!before.done && (!before.started || !past_break(fixed_wing_of(before.entity))))
                continue;
            stop_following(mem.entity);
            mem.started = agent(mem.entity)->land(mem.runway, mem.landing);
            mem.done = !mem.started;
            continue;
        }
        mem.done = fw->mode() != FlightMode::Land || fw->state().phase == FlightPhase::Landed;
    }
}

template <typename... T>
void reset_cached_ids() {
    (flecs::_::type<T>::reset(), ...);
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
    world.component<TransformLink>();
    world.component<FlightSettings>();
    world.component<Paused>();
    world.component<FlightSequence>();

    if (!world.try_get<FrameConfig>()) world.set<FrameConfig>({});
    if (!world.try_get<FlightSettings>()) world.set<FlightSettings>({});

    // Commands other entities (the flight members), so it must stay single threaded.
    world.system<FlightSequence>("FlightSequenceSystem")
        .kind(flecs::OnUpdate)
        .without<Paused>()
        .run([](flecs::iter& it) {
            const FlightSettings settings = settings_of(it.world());
            if (settings.paused) {
                it.fini();
                return;
            }
            const double dt = std::min<double>(it.delta_time(), settings.max_step);
            while (it.next()) {
                auto sequence = it.field<FlightSequence>(0);
                for (auto i : it) {
                    FlightSequence& seq = sequence[i];
                    seq.clock += dt;
                    if (seq.landing) {
                        step_landing(seq);
                    } else {
                        step_takeoff(it.entity(i), seq);
                    }
                    const bool finished =
                        std::all_of(seq.members.begin(), seq.members.end(),
                                    [](const FlightSequence::Member& m) { return m.done; });
                    if (finished) it.entity(i).remove<FlightSequence>();
                }
            }
        });

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
            // Widest follower offset per formation leader, and whether one is rejoining.
            struct Lead {
                flecs::entity leader;
                double extent = 0.0;
                double rejoin_distance = 0.0;      // m, furthest follower from its slot
                double follower_speed = kUnknown;  // slowest follower's limit, m/s
            };
            std::vector<Lead> leads;
            while (it.next()) {
                auto follow = it.field<FollowLeader>(0);
                auto controller = it.field<FlightController>(1);
                for (auto i : it) {
                    FlightController& fc = controller[i];
                    if (!follow[i].leader.is_alive()) {
                        // The leader was deleted: end the relation, fly on alone.
                        it.entity(i).remove<FollowLeader>();
                        if (fc.agent && fc.agent->mode() == FlightMode::Formation)
                            fc.agent->cruise();
                        continue;
                    }
                    if (!fc.agent || !fc.agent->tracks_leader()) continue;
                    const std::optional<LeaderState> leader = leader_state(cfg, follow[i], dt);
                    if (leader) fc.agent->set_leader_state(*leader);
                    if (fc.agent->mode() != FlightMode::Formation) continue;
                    const double extent = std::abs(follow[i].slot.offset.x);
                    double distance = 0.0;  // from the slot
                    if (leader) {
                        const double heading = std::isfinite(leader->heading)
                                                   ? leader->heading
                                                   : heading_of(leader->velocity.xy());
                        const Vec3 at = slot_position(leader->position, heading, follow[i].slot);
                        distance = (fc.agent->state().position - at).length();
                    }
                    const double speed = fc.agent->formation_speed_limit();
                    auto lead = std::find_if(leads.begin(), leads.end(), [&](const Lead& l) {
                        return l.leader == follow[i].leader;
                    });
                    if (lead == leads.end()) {
                        leads.push_back({follow[i].leader, extent, distance, speed});
                    } else {
                        lead->extent = std::max(lead->extent, extent);
                        lead->rejoin_distance = std::max(lead->rejoin_distance, distance);
                        if (!(lead->follower_speed <= speed)) lead->follower_speed = speed;
                    }
                }
            }
            // Leaders bank only as far as their followers can follow, stay below the
            // slowest one's speed and slow down for the ones still rejoining.
            for (const Lead& l : leads) {
                if (auto* fc = l.leader.try_get_mut<FlightController>(); fc && fc->agent)
                    fc->agent->set_formation_lead(l.extent, l.rejoin_distance, l.follower_speed);
            }
        });

    // Collision avoidance: every airborne agent gets the agents around it as traffic, except
    // its formation partners. Reads other entities, so it stays single threaded.
    world.system<FlightController>("TrafficSystem")
        .kind(flecs::OnUpdate)
        .without<Paused>()
        .run([](flecs::iter& it) {
            const FlightSettings settings = settings_of(it.world());
            if (settings.paused || !settings.collision_avoidance) {
                it.fini();
                return;
            }
            struct Entry {
                flecs::entity entity;
                FlightAgent* agent;
                Traffic traffic;
                flecs::entity leader;  // formation leader, if flying in one
            };
            std::vector<Entry> entries;
            while (it.next()) {
                auto controller = it.field<FlightController>(0);
                for (auto i : it) {
                    FlightController& fc = controller[i];
                    if (!fc.agent || fc.agent->kind() == VehicleKind::Ship) continue;
                    const FlightState& s = fc.agent->state();
                    if (s.on_ground) continue;
                    const flecs::entity e = it.entity(i);
                    flecs::entity leader;
                    if (const auto* f = e.try_get<FollowLeader>();
                        f && fc.agent->mode() == FlightMode::Formation)
                        leader = f->leader;
                    entries.push_back({e, &*fc.agent, {s.position, s.velocity}, leader});
                }
            }
            // Neighbours from a uniform grid over the ground plane.
            constexpr double kCell = 3000.0;    // m, larger than any conflict radius
            constexpr double kMaxHeight = 1500.0;
            auto cell = [&](double v) { return static_cast<std::int64_t>(std::floor(v / kCell)); };
            auto key = [](std::int64_t x, std::int64_t y) {
                return (static_cast<std::uint64_t>(x) << 32) ^
                       (static_cast<std::uint64_t>(y) & 0xffffffffu);
            };
            std::unordered_map<std::uint64_t, std::vector<std::size_t>> grid;
            for (std::size_t i = 0; i < entries.size(); ++i) {
                const Vec3& p = entries[i].traffic.position;
                grid[key(cell(p.x), cell(p.y))].push_back(i);
            }
            std::vector<Traffic> near;
            for (std::size_t i = 0; i < entries.size(); ++i) {
                const Entry& a = entries[i];
                near.clear();
                const std::int64_t cx = cell(a.traffic.position.x);
                const std::int64_t cy = cell(a.traffic.position.y);
                for (std::int64_t dx = -1; dx <= 1; ++dx) {
                    for (std::int64_t dy = -1; dy <= 1; ++dy) {
                        const auto found = grid.find(key(cx + dx, cy + dy));
                        if (found == grid.end()) continue;
                        for (const std::size_t j : found->second) {
                            const Entry& b = entries[j];
                            const bool partners = a.leader == b.entity || b.leader == a.entity ||
                                                  (a.leader && a.leader == b.leader);
                            if (j == i || partners ||
                                std::abs(b.traffic.position.z - a.traffic.position.z) > kMaxHeight)
                                continue;
                            near.push_back(b.traffic);
                        }
                    }
                }
                a.agent->set_traffic(near);
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
            const GeoProjection* proj = cfg.geo ? &*cfg.geo : nullptr;
            while (it.next()) {
                auto controller = it.field<FlightController>(0);
                auto pose = it.field<FlightPose>(1);
                // Optional terms: nullptr when this table does not have the component.
                const TerrainHeight* terrain =
                    it.is_set(2) ? &it.field<const TerrainHeight>(2)[0] : nullptr;
                const HeightAboveGround* height =
                    it.is_set(3) ? &it.field<const HeightAboveGround>(3)[0] : nullptr;
                const Wind* wind = it.is_set(4) ? &it.field<const Wind>(4)[0] : nullptr;
                GeoPose* geo_pose = it.is_set(5) && proj ? &it.field<GeoPose>(5)[0] : nullptr;
                for (auto i : it) {
                    FlightController& fc = controller[i];
                    if (!fc.agent) continue;
                    GeoPose* geo = geo_pose ? &geo_pose[i] : nullptr;
                    take_over_moves(f, proj, fc, pose[i], geo);
                    Environment env;
                    if (terrain) {
                        env.terrain_elevation = f.height_to_enu(terrain[i].value);
                    } else if (height) {
                        env.terrain_elevation = fc.agent->state().position.z -
                                                height[i].value / f.units_per_meter;
                    }
                    if (wind) env.wind = f.vector_to_enu(wind[i].velocity);
                    fc.agent->step(dt, env);
                    publish(f, proj, fc, pose[i], geo);
                }
            }
        });
}

flecs::entity load(flecs::world& world) { return world.import<FlightModule>(); }

void unload(flecs::world& world) {
    // Deleting the module entity deletes its children: the systems, everything created in
    // its scope and the components, which flecs removes from all entities first.
    if (flecs::entity module = module_entity(world)) module.destruct();
    // This binary may stay loaded and call load() again: forget the cached ids.
    reset_cached_ids<FlightModule, FrameConfig, FlightController, TerrainHeight,
                     HeightAboveGround, Wind, TrackedPose, TrackedGeoPose, FlightPose, GeoPose,
                     FollowLeader, TransformLink, FlightSettings, Paused, FlightSequence>();
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

void set_collision_avoidance(flecs::world& world, bool enabled) {
    FlightSettings s = settings_of(world);
    s.collision_avoidance = enabled;
    world.set<FlightSettings>(s);
}

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
    if (!is_finite(ic.position) || !std::isfinite(ic.heading)) {
        std::fprintf(stderr, "fm::ecs::attach: non-finite position or heading, not attached\n");
        return;
    }
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
    if (!valid_geo({latitude, longitude, 0.0})) {
        std::fprintf(stderr, "fm::ecs::set_geo_origin: invalid origin %g, %g ignored\n",
                     latitude, longitude);
        return;
    }
    world.set<FrameConfig>({WorldFrame::enu(), GeoProjection(latitude, longitude)});
}

const GeoProjection* geo(const flecs::world& world) {
    const FrameConfig* cfg = world.try_get<FrameConfig>();
    return cfg && cfg->geo ? &*cfg->geo : nullptr;
}

Vec3 to_local(flecs::world& world, const GeoPoint& p) { return ensure_geo(world, p).to_local(p); }

double local_heading(flecs::world& world, double true_heading_deg, const GeoPoint& near) {
    const GeoProjection proj = ensure_geo(world, near);
    return proj.grid_heading(deg2rad(true_heading_deg), proj.to_local(near));
}

void attach_geo(flecs::entity e, const VehicleParams& params, const GeoPoint& at, double yaw,
                double speed, bool on_ground, const AgentSetup& setup) {
    if (!valid_geo(at) || !std::isfinite(yaw)) {
        std::fprintf(stderr, "fm::ecs::attach_geo: invalid position or yaw, not attached\n");
        return;
    }
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
    auto* fc = e.try_get_mut<FlightController>();
    return fc && fc->agent ? &*fc->agent : nullptr;
}

bool follow(flecs::entity follower, flecs::entity leader, const FormationSlot& slot) {
    FlightAgent* a = agent(follower);
    if (!a || !leader.is_alive() || leader == follower) return false;
    a->join_formation(slot);
    if (a->mode() != FlightMode::Formation) return false;
    FollowLeader relation;
    relation.leader = leader;
    relation.slot = slot;
    if (const auto* fc = leader.try_get<FlightController>(); fc && fc->agent)
        a->set_leader_state(fc->agent->as_leader());
    follower.set<FollowLeader>(relation);
    return true;
}

void stop_following(flecs::entity follower) {
    follower.remove<FollowLeader>();
    if (FlightAgent* a = agent(follower); a && a->mode() == FlightMode::Formation) a->cruise();
}

bool land_on(flecs::entity e, flecs::entity carrier, const DeckSpot& spot) {
    FlightAgent* a = agent(e);
    if (!a || !carrier.is_alive() || carrier == e || !a->land_on(spot)) return false;
    FollowLeader relation;
    relation.leader = carrier;
    relation.slot = FormationSlot{spot.offset};
    if (const auto* fc = carrier.try_get<FlightController>(); fc && fc->agent)
        a->set_leader_state(fc->agent->as_leader());
    e.set<FollowLeader>(relation);
    return true;
}

bool takeoff_flight(flecs::entity leader, const std::vector<flecs::entity>& wingmen,
                    const TakeoffPlan& plan, const FlightTakeoff& options) {
    if (!plan.runway || !(options.interval >= 0.0)) return false;
    std::vector<flecs::entity> all{leader};
    all.insert(all.end(), wingmen.begin(), wingmen.end());
    for (std::size_t m = 0; m < all.size(); ++m) {
        const FixedWingAgent* fw = fixed_wing_of(all[m]);
        if (!fw || !fw->state().on_ground || (m > 0 && all[m] == leader)) return false;
    }
    FlightSequence seq;
    seq.takeoff = options;
    const int per_row = options.pairs ? 2 : 1;
    const int rows = (static_cast<int>(all.size()) + per_row - 1) / per_row;
    const Vec2 dir = heading_vector(plan.runway->heading);
    for (std::size_t m = 0; m < all.size(); ++m) {
        FlightSequence::Member mem;
        mem.entity = all[m];
        mem.row = static_cast<int>(m) / per_row;
        // Staggered line-up: the leader's row furthest down the runway, sides alternating.
        const double side = (m % 2 == 0 ? -1.0 : 1.0) * options.side_offset;
        const double along = (rows - 1 - mem.row) * options.row_spacing;
        Runway runway = *plan.runway;
        const Vec2 at = runway.start.xy() + dir * along + right_of(dir) * side;
        runway.start.x = at.x;
        runway.start.y = at.y;
        mem.plan = plan;
        mem.plan.runway = runway;
        mem.plan.hold = true;
        if (m > 0) {
            const std::size_t w = m - 1;
            const bool own = w < options.taxi.size() && !options.taxi[w].empty();
            mem.plan.then.reset();
            mem.plan.taxi = own ? options.taxi[w] : plan.taxi;
            mem.shared_taxi = !own && !plan.taxi.empty();
            mem.slot = w < options.slots.size() ? options.slots[w]
                                                : formation::echelon_right(static_cast<int>(w), 50.0);
        }
        // Without a path: straight to the line-up point.
        if (mem.plan.taxi.empty()) mem.plan.taxi = {runway.start};
        seq.members.push_back(std::move(mem));
    }
    leader.set<FlightSequence>(std::move(seq));
    return true;
}

bool land_flight(flecs::entity leader, const std::vector<flecs::entity>& wingmen,
                 const Runway& runway, const FlightLanding& options) {
    if (!is_finite(runway.start) || !std::isfinite(runway.heading) ||
        !(options.interval > 0.0) || !std::isfinite(options.break_distance))
        return false;
    std::vector<flecs::entity> all{leader};
    all.insert(all.end(), wingmen.begin(), wingmen.end());
    double natural_stop = 0.0;  // longest landing roll of the flight, from the runway start
    for (std::size_t m = 0; m < all.size(); ++m) {
        const FixedWingAgent* fw = fixed_wing_of(all[m]);
        if (!fw || fw->state().on_ground || (m > 0 && all[m] == leader)) return false;
        const double v_app = 0.5 * fw->overhead_initial_speed(runway);
        natural_stop = std::max(natural_stop, fw->params().touchdown_distance + 100.0 +
                                                  v_app * v_app /
                                                      (2.0 * std::max(fw->params().brake_deceleration, 0.1)));
    }
    // Break away from the wingmen of the leader's formation.
    bool left = true;
    if (options.left_pattern) {
        left = *options.left_pattern;
    } else {
        double offsets = 0.0;
        for (const flecs::entity w : wingmen) {
            const FollowLeader* f = w.try_get<FollowLeader>();
            if (f && f->leader == leader) offsets += f->slot.offset.x;
        }
        left = offsets >= 0.0;
    }
    const FixedWingAgent* lead_fw = fixed_wing_of(leader);
    FlightSequence seq;
    seq.landing = true;
    seq.land = options;
    seq.runway = runway;
    seq.left_pattern = left;
    seq.initial_speed = lead_fw->overhead_initial_speed(runway);
    seq.break_spacing = lead_fw->break_spacing(runway, options.interval, seq.initial_speed);
    seq.natural_stop = natural_stop;
    // Landing order = echelon order: the wingmen already on the echelon side (inner ones
    // first), then the ones on the break side, then those not in the leader's formation.
    const double echelon_side = left ? 1.0 : -1.0;  // echelon to the right for a left break
    struct Wingman {
        flecs::entity entity;
        Vec3 offset{};
        bool crossing = false;
    };
    std::vector<Wingman> keep, cross;
    std::vector<flecs::entity> others;
    for (const flecs::entity w : wingmen) {
        const FollowLeader* f = w.try_get<FollowLeader>();
        if (f && f->leader == leader && agent(w)->mode() == FlightMode::Formation) {
            const Vec3 o = f->slot.offset;
            const bool crossing = o.x * echelon_side < -1.0;
            (crossing ? cross : keep).push_back({w, o, crossing});
        } else {
            others.push_back(w);
        }
    }
    const auto depth = [&](const Wingman& w) { return std::abs(w.offset.x) - w.offset.y; };
    const auto by_depth = [&](const Wingman& a, const Wingman& b) { return depth(a) < depth(b); };
    std::stable_sort(keep.begin(), keep.end(), by_depth);
    std::stable_sort(cross.begin(), cross.end(), by_depth);
    std::vector<Wingman> echelon = keep;
    echelon.insert(echelon.end(), cross.begin(), cross.end());
    const double step = options.echelon_spacing * 0.7071;  // along and across per position
    FlightSequence::Member lead_member;
    lead_member.entity = leader;
    seq.members.push_back(lead_member);
    int crossing_index = 0;
    for (std::size_t k = 0; k < echelon.size(); ++k) {
        FlightSequence::Member mem;
        mem.entity = echelon[k].entity;
        const int index = static_cast<int>(k);
        mem.slot = left ? formation::echelon_right(index, options.echelon_spacing)
                        : formation::echelon_left(index, options.echelon_spacing);
        if (echelon[k].crossing) {
            // Straight back on its own side, behind the whole echelon.
            mem.transit.offset = {echelon[k].offset.x,
                                  -(static_cast<double>(echelon.size()) + 2.0 + crossing_index) * step,
                                  echelon[k].offset.z};
            mem.in_transit = true;
            ++crossing_index;
        }
        seq.members.push_back(mem);
    }
    for (const flecs::entity w : others) {
        FlightSequence::Member mem;
        mem.entity = w;
        seq.members.push_back(mem);
    }
    for (std::size_t m = 0; m < seq.members.size(); ++m) assign_landing(seq, m, seq.members[m]);
    // The leader starts its approach; its wingmen re-form in echelon for the break.
    FlightSequence::Member& lead = seq.members.front();
    lead.started = agent(leader)->land(lead.runway, lead.landing);
    for (std::size_t m = 1; m <= echelon.size(); ++m) {
        FlightSequence::Member& mem = seq.members[m];
        follow(mem.entity, leader, mem.in_transit ? mem.transit : mem.slot);
    }
    leader.set<FlightSequence>(std::move(seq));
    return true;
}

bool flight_sequence_active(flecs::entity leader) {
    return leader.is_alive() && leader.has<FlightSequence>();
}

}  // namespace fm::ecs
