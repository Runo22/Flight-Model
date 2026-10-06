#include "fm/rotorcraft.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "fm/guidance.hpp"

namespace fm {

namespace {
constexpr double kMaxSubstep = 0.01;      // s
constexpr double kNoGround = -1e9;
constexpr double kFormationGain = 0.8;    // 1/s, slot error -> velocity (with feedforward)
constexpr double kAltitudeGain = 0.8;     // 1/s
constexpr double kCrossTrackGain = 0.5;   // 1/s
constexpr double kVerticalTau = 0.5;      // s, vertical speed response
constexpr double kAttitudeRateMax = 6.0;  // rad/s, roll/pitch rate limit
constexpr double kYawTau = 0.5;           // s
constexpr double kLeaderTimeout = 2.0;    // s
constexpr double kHardImpactSpeed = 3.0;  // m/s, faster ground contact counts as a crash
}  // namespace

RotorcraftAgent::RotorcraftAgent(RotorcraftParams params, const InitialConditions& ic)
    : p_(std::move(params)), terrain_(std::max(0.5, p_.cruise_speed * 0.25)) {
    position_ = ic.position;
    attitude_.heading = wrap_2pi(ic.heading);
    yaw_target_ = attitude_.heading;
    course_target_ = attitude_.heading;
    course_anchor_ = position_.xy();
    speed_target_ = p_.cruise_speed;
    ground_reference_ = kNoGround;
    on_ground_ = ic.on_ground;
    hold_point_ = position_.xy();

    if (on_ground_) {
        ground_reference_ = ic.position.z;
        position_.z = ic.position.z + p_.gear_height;
        phase_ = FlightPhase::Parked;
        mode_ = FlightMode::Hover;
    } else {
        thrust_ = p_.mass * kGravity;
        phase_ = FlightPhase::Airborne;
        if (ic.speed > 0.5) {
            velocity_ = Vec3{std::sin(attitude_.heading), std::cos(attitude_.heading), 0.0} *
                        std::min(ic.speed, p_.max_speed);
            speed_target_ = std::min(ic.speed, p_.max_speed);
            mode_ = FlightMode::Cruise;
        } else {
            mode_ = FlightMode::Hover;
        }
    }
    altitude_target_ = position_.z;
    resolved_altitude_target_ = position_.z;
    update_output(isa(position_.z), {});
}

// --- Commands -------------------------------------------------------------------------

void RotorcraftAgent::cruise() {
    const double gs = velocity_.xy().length();
    if (gs < 1.0) {
        hover();
        return;
    }
    hold(heading_of(velocity_.xy()), resolved_altitude_target_, gs);
}

void RotorcraftAgent::hold(double course, double altitude, double speed, AltitudeRef altitude_ref) {
    mode_ = FlightMode::Cruise;
    route_.clear();
    course_target_ = wrap_2pi(course);
    course_anchor_ = position_.xy();
    if (altitude_ref != AltitudeRef::Keep) {
        altitude_target_ = altitude;
        altitude_ref_ = altitude_ref;
    }
    set_speed(speed);
}

void RotorcraftAgent::set_speed(double speed) {
    if (!std::isnan(speed) && speed > 0.0) speed_target_ = std::min(speed, p_.max_speed);
}

void RotorcraftAgent::hover() {
    if (on_ground_) {
        takeoff({});
        return;
    }
    mode_ = FlightMode::Hover;
    route_.clear();
    hold_point_ = position_.xy();
    altitude_target_ = resolved_altitude_target_;
    altitude_ref_ = AltitudeRef::Absolute;
}

void RotorcraftAgent::go_to(const Vec3& point, const GoToOptions& options) {
    Route r;
    Waypoint wp;
    wp.position = point;
    wp.altitude_ref = options.altitude_ref;
    wp.speed = options.speed;
    wp.fly_over = true;
    r.waypoints.push_back(wp);
    r.on_end = options.on_arrival;
    r.vertical = options.vertical;
    follow_route(std::move(r));
    if (mode_ == FlightMode::Route) mode_ = FlightMode::GoTo;
}

void RotorcraftAgent::follow_route(Route route) {
    if (route.waypoints.empty()) {
        hover();
        return;
    }
    if (on_ground_) {
        TakeoffPlan plan;
        plan.then = std::move(route);
        takeoff(plan);
        return;
    }
    route_.start(std::move(route), position_, resolved_altitude_target_);
    hold_timer_ = 0.0;
    waiting_ = false;
    mode_ = FlightMode::Route;
}

void RotorcraftAgent::loiter(const Vec3& center, double radius, bool clockwise,
                             AltitudeRef altitude_ref) {
    if (on_ground_) {
        Route r;
        Waypoint wp;
        wp.position = center;
        wp.altitude_ref = altitude_ref;
        r.waypoints.push_back(wp);
        r.on_end = EndBehavior::Loiter;
        r.loiter_radius = radius;
        follow_route(std::move(r));
        return;
    }
    route_.clear();
    if (altitude_ref != AltitudeRef::Keep) {
        altitude_target_ = center.z;
        altitude_ref_ = altitude_ref;
    }
    if (radius <= 0.0) {
        mode_ = FlightMode::Hover;
        hold_point_ = center.xy();
        return;
    }
    mode_ = FlightMode::Loiter;
    loiter_center_ = center;
    loiter_radius_ = radius;
    loiter_clockwise_ = clockwise;
}

bool RotorcraftAgent::takeoff(const TakeoffPlan& plan) {
    if (!on_ground_ || phase_ == FlightPhase::TerrainContact) return false;
    takeoff_ = plan;
    ground_reference_ = ground_elevation();
    hold_point_ = position_.xy();
    mode_ = FlightMode::Takeoff;
    phase_ = FlightPhase::InitialClimb;
    route_.clear();
    return true;
}

bool RotorcraftAgent::land() {
    if (on_ground_) return false;
    mode_ = FlightMode::Land;
    phase_ = FlightPhase::Landing;
    route_.clear();
    hold_point_ = position_.xy();
    return true;
}

void RotorcraftAgent::join_formation(const FormationSlot& slot) {
    if (on_ground_) return;
    mode_ = FlightMode::Formation;
    route_.clear();
    slot_ = slot;
    leader_tracker_.reset();
    leader_age_ = 0.0;
}

void RotorcraftAgent::set_leader_state(const LeaderState& leader) {
    leader_ = leader;
    leader_age_ = 0.0;
}

// --- Simulation -----------------------------------------------------------------------

void RotorcraftAgent::step(double dt, const Environment& env) {
    if (!(dt > 0.0)) return;
    terrain_.update(position_.xy(), env.terrain_elevation);

    SlotReference slot_ref{};
    if (mode_ == FlightMode::Formation) {
        leader_age_ += dt;
        if (leader_age_ > kLeaderTimeout) {
            hover();
        } else if (leader_.valid) {
            slot_ref = leader_tracker_.update(leader_, slot_, dt);
        }
    }

    const int substeps = std::max(1, static_cast<int>(std::ceil(dt / kMaxSubstep)));
    const double h = dt / substeps;
    AtmosphereSample atm{};
    for (int i = 0; i < substeps; ++i) {
        atm = isa(position_.z, env.temperature_offset);
        frame_time_ = h * i;
        SlotReference ref = slot_ref;
        ref.position += ref.velocity * frame_time_;
        const Command cmd = guidance(ref);
        integrate(cmd, atm, env.wind, h);
        hold_timer_ = std::max(0.0, hold_timer_ - h);
    }
    update_output(atm, env.wind);
}

RotorcraftAgent::Command RotorcraftAgent::guidance(const SlotReference& ref) {
    Command cmd;
    cmd.yaw = yaw_target_;

    const bool parked = phase_ == FlightPhase::Parked || phase_ == FlightPhase::Landed ||
                        phase_ == FlightPhase::TerrainContact;
    if (on_ground_ && parked && mode_ != FlightMode::Takeoff) {
        cmd.motors_off = true;
        return cmd;
    }

    if ((mode_ == FlightMode::GoTo || mode_ == FlightMode::Route) && route_.active()) {
        guide_route(cmd);  // may finish the route and switch mode
    }

    Vec2 v_h = cmd.velocity.xy();
    double altitude = altitude_target_now();
    double vz = kUnknown;  // NaN = derived from `altitude`
    bool face_motion = true;
    bool terrain_floor = true;

    switch (mode_) {
        case FlightMode::Cruise: {
            const Vec2 dir = heading_vector(course_target_);
            const double crosstrack = dot(position_.xy() - course_anchor_, right_of(dir));
            const double correction = std::clamp(-kCrossTrackGain * crosstrack,
                                                 -0.5 * speed_target_, 0.5 * speed_target_);
            v_h = dir * speed_target_ + right_of(dir) * correction;
            break;
        }
        case FlightMode::Hover:
            v_h = position_hold_velocity(hold_point_, speed_target_);
            break;
        case FlightMode::GoTo:
        case FlightMode::Route:
            altitude = resolved_altitude_target_;  // set by guide_route
            break;
        case FlightMode::Loiter: {
            const Vec2 r = position_.xy() - loiter_center_.xy();
            const double d = std::max(r.length(), 1e-6);
            const Vec2 ur = r / d;
            const Vec2 tangent = loiter_clockwise_ ? right_of(ur) : -right_of(ur);
            // Limit the orbit speed to what the acceleration limit can turn.
            const double speed = std::min(speed_target_,
                                          0.9 * std::sqrt(p_.max_acceleration * loiter_radius_));
            v_h = tangent * speed +
                  ur * std::clamp(position_gain() * (loiter_radius_ - d), -speed, speed);
            break;
        }
        case FlightMode::Takeoff: {
            terrain_floor = false;
            face_motion = false;
            const double climb =
                takeoff_.climb_height > 0.0 ? takeoff_.climb_height : p_.takeoff_height;
            v_h = position_hold_velocity(hold_point_, 2.0);
            altitude = ground_reference_ + p_.gear_height + climb;
            vz = std::min(p_.climb_rate_max, std::max(1.0, kAltitudeGain * (altitude - position_.z)));
            if (position_.z >= altitude - 0.5) {
                phase_ = FlightPhase::Airborne;
                altitude_target_ = altitude;
                altitude_ref_ = AltitudeRef::Absolute;
                resolved_altitude_target_ = altitude;
                if (takeoff_.then) {
                    Route r = std::move(*takeoff_.then);
                    takeoff_.then.reset();
                    follow_route(std::move(r));
                } else {
                    hover();
                }
            }
            break;
        }
        case FlightMode::Land: {
            terrain_floor = false;
            face_motion = false;
            v_h = position_hold_velocity(hold_point_, speed_target_);
            const double height = position_.z - p_.gear_height - ground_elevation();
            vz = -std::min(p_.descent_rate_max, std::max(p_.landing_speed, 0.4 * height));
            break;
        }
        case FlightMode::Formation: {
            if (!leader_.valid) {
                v_h = position_hold_velocity(hold_point_, speed_target_);
                break;
            }
            face_motion = false;
            const Vec3 err = ref.position - position_;
            v_h = ref.velocity.xy() + err.xy() * kFormationGain;
            altitude = ref.position.z;
            cmd.yaw = ref.heading;
            // Push away from the leader when closer than the minimum separation.
            const double min_sep =
                slot_.min_separation > 0.0 ? slot_.min_separation : p_.formation_min_separation;
            const Vec3 from_leader =
                position_ - (leader_.position + leader_.velocity * frame_time_);
            const double dist = from_leader.length();
            if (dist < min_sep) {
                const Vec3 away = dist > 1e-6 ? from_leader / dist : Vec3{0.0, 0.0, 1.0};
                v_h += away.xy() * (p_.max_speed * (1.0 - dist / min_sep));
                altitude += away.z * (min_sep - dist);
            }
            break;
        }
        default:
            break;
    }

    // Limit the horizontal speed.
    const double vh_len = v_h.length();
    if (vh_len > p_.max_speed) v_h = v_h * (p_.max_speed / vh_len);

    if (terrain_floor && terrain_.valid()) {
        const double lookahead = std::max(velocity_.xy().length(), 1.0) * p_.terrain_lookahead;
        altitude = std::max(altitude, terrain_.predicted_max(lookahead) + p_.min_terrain_clearance);
    }
    if (std::isnan(vz)) vz = vertical_speed_to(altitude);
    if (mode_ == FlightMode::Formation && leader_.valid)
        vz = std::clamp(vz + ref.velocity.z, -p_.descent_rate_max, p_.climb_rate_max);

    if (face_motion && v_h.length() > 2.0) yaw_target_ = heading_of(v_h);
    if (mode_ != FlightMode::Formation) cmd.yaw = yaw_target_;
    cmd.velocity = {v_h.x, v_h.y, vz};
    return cmd;
}

void RotorcraftAgent::guide_route(Command& cmd) {
    const Vec2 pos = position_.xy();
    LegProgress lp = route_.progress(pos);
    const EndBehavior end = route_.route().on_end;

    auto advance = [&]() {
        if (!route_.advance(position_, resolved_altitude_target_)) {
            finish_route(end);
            return false;
        }
        lp = route_.progress(pos);
        return true;
    };

    auto hold_at_waypoint = [&](const Waypoint& wp) {
        const Vec2 v = position_hold_velocity(wp.position.xy(), speed_target_);
        cmd.velocity = {v.x, v.y, 0.0};
        resolved_altitude_target_ =
            RouteTracker::resolve_altitude(wp, terrain_, 0.0, resolved_altitude_target_);
    };
    // Speed at which the current leg ends: stop for holds and route ends, slow down for
    // sharp corners.
    auto end_speed_of_leg = [&]() {
        const Waypoint& wp = route_.target();
        if (wp.hold_time > 0.0) return 0.0;
        if (const Waypoint* next = route_.next()) {
            const Vec2 next_dir = (next->position.xy() - wp.position.xy()).normalized();
            const double turn = std::abs(angle_right(lp.direction, next_dir));
            return speed_target_ * std::clamp(std::cos(turn), 0.15, 1.0);
        }
        return end == EndBehavior::ContinueStraight ? speed_target_ : 0.0;
    };

    if (waiting_) {
        hold_at_waypoint(route_.target());
        if (hold_timer_ > 0.0) return;
        waiting_ = false;
        if (!advance()) return;
    } else if (lp.remaining() <= std::max(1.0, end_speed_of_leg())) {
        if (route_.target().hold_time > 0.0) {
            waiting_ = true;
            hold_timer_ = route_.target().hold_time;
            hold_at_waypoint(route_.target());
            return;
        }
        if (!advance()) return;
    }

    set_speed(route_.target().speed);
    const double end_speed = end_speed_of_leg();
    const double brake = 0.5 * p_.max_acceleration;
    const double remaining = std::max(lp.remaining(), 0.0);
    const double along_speed =
        std::min({speed_target_, std::sqrt(end_speed * end_speed + 2.0 * brake * remaining),
                  end_speed + position_gain() * remaining});
    const double correction = std::clamp(-kCrossTrackGain * lp.crosstrack, -0.5 * speed_target_,
                                         0.5 * speed_target_);
    const Vec2 v = lp.direction * along_speed + right_of(lp.direction) * correction;
    cmd.velocity = {v.x, v.y, 0.0};
    const double lookahead = std::max(velocity_.xy().length(), 1.0) * 2.0;
    resolved_altitude_target_ =
        route_.altitude_target(lp, terrain_, lookahead, resolved_altitude_target_);
}

void RotorcraftAgent::finish_route(EndBehavior behavior) {
    const Waypoint last = route_.target();
    const double last_altitude =
        RouteTracker::resolve_altitude(last, terrain_, 0.0, resolved_altitude_target_);
    const bool agl = last.altitude_ref == AltitudeRef::AboveTerrain;
    const double radius = route_.route().loiter_radius;
    const Vec2 leg = route_.leg_end() - route_.leg_start();
    route_.clear();
    altitude_ref_ = agl ? AltitudeRef::AboveTerrain : AltitudeRef::Absolute;
    altitude_target_ = agl ? last.position.z : last_altitude;
    resolved_altitude_target_ = last_altitude;

    switch (behavior) {
        case EndBehavior::ContinueStraight:
            mode_ = FlightMode::Cruise;
            course_target_ = leg.length() > 0.1 ? heading_of(leg) : attitude_.heading;
            course_anchor_ = last.position.xy();
            break;
        case EndBehavior::Loiter:
            if (radius > 0.0) {
                mode_ = FlightMode::Loiter;
                loiter_center_ = {last.position.x, last.position.y, last_altitude};
                loiter_radius_ = radius;
                loiter_clockwise_ = true;
            } else {
                mode_ = FlightMode::Hover;
                hold_point_ = last.position.xy();
            }
            break;
        case EndBehavior::Land:
            mode_ = FlightMode::Land;
            phase_ = FlightPhase::Landing;
            hold_point_ = last.position.xy();
            break;
    }
}

void RotorcraftAgent::integrate(const Command& cmd, const AtmosphereSample& atm, const Vec3& wind,
                                double dt) {
    (void)atm;
    const double m = p_.mass;
    const double g = kGravity;
    const double weight = m * g;
    const double ground = ground_elevation();

    if (cmd.motors_off) {
        thrust_ = 0.0;
        velocity_ = {};
        attitude_.pitch = 0.0;
        attitude_.roll = 0.0;
        if (ground > kNoGround) position_.z = ground + p_.gear_height;
        return;
    }

    // Quadratic drag sized so that max tilt holds exactly max_speed in still air.
    const double drag_coeff = weight * std::tan(p_.max_tilt) / (p_.max_speed * p_.max_speed);
    const Vec3 v_air = velocity_ - wind;
    const Vec3 drag = v_air * (-drag_coeff * v_air.length());

    // Desired acceleration from the velocity command.
    Vec2 a_h = (cmd.velocity.xy() - velocity_.xy()) / p_.velocity_time_constant;
    const double a_len = a_h.length();
    if (a_len > p_.max_acceleration) a_h = a_h * (p_.max_acceleration / a_len);
    const double t_max = p_.thrust_to_weight * weight;
    const double a_z = std::clamp((cmd.velocity.z - velocity_.z) / kVerticalTau, -0.6 * g,
                                  0.8 * (p_.thrust_to_weight - 1.0) * g);

    // Required thrust vector (gravity and drag compensated), tilt and thrust limited with
    // priority on the vertical component.
    Vec3 force = Vec3{a_h.x, a_h.y, a_z + g} * m - drag;
    force.z = std::clamp(force.z, 0.2 * weight, 0.95 * t_max);
    const double max_h = std::min(force.z * std::tan(p_.max_tilt),
                                  std::sqrt(std::max(t_max * t_max - force.z * force.z, 0.0)));
    const double f_h = force.xy().length();
    if (f_h > max_h) {
        force.x *= max_h / f_h;
        force.y *= max_h / f_h;
    }

    // Attitude follows the thrust direction with a lag; yaw turns at a limited rate.
    const double yaw_rate = std::clamp(wrap_pi(cmd.yaw - attitude_.heading) / kYawTau,
                                       -p_.yaw_rate_max, p_.yaw_rate_max);
    attitude_.heading = wrap_2pi(attitude_.heading + yaw_rate * dt);
    const Attitude desired = attitude_from_up(force, attitude_.heading);
    const double max_step = kAttitudeRateMax * dt;
    attitude_.roll = approach(attitude_.roll,
                              lag(attitude_.roll, desired.roll, p_.attitude_time_constant, dt),
                              max_step);
    attitude_.pitch = approach(attitude_.pitch,
                               lag(attitude_.pitch, desired.pitch, p_.attitude_time_constant, dt),
                               max_step);
    thrust_ = lag(thrust_, std::min(force.length(), t_max), p_.thrust_time_constant, dt);

    const Vec3 up = body_axes(attitude_).up;
    const Vec3 accel = (up * thrust_ + drag) / m - Vec3{0.0, 0.0, g};
    velocity_ += accel * dt;
    position_ += velocity_ * dt;

    if (ground > kNoGround && position_.z - p_.gear_height <= ground) {
        const double impact = -velocity_.z;
        position_.z = ground + p_.gear_height;
        velocity_.z = std::max(velocity_.z, 0.0);
        const double friction = std::exp(-8.0 * dt);
        velocity_.x *= friction;
        velocity_.y *= friction;
        if (!on_ground_ && mode_ != FlightMode::Takeoff) {
            phase_ = impact > kHardImpactSpeed ? FlightPhase::TerrainContact : FlightPhase::Landed;
            mode_ = FlightMode::Land;
            route_.clear();
        }
        on_ground_ = true;
    } else {
        on_ground_ = false;
    }
}

void RotorcraftAgent::update_output(const AtmosphereSample& atm, const Vec3& wind) {
    const Vec3 v_air = velocity_ - wind;
    out_.position = position_;
    out_.velocity = velocity_;
    out_.attitude = attitude_;
    out_.airspeed = v_air.length();
    out_.ground_speed = velocity_.xy().length();
    out_.vertical_speed = velocity_.z;
    out_.track = out_.ground_speed > 0.1 ? heading_of(velocity_.xy()) : attitude_.heading;
    out_.flight_path_angle = std::atan2(velocity_.z, std::max(out_.ground_speed, 1e-6));
    out_.angle_of_attack = 0.0;
    out_.load_factor = thrust_ / (p_.mass * kGravity);
    out_.throttle = thrust_ / (p_.thrust_to_weight * p_.mass * kGravity);
    out_.mach = out_.airspeed / atm.speed_of_sound;
    if (terrain_.valid()) {
        out_.height_above_terrain = position_.z - terrain_.current();
    } else {
        out_.height_above_terrain =
            ground_reference_ > kNoGround ? position_.z - ground_reference_ : kUnknown;
    }
    out_.on_ground = on_ground_;
    out_.speed_brake = false;
    out_.flaps = false;
    out_.phase = phase_;
}

// --- Helpers --------------------------------------------------------------------------

Vec2 RotorcraftAgent::position_hold_velocity(const Vec2& target, double max_speed) const {
    const Vec2 d = target - position_.xy();
    const double dist = d.length();
    if (dist < 1e-3) return {};
    const double brake = 0.5 * p_.max_acceleration;
    const double speed =
        std::min({max_speed, std::sqrt(2.0 * brake * dist), position_gain() * dist});
    return d * (speed / dist);
}

double RotorcraftAgent::vertical_speed_to(double altitude) const {
    return std::clamp(kAltitudeGain * (altitude - position_.z), -p_.descent_rate_max,
                      p_.climb_rate_max);
}

double RotorcraftAgent::ground_elevation() const {
    return terrain_.valid() ? terrain_.current() : ground_reference_;
}

double RotorcraftAgent::altitude_target_now() const {
    if (altitude_ref_ != AltitudeRef::AboveTerrain) return altitude_target_;
    const double ground = terrain_.valid() ? terrain_.extrapolate(velocity_.xy().length() * 2.0)
                                           : std::max(ground_reference_, 0.0);
    return ground + altitude_target_;
}

}  // namespace fm
