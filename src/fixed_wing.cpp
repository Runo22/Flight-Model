#include "fm/fixed_wing.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace fm {

namespace {
constexpr double kMaxSubstep = 0.02;       // s
constexpr double kMinAirspeed = 1.0;       // m/s, numerical floor
constexpr double kNoGround = -1e9;         // no ground reference known
constexpr double kLowBankHeight = 30.0;    // m, bank is restricted below this height
constexpr double kLowBankLimit = deg2rad(10.0);
constexpr double kTakeoffBankLimit = deg2rad(15.0);
constexpr double kWheelBrakeFriction = 0.3;
constexpr double kStallMargin = 1.3;       // minimum speed, clean: 1.3 Vs
constexpr double kFlapsStallMargin = 1.2;  // minimum speed with flaps: 1.2 Vs
constexpr double kTurnStallMargin = 1.2;   // bank limited to keep n <= n_stall / 1.2
constexpr double kL1TurnFactor = 0.6;      // L1 distance >= 0.6 * K * turn radius
constexpr double kLeaderTimeout = 2.0;     // s without leader updates before giving up
constexpr double kFormationSpeedTau = 3.0; // s, speed response while station keeping
constexpr double kFormationAlongGain = 0.1;  // 1/s, along-track error to speed
constexpr double kTropopauseSigma = 0.2971;  // ISA density ratio at 11 km
constexpr double kClimbModeEnter = 150.0;  // m below the target altitude
constexpr double kClimbModeExit = 30.0;    // m
constexpr double kClimbSpeedRefresh = 150.0;  // m of altitude change before recomputing Vy
// Formation lead: share of the sustained speed the leader flies at most, and the most an
// outside follower is asked for in a turn.
constexpr double kLeadSpeedShare = 0.88;
constexpr double kAvoidMinHeight = 150.0;  // m above the ground: no resolutions below
constexpr double kAvoidMargin = 30.0;      // m beyond the protected height
constexpr double kAvoidClearTime = 3.0;    // s without a conflict before the resolution ends
constexpr double kAvoidLevel = 10.0;       // m: closer vertically counts as co-altitude
constexpr double kFollowerSpeedShare = 0.92;
constexpr double kRejoinSpeedShare = 0.8;  // of the cruise speed, while followers rejoin
// Leading followers: share of the slowest follower's formation speed limit flown at most,
// in formation and while one of them rejoins.
constexpr double kLeadFollowerShare = 0.9;
constexpr double kRejoinFollowerShare = 0.75;
constexpr double kRejoinClosing = 0.35;    // of the cruise speed, a rejoining follower's overtake
constexpr double kRejoinDistance = 300.0;  // m from its slot: a follower is rejoining
constexpr double kHoldDistance = 2000.0;   // m from its slot: the leader holds for it
constexpr double kTurningRejoinRate = 0.01;  // rad/s: a leader turning faster is circled
constexpr double kMaxJoinDistance = 2000.0;   // m, station keeping within it
constexpr double kRejoinTaper = 20.0;          // s: overtake at most distance / 20 s
constexpr double kHoldSettleTime = 5.0;        // s with the followers close: stop holding
constexpr double kLoiterBank = deg2rad(30.0);  // default orbit radius: at most this bank
constexpr double kLoiterStallMargin = 1.4;     // default orbit speed: at least 1.4 Vs
// Taxi and line-up.
constexpr double kLineUpHeading = deg2rad(3.0);  // lined up: heading within this ...
constexpr double kLineUpOffset = 3.0;            // ... and this close to the centerline (m)
constexpr double kOnRunwayOffset = 30.0;         // m from the centerline: on the runway
constexpr double kCenterlineLength = 1.0e4;      // m, line followed while lining up
constexpr double kRollLookahead = 30.0;          // m, centerline tracking on the takeoff run
constexpr double kTaxiDeceleration = 1.0;        // m/s^2, planned slowing
constexpr double kTaxiTurnAccel = 1.5;           // m/s^2, lateral, sets the corner speed
constexpr double kTaxiMaxAccel = 0.6;            // m/s^2
constexpr double kTaxiMaxBrake = 3.0;            // m/s^2
constexpr double kTaxiMaxThrottle = 0.6;
constexpr double kTaxiStopSpeed = 0.01;          // m/s
constexpr double kTaxiStopDistance = 1.5;        // m short of the end: arrived
constexpr double kGroundLateralAccel = 3.0;      // m/s^2, steering limit at speed
constexpr double kSteerRateAtSpeed = deg2rad(5.0);  // rad/s, allowed at any speed
constexpr double kHoldStopTime = 1.0;            // s, stopping when lined up to wait
// Overhead pattern.
constexpr double kBreakBank = deg2rad(60.0);       // most bank in the break turn
constexpr double kInitialSpeedFactor = 2.0;        // initial leg speed / approach speed
constexpr double kDownwindSpeedFactor = 1.3;       // downwind speed / approach speed
constexpr double kBaseSpeedFactor = 1.15;          // base turn speed / approach speed
constexpr double kBaseBank = deg2rad(45.0);        // most bank planned for the base turn
constexpr double kShortFinalTime = 35.0;           // s straight on the glide path
constexpr double kInitialCorridor = 2000.0;        // m beside the runway line: on initial

// Bank of the overhead break turn: hard, within the AI's bank and load factor limits.
double break_bank(const FixedWingParams& p) {
    return std::min({kBreakBank, p.bank_max, std::acos(1.0 / std::max(p.load_factor_max, 1.01))});
}

// Thrust / power lapse at the density ratio sigma: sigma^n in the troposphere; above the
// tropopause (constant temperature) in proportion to the density.
double density_lapse(double sigma, double exponent) {
    return sigma >= kTropopauseSigma
               ? std::pow(sigma, exponent)
               : std::pow(kTropopauseSigma, exponent) * sigma / kTropopauseSigma;
}
}  // namespace

FixedWingAgent::FixedWingAgent(FixedWingParams params, const InitialConditions& ic)
    : p_(std::move(params)), terrain_(std::max(1.0, p_.cruise_speed * 0.25)) {
    mass_ = p_.mass;
    if (fuel_modelled()) set_fuel(std::isnan(ic.fuel) ? p_.fuel_capacity : ic.fuel);
    if (p_.engine.critical_altitude > 0.0) {
        rated_lapse_ = density_lapse(isa(p_.engine.critical_altitude).sigma,
                                     p_.engine.density_exponent);
    }
    if (p_.cruise_altitude >= 0.0)
        cruise_calibrated_ = calibrated_airspeed(p_.cruise_speed, isa(p_.cruise_altitude));
    position_ = ic.position;
    heading_ = wrap_2pi(ic.heading);
    course_target_ = heading_;
    course_anchor_ = position_.xy();
    runway_elevation_ = kNoGround;
    on_ground_ = ic.on_ground;
    gear_down_ = on_ground_ || !p_.retractable_gear;

    const AtmosphereSample atm = isa(position_.z);
    speed_target_ = default_speed(atm);  // the initial speed is not a speed command
    if (on_ground_) {
        runway_elevation_ = ic.position.z;
        position_.z = ic.position.z + p_.gear_height;
        tas_ = std::max(0.0, ic.speed);
        throttle_ = p_.engine.idle_throttle;
        phase_ = FlightPhase::Parked;
    } else {
        // Start trimmed for level flight.
        tas_ = ic.speed > 0.0 ? ic.speed : speed_target_;
        trim_level(atm);
        phase_ = FlightPhase::Airborne;
    }
    altitude_target_ = position_.z;
    resolved_altitude_target_ = position_.z;
    update_output(atm, {});
}

// --- Commands -------------------------------------------------------------------------

void FixedWingAgent::cruise() {
    const double course = out_.ground_speed > 1.0 ? out_.track : heading_;
    hold_course(course, position_.xy(), resolved_altitude_target_, kUnknown,  // keep the speed
                AltitudeRef::Absolute);
}

void FixedWingAgent::hold(double course, double altitude, double speed, AltitudeRef altitude_ref) {
    hold_course(course, position_.xy(), altitude, speed, altitude_ref);
}

void FixedWingAgent::hold_course(double course, const Vec2& anchor, double altitude, double speed,
                                 AltitudeRef altitude_ref) {
    mode_ = FlightMode::Cruise;
    route_.clear();
    course_target_ = wrap_2pi(course);
    course_anchor_ = anchor;
    if (altitude_ref != AltitudeRef::Keep) {
        altitude_target_ = altitude;
        altitude_ref_ = altitude_ref;
    }
    set_speed(speed);
}

void FixedWingAgent::set_speed(double speed, SpeedRef ref) {
    if (!std::isfinite(speed) || speed <= 0.0) return;
    speed_commanded_ = true;
    speed_ref_ = ref;
    speed_command_ = speed;
    speed_target_ = true_airspeed(speed, ref, isa(position_.z, temperature_offset_));
}

void FixedWingAgent::set_fuel(double fuel) {
    if (!fuel_modelled() || !std::isfinite(fuel)) return;
    fuel_ = std::clamp(fuel, 0.0, p_.fuel_capacity);
    mass_ = p_.mass - (p_.fuel_capacity - fuel_);
    out_.fuel = fuel_;
    out_.mass = mass_;
}

void FixedWingAgent::go_to(const Vec3& point, const GoToOptions& options) {
    Route r;
    Waypoint wp;
    wp.position = point;
    wp.altitude_ref = options.altitude_ref;
    wp.speed = options.speed;
    wp.speed_ref = options.speed_ref;
    wp.fly_over = true;
    r.waypoints.push_back(wp);
    r.on_end = options.on_arrival;
    r.vertical = options.vertical;
    follow_route(std::move(r));
    if (mode_ == FlightMode::Route) mode_ = FlightMode::GoTo;
}

void FixedWingAgent::follow_route(Route route) {
    if (route.waypoints.empty()) {
        cruise();
        return;
    }
    if (on_ground_) {
        // A parked aircraft takes off along its current heading first.
        TakeoffPlan plan;
        plan.then = std::move(route);
        takeoff(plan);
        return;
    }
    route_.start(std::move(route), position_, resolved_altitude_target_);
    mode_ = FlightMode::Route;
}

void FixedWingAgent::loiter(const Vec3& center, double radius, bool clockwise,
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
    mode_ = FlightMode::Loiter;
    route_.clear();
    loiter_center_ = center;
    loiter_radius_ = radius > 0.0 ? radius : loiter_radius_default();
    loiter_clockwise_ = clockwise;
    loiter_here_ = false;
    if (altitude_ref != AltitudeRef::Keep) {
        altitude_target_ = center.z;
        altitude_ref_ = altitude_ref;
    }
}

bool FixedWingAgent::takeoff(const TakeoffPlan& plan) {
    if (!on_ground_ || phase_ == FlightPhase::TerrainContact) return false;
    takeoff_ = plan;
    mode_ = FlightMode::Takeoff;
    route_.clear();
    // Ground path: taxi points, then the runway start and its centerline.
    const Vec2 here = position_.xy();
    std::vector<Vec2> path{here};
    for (const Vec3& p : plan.taxi) path.push_back(p.xy());
    double heading = heading_;
    Vec2 origin = path.back();
    if (plan.runway) {
        heading = plan.runway->heading;
        const Vec2 start = plan.runway->start.xy();
        const Vec2 dir = heading_vector(heading);
        if (plan.taxi.empty() && std::abs(dot(here - start, right_of(dir))) <= kOnRunwayOffset) {
            // On the runway: the run starts abeam the current position.
            origin = start + dir * dot(here - start, dir);
            path = {origin};
        } else {
            path.push_back(start);
            origin = start;
        }
    } else if (std::isfinite(plan.heading)) {
        heading = plan.heading;
    } else {
        // Straight on from the last taxi segment.
        for (std::size_t i = path.size() - 1; i > 0; --i) {
            if ((path[i] - path[i - 1]).length() > 1.0) {
                heading = heading_of(path[i] - path[i - 1]);
                break;
            }
        }
    }
    takeoff_heading_ = wrap_2pi(heading);
    course_target_ = takeoff_heading_;
    takeoff_origin_ = origin;
    takeoff_released_ = !plan.hold;
    const Vec2 dir = heading_vector(takeoff_heading_);
    path.push_back(origin + dir * kCenterlineLength);
    taxi_.start(std::move(path), false);
    lining_up_ = taxi_.points().size() > 2 ||
                 std::abs(wrap_pi(heading_ - takeoff_heading_)) > kLineUpHeading ||
                 std::abs(dot(here - origin, right_of(dir))) > kLineUpOffset;
    if (!lining_up_) taxi_.clear();
    runway_elevation_ = ground_elevation();
    flaps_ = true;
    if (lining_up_) {
        phase_ = FlightPhase::Taxi;
    } else {
        phase_ = takeoff_released_ ? FlightPhase::TakeoffRoll : FlightPhase::LineUp;
    }
    out_.phase = phase_;
    return true;
}

bool FixedWingAgent::release_takeoff() {
    if (mode_ != FlightMode::Takeoff || !on_ground_) return false;
    takeoff_released_ = true;
    return true;
}

bool FixedWingAgent::taxi(std::span<const Vec3> path, double final_heading) {
    if (!on_ground_ || phase_ == FlightPhase::TerrainContact) return false;
    std::vector<Vec2> points{position_.xy()};
    for (const Vec3& p : path) points.push_back(p.xy());
    if (std::isfinite(final_heading) && points.size() >= 2) {
        // Arrive along the final heading: a straight last stretch, long enough to line up.
        const Vec2 end = points.back();
        const Vec2 dir = heading_vector(final_heading);
        const Vec2 last_leg = end - points[points.size() - 2];
        if (dot(last_leg.normalized(), dir) < std::cos(deg2rad(5.0)))
            points.insert(points.end() - 1, end - dir * (6.0 * taxi_settings().turn_radius));
    }
    mode_ = FlightMode::Taxi;
    route_.clear();
    flaps_ = false;
    speed_brake_ = false;
    taxi_.start(std::move(points), true);
    phase_ = FlightPhase::Taxi;
    out_.phase = phase_;
    return true;
}

void FixedWingAgent::hover() {
    loiter(position_, 0.0, true, AltitudeRef::Absolute);
    loiter_here_ = mode_ == FlightMode::Loiter;
}

bool FixedWingAgent::land(const Runway& runway, const LandingOptions& options) {
    if (on_ground_) return false;
    mode_ = FlightMode::Land;
    route_.clear();
    taxi_.clear();
    runway_ = runway;
    runway_.heading = wrap_2pi(runway.heading);
    runway_elevation_ = runway.start.z;
    landing_ = options;
    landing_stage_ = LandingStage::Positioning;
    phase_ = FlightPhase::Approach;
    if (options.pattern == LandingPattern::Overhead) {
        // Already flying along the runway towards the break point (e.g. next to the leader
        // of a formation): the initial leg starts here, on a line parallel to the runway.
        const Vec2 dir = heading_vector(runway_.heading);
        const Vec2 rel = position_.xy() - runway_.start.xy();
        const double along = dot(rel, dir);
        const double offset = dot(rel, right_of(dir));
        const double track = out_.ground_speed > 1.0 ? out_.track : heading_;
        if (std::abs(wrap_pi(track - runway_.heading)) < deg2rad(45.0) &&
            along < options.break_distance && along > -2.0 * p_.final_approach_distance &&
            std::abs(offset) < kInitialCorridor) {
            landing_stage_ = LandingStage::Initial;
            initial_offset_ = std::isfinite(options.initial_offset) ? options.initial_offset : offset;
        }
    }
    return true;
}

void FixedWingAgent::join_formation(const FormationSlot& slot) {
    if (on_ground_) return;
    mode_ = FlightMode::Formation;
    route_.clear();
    slot_ = slot;
    leader_tracker_.reset();
    leader_age_ = 0.0;
}

void FixedWingAgent::set_formation_lead(double lateral_extent, double rejoin_distance,
                                        double follower_speed) {
    lead_extent_ = std::isfinite(lateral_extent) ? std::max(lateral_extent, 0.0) : 0.0;
    lead_rejoin_distance_ = std::isfinite(rejoin_distance) ? std::max(rejoin_distance, 0.0) : 0.0;
    lead_follower_speed_ =
        std::isfinite(follower_speed) && follower_speed > 0.0 ? follower_speed : kUnknown;
    lead_age_ = 0.0;
}

void FixedWingAgent::set_traffic(std::span<const Traffic> traffic) {
    conflict_now_ = false;
    // No resolutions on the ground, during takeoff / landing or close to the ground.
    const double height = position_.z - p_.gear_height - ground_elevation();
    if (on_ground_ || mode_ == FlightMode::Takeoff || mode_ == FlightMode::Land ||
        height < kAvoidMinHeight)
        return;
    const Vec3 own_velocity = out_.velocity;
    double earliest = p_.avoidance_lookahead;
    const Traffic* threat = nullptr;
    Vec3 miss{};
    for (const Traffic& t : traffic) {
        if (!is_finite(t.position) || !is_finite(t.velocity)) continue;
        const Vec3 r = t.position - position_;
        const Vec3 w = t.velocity - own_velocity;
        const double w2 = dot(w, w);
        const double t_cpa = w2 > 1e-6 ? std::max(-dot(r, w) / w2, 0.0) : 0.0;
        if (t_cpa > earliest) continue;
        const Vec3 m = r + w * t_cpa;  // the other one relative to us at the closest point
        if (m.xy().length() >= p_.avoidance_radius || std::abs(m.z) >= p_.avoidance_height)
            continue;
        earliest = t_cpa;
        threat = &t;
        miss = m;
    }
    if (!threat) return;
    conflict_now_ = true;
    if (avoid_sense_ == 0) {
        // Move away vertically; level with each other, decide by the relative position so
        // that both aircraft pick opposite senses.
        const Vec3 r = threat->position - position_;
        if (std::abs(miss.z) > kAvoidLevel) {
            avoid_sense_ = miss.z > 0.0 ? -1 : 1;
        } else {
            avoid_sense_ = (std::abs(r.x) > 1e-3 ? r.x : r.y) > 0.0 ? 1 : -1;
        }
    }
    const double needed = p_.avoidance_height + kAvoidMargin;
    const double other_altitude = threat->position.z + threat->velocity.z * earliest;
    avoid_altitude_ = other_altitude + avoid_sense_ * needed;
    // A descent that would go through the terrain floor becomes a climb.
    if (avoid_sense_ < 0 && avoid_altitude_ < ground_elevation() + p_.min_terrain_clearance) {
        avoid_sense_ = 1;
        avoid_altitude_ = other_altitude + needed;
    }
    avoid_clear_time_ = 0.0;
}

void FixedWingAgent::set_leader_state(const LeaderState& leader) {
    leader_ = leader;
    leader_age_ = 0.0;
}

bool FixedWingAgent::relocate(const Vec3& position, double heading) {
    const Relocation r = classify_relocation(position_, heading_, position, heading);
    motion_.reset();
    // Samples from far away say nothing about the slope here.
    if ((position - position_).xy().length() > 2.0 * terrain_.spacing()) terrain_.reset();
    position_ = position;
    heading_ = wrap_2pi(heading_ + r.turn);
    // Ground contact is resolved in the next step, once the terrain there is known.
    relocation_ = relocation_ ? combine(*relocation_, r) : r;

    if (r.jump) {
        // Restart the current task from here.
        const Vec2 here = position_.xy();
        switch (mode_) {
            case FlightMode::Cruise:
            case FlightMode::Formation:
                course_target_ = wrap_2pi(course_target_ + r.turn);
                course_anchor_ = here;
                if (r.vertical_jump && altitude_ref_ == AltitudeRef::Absolute)
                    altitude_target_ = position_.z;
                leader_tracker_.reset();
                break;
            case FlightMode::GoTo:
            case FlightMode::Route:
                route_.restart_leg(position_,
                                   r.vertical_jump ? position_.z : resolved_altitude_target_);
                break;
            case FlightMode::Loiter:
                if (loiter_here_) {
                    loiter_center_ = position_;
                    if (r.vertical_jump) altitude_target_ = position_.z;
                }
                break;
            case FlightMode::Takeoff:
                if (lining_up_) {
                    taxi_.rejoin(here);
                } else {
                    takeoff_origin_ = here;
                    takeoff_heading_ = wrap_2pi(takeoff_heading_ + r.turn);
                    course_target_ = takeoff_heading_;
                }
                break;
            case FlightMode::Taxi:
                taxi_.rejoin(here);
                break;
            case FlightMode::Land:
                if (landing_stage_ == LandingStage::Rollout) {
                    runway_.heading = wrap_2pi(runway_.heading + r.turn);
                    taxi_.clear();  // brake to a stop from here
                } else {
                    landing_stage_ = LandingStage::Positioning;  // a new approach from here
                }
                break;
            default:
                break;
        }
    }
    update_output(isa(position_.z), wind_);
    return true;
}

// --- Simulation -----------------------------------------------------------------------

void FixedWingAgent::step(double dt, const Environment& input) {
    if (!(dt > 0.0) || !std::isfinite(dt)) return;
    const Environment env = sanitized(input);  // bad host data must not poison the state
    wind_ = env.wind;
    temperature_offset_ = env.temperature_offset;
    terrain_.update(position_.xy(), env.terrain_elevation);
    if (relocation_) {
        const Relocation r = *relocation_;
        relocation_.reset();
        settle_relocation(r);
    }
    // CAS and Mach targets (and the default speeds): the true airspeed for the altitude
    // reached one speed time constant ahead, so the speed keeps up while climbing or
    // descending.
    {
        const double ahead = position_.z + out_.vertical_speed * p_.speed_time_constant;
        const AtmosphereSample atm_ahead = isa(ahead, temperature_offset_);
        if (!speed_commanded_) {
            speed_target_ = default_speed(atm_ahead);
        } else if (speed_ref_ != SpeedRef::True) {
            speed_target_ = true_airspeed(speed_command_, speed_ref_, atm_ahead);
        }
    }

    lead_age_ += dt;
    if (lead_hold_)
        hold_joined_time_ = lead_rejoin_distance_ < kRejoinDistance ? hold_joined_time_ + dt : 0.0;
    if (lead_age_ > kLeaderTimeout) {  // no followers reported any more
        lead_extent_ = 0.0;
        lead_rejoin_distance_ = 0.0;
        lead_follower_speed_ = kUnknown;
    }
    if (avoid_sense_ != 0) {
        avoid_clear_time_ = conflict_now_ ? 0.0 : avoid_clear_time_ + dt;
        if (avoid_clear_time_ > kAvoidClearTime || on_ground_ || mode_ == FlightMode::Takeoff ||
            mode_ == FlightMode::Land)
            avoid_sense_ = 0;
    }
    conflict_now_ = false;  // until the next traffic update

    SlotReference slot_ref{};
    if (mode_ == FlightMode::Formation) {
        leader_age_ += dt;
        if (leader_age_ > kLeaderTimeout) {
            cruise();
        } else if (leader_.valid) {
            slot_ref = leader_tracker_.update(leader_, slot_, dt);
        }
    }

    const int substeps = std::max(1, static_cast<int>(std::ceil(dt / kMaxSubstep)));
    const double h = dt / substeps;
    AtmosphereSample atm{};
    for (int i = 0; i < substeps; ++i) {
        atm = isa(position_.z, env.temperature_offset);
        // Extrapolate the slot inside the frame (the leader is only known at frame start).
        frame_time_ = h * i;
        SlotReference ref = slot_ref;
        ref.position += ref.velocity * frame_time_;
        const Command cmd = guidance(atm, env.wind, ref);
        const Control c = autopilot(cmd, atm);
        integrate(c, atm, env.wind, h);
        burn_fuel(tas_, atm, h);
    }
    update_output(atm, env.wind);
    motion_.update(out_, dt);
}

FixedWingAgent::Command FixedWingAgent::guidance(const AtmosphereSample& atm, const Vec3& wind,
                                                 const SlotReference& slot_ref) {
    Command cmd;
    cmd.heading = course_target_;
    cmd.speed = speed_target_;
    cmd.bank_limit = p_.bank_max;

    const Vec2 air_h = heading_vector(heading_) * (tas_ * std::cos(gamma_));
    const Vec2 vg = on_ground_ ? air_h : air_h + wind.xy();
    const double gs = std::max(vg.length(), 1.0);
    const double lookahead = gs * p_.altitude_time_constant;

    if (on_ground_ && mode_ != FlightMode::Takeoff && mode_ != FlightMode::Taxi) {
        // Landing roll, parked or crashed: idle and brake (on the centerline after landing).
        cmd.heading = mode_ == FlightMode::Land ? runway_.heading : heading_;
        cmd.altitude = position_.z;
        cmd.speed = 0.0;
        cmd.bank_limit = 0.0;
        if (mode_ == FlightMode::Land && taxi_.active()) {
            // Landing roll to the stop point: braking, then on at taxi speed.
            const TaxiPath::Steering st =
                taxi_.update(position_.xy(), heading_, tas_, taxi_settings());
            cmd.heading = st.heading;
            cmd.taxi = true;
            cmd.taxi_accel = st.acceleration;
            cmd.taxi_max_brake = p_.brake_deceleration;
        }
        return cmd;
    }

    if ((mode_ == FlightMode::GoTo || mode_ == FlightMode::Route) && route_.active()) {
        guide_route(cmd, vg, lookahead);  // may finish the route and switch mode
        cmd.climb_mode = true;
    }
    switch (mode_) {
        case FlightMode::Cruise:
            cmd.use_accel = true;
            cmd.lateral_accel = l1_line(course_anchor_, heading_vector(course_target_),
                                        position_.xy(), vg, l1_settings(gs));
            cmd.altitude = altitude_target_now(lookahead);
            cmd.climb_mode = true;
            break;
        case FlightMode::Loiter:
            cmd.use_accel = true;
            cmd.lateral_accel = l1_loiter(loiter_center_.xy(), loiter_radius_, loiter_clockwise_,
                                          position_.xy(), vg, l1_settings(gs));
            cmd.altitude = altitude_target_now(lookahead);
            cmd.climb_mode = true;
            break;
        case FlightMode::Takeoff:
            guide_takeoff(cmd, atm);
            break;
        case FlightMode::Taxi:
            guide_taxi(cmd);
            break;
        case FlightMode::Land:
            guide_landing(cmd, atm, vg);
            break;
        case FlightMode::Formation:
            if (leader_.valid) {
                guide_formation(cmd, vg, slot_ref);
            } else {  // waiting for the first leader update: keep flying straight
                cmd.use_accel = true;
                cmd.lateral_accel = l1_line(course_anchor_, heading_vector(course_target_),
                                            position_.xy(), vg, l1_settings(gs));
                cmd.altitude = altitude_target_now(lookahead);
            }
            break;
        default:
            break;
    }
    const bool own_speed = mode_ == FlightMode::Takeoff || mode_ == FlightMode::Formation ||
                           mode_ == FlightMode::Land || mode_ == FlightMode::Taxi;
    if (!own_speed) cmd.speed = speed_target_;
    // Leading followers still far away: orbit here until they have joined.
    update_lead_hold(atm);
    if (lead_hold_) {
        cmd.use_accel = true;
        cmd.lateral_accel = l1_loiter(hold_center_, hold_radius_, true, position_.xy(), vg,
                                      l1_settings(gs));
    }
    resolved_altitude_target_ = cmd.altitude;
    // Collision avoidance overrides the task's altitude until the conflict is clear.
    if (avoid_sense_ > 0) cmd.altitude = std::max(cmd.altitude, avoid_altitude_);
    if (avoid_sense_ < 0) cmd.altitude = std::min(cmd.altitude, avoid_altitude_);

    if (!on_ground_) {
        // Terrain floor from the predicted terrain ahead.
        if (terrain_.valid() && mode_ != FlightMode::Takeoff && cmd.terrain_floor) {
            const double floor = terrain_.predicted_max(gs * p_.terrain_lookahead) +
                                 p_.min_terrain_clearance;
            cmd.altitude = std::max(cmd.altitude, floor);
            cmd.terrain_alert = position_.z < floor;
        }
        if (position_.z - p_.gear_height - ground_elevation() < kLowBankHeight)
            cmd.bank_limit = std::min(cmd.bank_limit, kLowBankLimit);
    }
    return cmd;
}

void FixedWingAgent::guide_route(Command& cmd, const Vec2& vg, double lookahead) {
    const double gs = std::max(vg.length(), 1.0);
    const Vec2 pos = position_.xy();
    LegProgress lp = route_.progress(pos);

    const Waypoint& wp = route_.target();
    const Waypoint* next = route_.next();
    double switch_distance = 0.0;
    if (next != nullptr && !wp.fly_over) {
        // Fly-by: start the turn early so the turn is tangent to both legs.
        const Vec2 next_leg = next->position.xy() - wp.position.xy();
        const double turn =
            std::min(std::abs(angle_right(lp.direction, next_leg.normalized())), deg2rad(150.0));
        switch_distance = std::min({turn_radius(gs) * std::tan(turn / 2.0), 0.5 * lp.length,
                                    0.5 * next_leg.length()});
    }
    if (lp.remaining() <= switch_distance) {
        const EndBehavior end = route_.route().on_end;
        if (!route_.advance(position_, resolved_altitude_target_)) {
            finish_route(end);
            return;
        }
        lp = route_.progress(pos);
    }

    const Waypoint& target = route_.target();
    set_speed(target.speed, target.speed_ref);
    cmd.use_accel = true;
    cmd.lateral_accel = l1_leg(route_.leg_start(), route_.leg_end(), pos, vg, l1_settings(gs));
    cmd.altitude = route_.altitude_target(lp, terrain_, lookahead, resolved_altitude_target_);
}

void FixedWingAgent::finish_route(EndBehavior behavior) {
    const Waypoint last = route_.target();
    const double last_altitude =
        RouteTracker::resolve_altitude(last, terrain_, 0.0, resolved_altitude_target_);
    const bool agl = last.altitude_ref == AltitudeRef::AboveTerrain;
    const AltitudeRef ref = agl ? AltitudeRef::AboveTerrain : AltitudeRef::Absolute;
    const double altitude = agl ? last.position.z : last_altitude;
    const double radius = route_.route().loiter_radius;
    const std::optional<Runway> runway = route_.route().runway;
    const Vec2 leg = route_.leg_end() - route_.leg_start();

    if (behavior == EndBehavior::ContinueStraight) {
        // Continue on the extension of the last leg.
        const double course = leg.length() > 1.0 ? heading_of(leg) : heading_;
        hold_course(course, last.position.xy(), altitude, kUnknown, ref);
        return;
    }
    if (behavior == EndBehavior::Land && runway) {
        land(*runway);
        return;
    }
    // Without a runway a fixed-wing aircraft orbits the last waypoint.
    loiter({last.position.x, last.position.y, altitude}, radius, true, ref);
}

void FixedWingAgent::guide_takeoff(Command& cmd, const AtmosphereSample& atm) {
    const double climb_height =
        takeoff_.climb_height > 0.0 ? takeoff_.climb_height : p_.takeoff_climb_height;
    cmd.heading = takeoff_heading_;
    cmd.altitude = runway_elevation_ + climb_height;
    const double vs_flaps = stall_speed(atm.density, true);
    const double vs_clean = stall_speed(atm.density, false);

    if (on_ground_) {
        cmd.bank_limit = 0.0;
        if (lining_up_) {
            const TaxiPath::Steering st =
                taxi_.update(position_.xy(), heading_, tas_, taxi_settings());
            phase_ = st.last_segment ? FlightPhase::LineUp : FlightPhase::Taxi;
            const bool lined_up = st.last_segment && std::abs(st.cross_track) <= kLineUpOffset &&
                                  std::abs(wrap_pi(heading_ - takeoff_heading_)) <= kLineUpHeading;
            if (!lined_up) {
                cmd.heading = st.heading;
                cmd.taxi = true;
                cmd.taxi_accel = st.acceleration;
                return;
            }
            lining_up_ = false;
            taxi_.clear();
            runway_elevation_ = ground_elevation();
        }
        // Takeoff run along the centerline.
        const Vec2 dir = heading_vector(takeoff_heading_);
        const double along = dot(position_.xy() - takeoff_origin_, dir);
        const Vec2 aim = takeoff_origin_ + dir * (along + std::max(kRollLookahead, 3.0 * tas_));
        cmd.heading = heading_of(aim - position_.xy());
        if (!takeoff_released_) {
            // Lined up and waiting for the release: brakes on.
            phase_ = FlightPhase::LineUp;
            cmd.taxi = true;
            cmd.taxi_accel = -tas_ / kHoldStopTime;
            return;
        }
        cmd.full_thrust = true;
        cmd.rotate = tas_ >= rotate_speed(atm.density);
        phase_ = cmd.rotate ? FlightPhase::Rotation : FlightPhase::TakeoffRoll;
        return;
    }

    phase_ = FlightPhase::InitialClimb;
    cmd.use_accel = true;
    cmd.lateral_accel = l1_line(takeoff_origin_, heading_vector(cmd.heading), position_.xy(),
                                heading_vector(heading_) * tas_, l1_settings(tas_));
    cmd.bank_limit = std::min(p_.bank_max, kTakeoffBankLimit);
    const double height = position_.z - p_.gear_height - runway_elevation_;
    if (gear_down_ && p_.retractable_gear && height > p_.gear_up_height) gear_down_ = false;
    // Retract flaps once high enough and fast enough to fly clean.
    if (flaps_ && height > p_.flaps_up_height && tas_ > 1.05 * kStallMargin * vs_clean)
        flaps_ = false;
    if (!flaps_) {
        cmd.speed = speed_target_;
    } else {
        cmd.speed = height > p_.flaps_up_height ? 1.5 * vs_clean : 1.25 * vs_flaps;
    }

    if (height >= climb_height - 15.0) {
        // Never back down when placed above the climb height (relocation).
        finish_takeoff(std::max(runway_elevation_ + climb_height, position_.z));
    }
}

void FixedWingAgent::guide_taxi(Command& cmd) {
    cmd.altitude = position_.z;
    cmd.bank_limit = 0.0;
    cmd.heading = heading_;
    const TaxiPath::Steering st = taxi_.update(position_.xy(), heading_, tas_, taxi_settings());
    if (!taxi_.active() || (st.remaining < kTaxiStopDistance && tas_ < kTaxiStopSpeed)) {
        // Arrived: parked, brakes set.
        taxi_.clear();
        mode_ = FlightMode::Cruise;
        course_target_ = heading_;
        phase_ = FlightPhase::Parked;
        return;
    }
    phase_ = FlightPhase::Taxi;
    cmd.heading = st.heading;
    cmd.taxi = true;
    cmd.taxi_accel = st.acceleration;
}

TaxiPath::Settings FixedWingAgent::taxi_settings() const {
    TaxiPath::Settings s;
    s.speed = std::max(p_.taxi_speed, 0.5);
    s.turn_radius = std::max(p_.taxi_turn_radius, 1.0);
    s.turn_speed = std::min(s.speed, std::sqrt(kTaxiTurnAccel * s.turn_radius));
    s.deceleration = kTaxiDeceleration;
    return s;
}

void FixedWingAgent::finish_takeoff(double altitude) {
    phase_ = FlightPhase::Airborne;
    flaps_ = false;
    resolved_altitude_target_ = altitude;  // the route's first leg starts from here
    if (takeoff_.then) {
        Route r = std::move(*takeoff_.then);
        takeoff_.then.reset();
        follow_route(std::move(r));
    } else {
        hold_course(takeoff_heading_, takeoff_origin_, altitude, kUnknown,
                    AltitudeRef::Absolute);
    }
}

void FixedWingAgent::guide_landing(Command& cmd, const AtmosphereSample& atm, const Vec2& vg) {
    const Vec2 dir = heading_vector(runway_.heading);
    const Vec3 aim = aim_point();
    const Vec2 pos = position_.xy();
    const double along = dot(pos - aim.xy(), dir);  // negative before the aim point
    const double crosstrack = dot(pos - aim.xy(), right_of(dir));
    const double gs = std::max(vg.length(), 1.0);
    const double v_app = approach_speed(atm.density);
    const double tan_gs = std::tan(p_.glide_slope);
    const double faf_distance = p_.final_approach_distance;
    const double faf_altitude = aim.z + faf_distance * tan_gs;
    const double glide_altitude = aim.z + std::max(-along, 0.0) * tan_gs;
    const double wheel_height = position_.z - p_.gear_height - runway_.start.z;
    const L1Settings l1 = l1_settings(gs);
    // Speed while positioning: between approach speed and a moderate clean speed.
    const double positioning_speed =
        std::max(1.3 * v_app, std::min(speed_target_, 1.8 * stall_speed(atm.density, false)));
    // Pilots pull more firmly in the flare than in normal flight path changes.
    const double flare_gamma_tau = std::min(p_.gamma_time_constant, 1.0);

    cmd.heading = runway_.heading;
    switch (landing_stage_) {
        case LandingStage::Initial:
        case LandingStage::Break:
        case LandingStage::Downwind:
        case LandingStage::Base:
            phase_ = FlightPhase::Approach;
            guide_pattern(cmd, atm, vg);
            break;
        case LandingStage::Positioning: {
            phase_ = FlightPhase::Approach;
            if (landing_.pattern == LandingPattern::Overhead) {
                guide_pattern(cmd, atm, vg);
                break;
            }
            // Entry point on the extended centerline, far enough out to turn onto final.
            const double radius = turn_radius(positioning_speed);
            const Vec2 entry = aim.xy() - dir * (faf_distance + 2.0 * radius);
            cmd.use_accel = true;
            cmd.lateral_accel = l1_point(entry, pos, vg, l1);
            cmd.altitude = faf_altitude;
            cmd.speed = positioning_speed;
            const bool aligned = along < -0.8 * faf_distance &&
                                 std::abs(wrap_pi(heading_of(vg) - runway_.heading)) < deg2rad(60.0) &&
                                 std::abs(crosstrack) < 0.5 * -along;
            if (aligned || (pos - entry).length() < radius) landing_stage_ = LandingStage::Final;
            break;
        }
        case LandingStage::Final: {
            phase_ = FlightPhase::Approach;
            cmd.use_accel = true;
            cmd.lateral_accel = l1_line(aim.xy(), dir, pos, vg, l1);
            cmd.terrain_floor = false;  // the glide path leads down to the runway
            // Level at the final approach fix altitude until the glide path is reached
            // from below, then follow it with a vertical speed feedforward.
            cmd.altitude = std::min(glide_altitude, faf_altitude);
            if (glide_altitude <= faf_altitude)
                cmd.vertical_speed_ff = -std::max(dot(vg, dir), 0.0) * tan_gs;
            if (-along < 1.3 * faf_distance) {
                flaps_ = true;
                gear_down_ = true;
                cmd.speed = v_app;
            } else {
                cmd.speed = positioning_speed;
            }
            // Stability check below the decision height, and never pass the aim point high.
            if (wheel_height < p_.decision_height) {
                const double tolerance = std::max(5.0, 0.5 * p_.decision_height);
                const bool unstable =
                    std::abs(crosstrack) > tolerance ||
                    std::abs(position_.z - p_.gear_height - glide_altitude) > tolerance ||
                    std::abs(tas_ - v_app) > 0.15 * v_app ||
                    std::abs(wrap_pi(heading_of(vg) - runway_.heading)) > deg2rad(15.0);
                if (unstable) {
                    landing_stage_ = LandingStage::GoAround;
                    break;
                }
            }
            // Start the flare early by the height lost while the flight path responds.
            const double sink = std::max(-out_.vertical_speed, p_.touchdown_sink_rate);
            const double flare_start = p_.flare_height + sink * flare_gamma_tau;
            if (wheel_height < flare_start) {
                landing_stage_ = LandingStage::Flare;
                flare_height_ = flare_start;
                flare_sink_ = sink;
            } else if (along > 0.0) {
                landing_stage_ = LandingStage::GoAround;  // passed the aim point too high
            }
            break;
        }
        case LandingStage::Flare: {
            phase_ = FlightPhase::Flare;
            cmd.use_accel = true;
            cmd.lateral_accel = l1_line(aim.xy(), dir, pos, vg, l1);
            cmd.bank_limit = deg2rad(5.0);
            cmd.terrain_floor = false;
            cmd.idle_thrust = true;
            cmd.gamma_time_constant = flare_gamma_tau;
            // Constant vertical deceleration from the flare-start sink rate to the touchdown
            // sink rate, on the height expected once the flight path has responded.
            const double predicted_height =
                std::max(wheel_height + out_.vertical_speed * flare_gamma_tau, 0.0);
            const double s0 = flare_sink_;
            const double s1 = p_.touchdown_sink_rate;
            const double fraction = std::clamp(predicted_height / flare_height_, 0.0, 1.0);
            cmd.altitude = position_.z;
            cmd.vertical_speed_ff = -std::sqrt(s1 * s1 + (s0 * s0 - s1 * s1) * fraction);
            cmd.speed = v_app;
            break;
        }
        case LandingStage::GoAround: {
            phase_ = FlightPhase::GoAround;
            cmd.use_accel = true;
            cmd.lateral_accel = l1_line(aim.xy(), dir, pos, vg, l1);
            const double height = position_.z - p_.gear_height - runway_.start.z;
            const bool overhead = landing_.pattern == LandingPattern::Overhead;
            cmd.altitude =
                runway_.start.z + (overhead ? p_.pattern_height : p_.takeoff_climb_height);
            cmd.speed = 1.2 * v_app;
            cmd.full_thrust = height < p_.flaps_up_height;
            if (p_.retractable_gear && out_.vertical_speed > 1.0) gear_down_ = false;
            if (flaps_ && height > p_.flaps_up_height &&
                tas_ > 1.05 * kStallMargin * stall_speed(atm.density, false))
                flaps_ = false;
            if (!flaps_) cmd.speed = positioning_speed;
            if (position_.z > cmd.altitude - 30.0) {
                // Overhead: on along the runway to the break again (a closed pattern).
                landing_stage_ = overhead ? LandingStage::Initial : LandingStage::Positioning;
                initial_offset_ = dot(pos - runway_.start.xy(), right_of(dir));
            }
            break;
        }
        case LandingStage::Rollout:
            break;  // handled on the ground
    }
}

// Form up first, then go: a leader whose followers are far from their slots orbits where it
// is until they have joined, then carries on with its task. Not when they cannot even fly
// its minimum speed (waiting would not help), nor on the ground or in a takeoff / landing.
void FixedWingAgent::update_lead_hold(const AtmosphereSample& atm) {
    const double v_min = kStallMargin * stall_speed(atm.density, false);
    const bool can_wait = !on_ground_ &&
                          (mode_ == FlightMode::Cruise || mode_ == FlightMode::GoTo ||
                           mode_ == FlightMode::Route) &&
                          std::isfinite(lead_follower_speed_) && lead_follower_speed_ >= v_min;
    if (!can_wait) {
        lead_hold_ = false;
        return;
    }
    if (lead_hold_) {
        // Joined and settled in their slots: carry on.
        if (hold_joined_time_ > kHoldSettleTime) lead_hold_ = false;
        return;
    }
    if (lead_rejoin_distance_ < kHoldDistance) return;
    hold_joined_time_ = 0.0;
    lead_hold_ = true;
    // Sized for the speed flown while holding, with a bank margin so the orbit is flown
    // as planned.
    const double v = std::max(1.2 * v_min, kRejoinSpeedShare * cruise_speed_at(atm));
    hold_radius_ = 1.1 * v * v / (kGravity * std::tan(0.8 * std::min(p_.bank_max, kLoiterBank)));
    hold_center_ = position_.xy() + right_of(heading_vector(heading_)) * hold_radius_;
}

// Overhead pattern: initial leg along the runway at pattern height, a 180 deg break turn,
// downwind, and a descending base turn onto a short final flown like a straight-in final.
// Aircraft breaking later fly a longer downwind and land correspondingly later.
void FixedWingAgent::guide_pattern(Command& cmd, const AtmosphereSample& atm, const Vec2& vg) {
    const Vec2 dir = heading_vector(runway_.heading);
    const Vec2 right = right_of(dir);
    const Vec2 start = runway_.start.xy();
    const Vec2 pos = position_.xy();
    const double along = dot(pos - start, dir);  // m past the runway start
    const double gs = std::max(vg.length(), 1.0);
    const L1Settings l1 = l1_settings(gs);
    const double side = landing_.left_pattern ? -1.0 : 1.0;  // downwind side, right = +1
    const double v_app = approach_speed(atm.density);
    const double v_initial = std::isfinite(landing_.initial_speed)
                                 ? landing_.initial_speed
                                 : kInitialSpeedFactor * v_app;
    const double v_downwind = kDownwindSpeedFactor * v_app;
    const double pattern_altitude = runway_.start.z + p_.pattern_height;
    const double final_length = kShortFinalTime * v_app;  // m before the aim point
    const double base_along = p_.touchdown_distance - final_length;
    // Descent from abeam the aim point on the downwind (the perch) through the base turn
    // to the glide path height at the start of the final.
    const double base_radius = 0.5 * std::abs(downwind_offset_);
    const double final_altitude = runway_.start.z + final_length * std::tan(p_.glide_slope);
    const double descent_path = final_length + kPi * base_radius;
    const auto descend = [&](double flown) {
        const double drop = pattern_altitude - final_altitude;
        cmd.altitude = pattern_altitude - drop * std::clamp(flown / descent_path, 0.0, 1.0);
        if (flown > 0.0 && flown < descent_path) cmd.vertical_speed_ff = -drop * gs / descent_path;
    };
    cmd.use_accel = true;
    cmd.altitude = pattern_altitude;
    switch (landing_stage_) {
        case LandingStage::Positioning: {
            // To the initial point on the extended centerline.
            const Vec3 aim = aim_point();
            const double radius = turn_radius(v_initial);
            const Vec2 ip = aim.xy() - dir * (p_.final_approach_distance + 2.0 * radius);
            cmd.lateral_accel = l1_point(ip, pos, vg, l1);
            cmd.speed = v_initial;
            // Joining the extended centerline between the initial point and the runway.
            const double to_aim = dot(pos - aim.xy(), dir);
            const bool aligned = to_aim < -0.8 * p_.final_approach_distance &&
                                 to_aim > -p_.final_approach_distance - 3.0 * radius &&
                                 std::abs(wrap_pi(heading_of(vg) - runway_.heading)) < deg2rad(30.0) &&
                                 std::abs(dot(pos - start, right)) < 0.5 * radius;
            if (aligned || (pos - ip).length() < radius) {
                landing_stage_ = LandingStage::Initial;
                initial_offset_ =
                    std::isfinite(landing_.initial_offset) ? landing_.initial_offset : 0.0;
            }
            break;
        }
        case LandingStage::Initial:
            cmd.lateral_accel = l1_line(start + right * initial_offset_, dir, pos, vg, l1);
            cmd.speed = v_initial;
            if (along >= landing_.break_distance) {
                landing_stage_ = LandingStage::Break;
                // Downwind two break radii out, at least wide enough for the base turn.
                const double v_base = kBaseSpeedFactor * v_app;
                const double base_bank = std::min(p_.bank_max, kBaseBank);
                const double min_offset = 2.0 * v_base * v_base / (kGravity * std::tan(base_bank));
                downwind_offset_ = initial_offset_ +
                                   side * 2.0 * break_radius(0.5 * (v_initial + v_downwind));
                if (side * downwind_offset_ < min_offset) downwind_offset_ = side * min_offset;
            }
            break;
        case LandingStage::Break: {
            // Hard level turn to the downwind, slowing down at idle (speed brake).
            cmd.lateral_accel = side * kGravity * std::tan(break_bank(p_));
            cmd.speed = v_downwind;
            if (std::abs(wrap_pi(heading_of(vg) - runway_.heading - kPi)) < deg2rad(30.0))
                landing_stage_ = LandingStage::Downwind;
            break;
        }
        case LandingStage::Downwind:
            cmd.lateral_accel = l1_line(start + right * downwind_offset_, -dir, pos, vg, l1);
            cmd.speed = v_downwind;
            flaps_ = true;
            gear_down_ = true;
            descend(p_.touchdown_distance - along);
            if (along <= base_along) landing_stage_ = LandingStage::Base;
            break;
        case LandingStage::Base: {
            // Half circle from the downwind onto the runway line.
            const Vec2 center = start + dir * base_along + right * (0.5 * downwind_offset_);
            cmd.lateral_accel = l1_loiter(center, base_radius, side > 0.0, pos, vg, l1);
            cmd.speed = kBaseSpeedFactor * v_app;
            cmd.terrain_floor = false;
            const double heading_error = std::abs(wrap_pi(heading_of(vg) - runway_.heading));
            const double turned = std::clamp(1.0 - heading_error / kPi, 0.0, 1.0);
            descend(final_length + turned * kPi * base_radius);
            if (heading_error < deg2rad(20.0) || along > base_along + 0.5 * final_length)
                landing_stage_ = LandingStage::Final;
            break;
        }
        default:
            break;
    }
}

double FixedWingAgent::overhead_initial_speed(const Runway& runway) const {
    return kInitialSpeedFactor * approach_speed(isa(runway.start.z + p_.pattern_height).density);
}

double FixedWingAgent::break_spacing(const Runway& runway, double interval,
                                     double initial_speed) const {
    // Breaking d further on costs d on the initial leg and d again on the downwind.
    const double v_downwind =
        kDownwindSpeedFactor * approach_speed(isa(runway.start.z + p_.pattern_height).density);
    return interval / (1.0 / initial_speed + 1.0 / v_downwind);
}

double FixedWingAgent::break_radius(double speed) const {
    return speed * speed / (kGravity * std::tan(break_bank(p_)));
}

void FixedWingAgent::guide_formation(Command& cmd, const Vec2& vg, const SlotReference& ref) {
    const Vec2 pos = position_.xy();
    const Vec2 dir = heading_vector(ref.heading);
    const Vec2 err = ref.position.xy() - pos;
    const double along = dot(err, dir);
    const double gs = std::max(vg.length(), 1.0);
    const double join_delta = 0.15 * p_.cruise_speed;
    const double wind_offset = tas_ - gs;  // rough airspeed/groundspeed difference
    const L1Settings l1 = l1_settings(gs);
    // Station keeping close to the slot, rejoin beyond: at most 2 km, and within a third of
    // the leader's turn radius while it keeps turning (tracking the slot line from further
    // away only flies a parallel circle).
    double join_distance = std::min(3.0 * l1_distance(l1, gs), kMaxJoinDistance);
    const double slot_speed = ref.velocity.xy().length();
    if (std::abs(ref.turn_rate) > kTurningRejoinRate)
        join_distance = std::min(join_distance, 0.3 * slot_speed / std::abs(ref.turn_rate));
    join_distance = std::max(join_distance, 10.0 * p_.formation_min_separation);

    cmd.use_accel = true;
    cmd.altitude = ref.position.z;
    if (err.length() > join_distance) {
        // Rejoin, faster than the slot when behind it, slower when ahead.
        double overtake = kRejoinClosing * cruise_speed_at(isa(position_.z, temperature_offset_));
        if (std::isfinite(formation_speed_limit_))
            overtake = std::min(overtake, std::max(formation_speed_limit_ - slot_speed, join_delta));
        overtake = std::min(overtake, std::max(err.length() / kRejoinTaper, join_delta));
        bool behind = along >= 0.0;
        if (std::abs(ref.turn_rate) > kTurningRejoinRate && slot_speed > 1.0) {
            // The leader keeps turning (e.g. holding for the flight): fly the slot's circle
            // and catch up along it. Chasing a point on the circle would only orbit outside.
            const Vec2 v_dir = ref.velocity.xy() / slot_speed;
            const bool right = ref.turn_rate > 0.0;
            const double radius = slot_speed / std::abs(ref.turn_rate);
            const Vec2 center = ref.position.xy() + (right ? right_of(v_dir) : -right_of(v_dir)) * radius;
            cmd.lateral_accel = l1_loiter(center, radius, right, pos, vg, l1);
            // Behind when the slot is less than half a circle ahead in the turn direction.
            const double ahead = cross(pos - center, ref.position.xy() - center);  // > 0: CCW
            behind = right ? ahead < 0.0 : ahead > 0.0;
        } else {
            // Head for where the slot will be.
            const double t_go = std::clamp(err.length() / gs, 0.0, 60.0);
            cmd.lateral_accel = l1_point(ref.position.xy() + ref.velocity.xy() * t_go, pos, vg, l1);
        }
        cmd.speed = slot_speed + wind_offset + (behind ? overtake : -overtake);
    } else {
        // Station keeping: track the slot line, anticipate the leader's turn and match
        // its along-track speed.
        const L1Settings tight{p_.l1_period * 0.6, p_.l1_damping, 0.0};
        cmd.lateral_accel = l1_line(ref.position.xy(), dir, pos, vg, tight) + ref.turn_rate * gs;
        // Along-track speed of the slot plus a bounded closing speed, as an absolute target
        // (relative to the own speed it would keep accelerating while far behind or while
        // flying across the slot's direction in a turn).
        const double closing = std::clamp(kFormationAlongGain * along, -join_delta, join_delta);
        cmd.speed = dot(ref.velocity.xy(), dir) + closing + wind_offset;
        cmd.speed_time_constant = std::min(p_.speed_time_constant, kFormationSpeedTau);
    }

    // Separation: never get closer to the leader than the minimum distance.
    const double min_sep =
        slot_.min_separation > 0.0 ? slot_.min_separation : p_.formation_min_separation;
    const Vec3 to_leader = leader_.position + leader_.velocity * frame_time_ - position_;
    if (to_leader.length() < min_sep) {
        const double side = slot_.offset.z > 0.0 ? 1.0 : -1.0;
        cmd.altitude = leader_.position.z + side * min_sep;
        if (dot(to_leader.xy(), heading_vector(heading_)) > 0.0)
            cmd.speed = std::min(cmd.speed, tas_ - 0.5 * join_delta);
    }
}

FixedWingAgent::Control FixedWingAgent::autopilot(const Command& cmd, const AtmosphereSample& atm) {
    Control c;
    c.rotate = cmd.rotate;
    c.gamma_time_constant = cmd.gamma_time_constant;
    if (on_ground_) {
        c.throttle = cmd.full_thrust ? 1.0 : p_.engine.idle_throttle;
        c.steer_heading = cmd.heading;
        if (cmd.taxi) {
            taxi_controls(cmd.taxi_accel,
                          cmd.taxi_max_brake > 0.0 ? cmd.taxi_max_brake : kTaxiMaxBrake, atm, c);
        }
        return c;
    }

    const double g = kGravity;
    const double m = mass_;
    const double v = std::max(tas_, kMinAirspeed);

    // Speed envelope
    const double v_min =
        (flaps_ ? kFlapsStallMargin : kStallMargin) * stall_speed(atm.density, flaps_);
    const double v_max = std::max(v_min, std::min(p_.max_speed / std::sqrt(atm.sigma),
                                                  p_.mach_max * atm.speed_of_sound));
    double v_cmd = std::clamp(cmd.speed, v_min, v_max);
    // Leading a formation: keep a speed margin the followers can use to catch up.
    if (lead_extent_ > 0.0)
        v_cmd = std::min(v_cmd, std::max(v_min, kLeadSpeedShare * sustained_speed(atm, v_min, v_max)));
    // Followers rejoining: slow down so they catch up (form up first, then go). Never
    // faster than the slowest follower can follow, but never below the own minimum speed
    // either: a follower too slow for that falls behind.
    const bool rejoining = lead_rejoin_distance_ > kRejoinDistance;
    if (rejoining) v_cmd = std::min(v_cmd, std::max(v_min, kRejoinSpeedShare * cruise_speed_at(atm)));
    if (std::isfinite(lead_follower_speed_)) {
        const double share = rejoining ? kRejoinFollowerShare : kLeadFollowerShare;
        v_cmd = std::min(v_cmd, std::max(v_min, share * lead_follower_speed_));
    }
    // As a follower: what can be held in formation (for the leader to stay below).
    formation_speed_limit_ = mode_ == FlightMode::Formation
                                 ? kFollowerSpeedShare * sustained_speed(atm, v_min, v_max)
                                 : kUnknown;
    // Below the terrain floor: slow down towards the best climb angle speed and put the
    // energy into climbing, like a pilot clearing rising ground.
    if (cmd.terrain_alert) v_cmd = std::min(v_cmd, std::max(v_min, min_drag_speed(atm.density)));

    // Climb mode (like a flight level change): a large climb that the commanded speed
    // cannot sustain at the wanted rate is flown at full power at the climb speed, back to
    // the commanded speed near the target altitude.
    const double altitude_error = cmd.altitude - position_.z;
    if (!cmd.climb_mode || p_.climb_speed < 0.0 || altitude_error < kClimbModeExit) {
        climbing_ = false;
    } else if (!climbing_ && altitude_error > kClimbModeEnter) {
        const double wanted = std::min(
            p_.climb_rate_max, cmd.vertical_speed_ff + altitude_error / p_.altitude_time_constant);
        climbing_ = climb_rate_at(v_cmd, atm) < 0.5 * wanted;
    }
    if (climbing_) v_cmd = std::min(v_cmd, climb_speed(atm, v_min, v_max));

    // Lateral: bank from lateral acceleration or heading error, limited by stall margin.
    const double qs = 0.5 * atm.density * v * v * p_.wing_area;
    const double n_stall = qs * cl_max() / (m * g);
    const double n_turn = std::min(p_.load_factor_max, n_stall / kTurnStallMargin);
    double bank_limit =
        std::min(cmd.bank_limit, n_turn > 1.0 ? std::acos(1.0 / n_turn) : deg2rad(5.0));
    if (lead_extent_ > 0.0) bank_limit = std::min(bank_limit, lead_bank_limit(atm, v_min, v_max));
    double bank_cmd = 0.0;
    if (cmd.use_accel) {
        bank_cmd = std::atan(cmd.lateral_accel / g);
    } else {
        const double turn_rate = wrap_pi(cmd.heading - heading_) / p_.heading_time_constant;
        bank_cmd = std::atan(v * turn_rate / g);
    }
    c.bank = std::clamp(bank_cmd, -bank_limit, bank_limit);

    const double hdot_cmd = std::clamp(
        cmd.vertical_speed_ff + (cmd.altitude - position_.z) / p_.altitude_time_constant,
        -p_.descent_rate_max, p_.climb_rate_max);
    if (cmd.idle_thrust) {
        // Flare: thrust at idle, the flight path follows the commanded sink rate only.
        c.gamma = std::clamp(std::asin(std::clamp(hdot_cmd / v, -0.9, 0.9)),
                             -p_.descent_angle_max, p_.climb_angle_max);
        c.throttle = p_.engine.idle_throttle;
        return c;
    }

    // Energy management: thrust controls total energy, flight path splits it between
    // altitude and speed (TECS-like, solved with the model inverse).
    const bool has_brake = p_.speedbrake_dcd0 > 0.0;
    const double n_now = std::cos(gamma_) / std::max(std::cos(bank_), 0.2);
    const double t_max = thrust_available(v, atm);
    // Engine out (no fuel): glide at the best glide speed instead of holding the altitude.
    const bool engine_out = t_max <= 0.0;
    if (engine_out) v_cmd = std::clamp(min_drag_speed(atm.density), v_min, v_max);
    const double t_idle = p_.engine.idle_throttle * t_max;
    const double excess = (t_max - drag(v, n_now, atm, false)) / m;
    const double idle_accel = (t_idle - drag(v, n_now, atm, has_brake)) / m;

    // Half of the excess thrust goes to acceleration when climbing and accelerating at once,
    // all of it in level flight or below the minimum speed, and none when the terrain
    // floor is violated.
    double accel_cap = (hdot_cmd > 1.0 ? 0.5 : 1.0) * std::max(excess, 0.0);
    if (v < v_min) accel_cap = std::max(excess, 0.0);
    else if (cmd.terrain_alert) accel_cap = 0.0;
    const double decel_cap = std::max(-idle_accel, 0.0);
    const double tau_v = cmd.speed_time_constant > 0.0 ? cmd.speed_time_constant
                                                       : p_.speed_time_constant;
    const double vdot_cmd = std::clamp((v_cmd - v) / tau_v, -decel_cap, accel_cap);

    double gamma_cmd = std::asin(std::clamp(hdot_cmd / v, -0.9, 0.9));
    gamma_cmd = std::clamp(gamma_cmd, -p_.descent_angle_max, p_.climb_angle_max);

    // Flight path limited by the energy available at full and idle thrust.
    const double gamma_lo =
        std::min(std::asin(std::clamp((idle_accel - vdot_cmd) / g, -0.9, 0.9)), 0.0);
    double gamma_hi = std::asin(std::clamp((excess - vdot_cmd) / g, -0.9, 0.9));
    if (v >= v_min && !engine_out) gamma_hi = std::max(gamma_hi, std::min(gamma_cmd, 0.0));
    gamma_cmd = std::clamp(gamma_cmd, gamma_lo, std::max(gamma_lo, gamma_hi));
    c.gamma = gamma_cmd;

    const double n_cmd = std::cos(gamma_cmd) / std::max(std::cos(c.bank), 0.2);
    const double t_req = drag(v, n_cmd, atm, false) + m * (g * std::sin(gamma_cmd) + vdot_cmd);
    double throttle = t_req / std::max(t_max, 1e-6);
    if (cmd.full_thrust) throttle = 1.0;
    c.speed_brake = has_brake && throttle < p_.engine.idle_throttle && v > v_cmd + 2.0;
    c.throttle = std::clamp(throttle, p_.engine.idle_throttle, 1.0);
    return c;
}

// Thrust and wheel brakes for the commanded acceleration while taxiing.
void FixedWingAgent::taxi_controls(double accel, double max_brake, const AtmosphereSample& atm,
                                   Control& c) const {
    const double m = mass_;
    const double w = m * kGravity;
    const double slope = terrain_.valid() ? std::atan(terrain_.slope()) : 0.0;
    const double resistance = p_.rolling_friction * w * std::cos(slope) + w * std::sin(slope);
    const double a = std::clamp(accel, -max_brake, kTaxiMaxAccel);
    const double t_max = thrust_available(tas_, atm);
    c.taxi = true;
    c.throttle = std::clamp((m * a + resistance) / std::max(t_max, 1e-6), p_.engine.idle_throttle,
                            std::max(kTaxiMaxThrottle, p_.engine.idle_throttle));
    // The brakes take what the engines (idle thrust, spooling down) give too much.
    const double surplus = (throttle_ * t_max - resistance) / m - a;
    c.brake = std::clamp(surplus, 0.0, max_brake);
}

void FixedWingAgent::integrate(const Control& c, const AtmosphereSample& atm, const Vec3& wind,
                               double dt) {
    throttle_ = lag(throttle_, c.throttle, p_.engine.spool_time, dt);
    if (on_ground_) {
        integrate_ground(c, atm, dt);
        return;
    }
    const double g = kGravity;
    const double m = mass_;
    const double w = m * g;
    speed_brake_ = c.speed_brake;

    const double roll_rate = std::clamp((c.bank - bank_) / p_.roll_time_constant,
                                        -p_.roll_rate_max, p_.roll_rate_max);
    bank_ += roll_rate * dt;

    // Load factor needed to follow the commanded flight path, limited by structure/stall.
    const double v = std::max(tas_, kMinAirspeed);
    const double qs = 0.5 * atm.density * v * v * p_.wing_area;
    const double n_hi = std::min(p_.load_factor_max, qs * cl_max() / w);
    const double n_lo = std::min(std::max(p_.load_factor_min, -0.5 * qs * cl_max() / w), n_hi);
    const double cos_bank = std::max(std::cos(bank_), 0.05);
    const double gamma_tau =
        c.gamma_time_constant > 0.0 ? c.gamma_time_constant : p_.gamma_time_constant;
    const double gamma_rate_cmd = (c.gamma - gamma_) / gamma_tau;
    const double n = std::clamp((std::cos(gamma_) + v * gamma_rate_cmd / g) / cos_bank, n_lo, n_hi);
    load_factor_ = n;

    // Point-mass equations of motion in the wind axes.
    const double gamma_dot = g / v * (n * std::cos(bank_) - std::cos(gamma_));
    const double heading_dot = g * n * std::sin(bank_) / (v * std::max(std::cos(gamma_), 0.1));
    alpha_ = (n * w / qs - cl0()) / p_.cl_alpha;
    const double thrust = throttle_ * thrust_available(v, atm);
    const double v_dot = (thrust * std::cos(alpha_) - drag(v, n, atm, speed_brake_)) / m -
                         g * std::sin(gamma_);

    tas_ = std::max(kMinAirspeed, tas_ + v_dot * dt);
    gamma_ = std::clamp(gamma_ + gamma_dot * dt, -deg2rad(85.0), deg2rad(85.0));
    heading_ = wrap_2pi(heading_ + heading_dot * dt);
    const Vec2 dir = heading_vector(heading_);
    const Vec3 air{dir.x * std::cos(gamma_), dir.y * std::cos(gamma_), std::sin(gamma_)};
    position_ += (air * tas_ + wind) * dt;

    const double ground = ground_elevation();
    if (position_.z - p_.gear_height < ground) {
        const bool gentle = tas_ * std::sin(gamma_) > -4.0 && std::abs(bank_) < deg2rad(10.0);
        position_.z = ground + p_.gear_height;
        on_ground_ = true;
        ground_pitch_ = std::max(0.0, gamma_ + alpha_);
        gamma_ = 0.0;
        bank_ = 0.0;
        if (mode_ == FlightMode::Takeoff && gentle) {
            // Wheels touched again right after lift-off (e.g. rising ground): keep rolling.
            phase_ = FlightPhase::Rotation;
            return;
        }
        if (mode_ == FlightMode::Land && gentle) {
            // Touchdown: landing roll on the centerline, spoilers out when available.
            landing_stage_ = LandingStage::Rollout;
            phase_ = FlightPhase::Rollout;
            speed_brake_ = p_.speedbrake_dcd0 > 0.0;
            taxi_.clear();
            if (std::isfinite(landing_.stop_distance)) {
                // Roll on to the stop point, but never stop shorter than braking allows.
                const Vec2 line = heading_vector(runway_.heading);
                const Vec2 start = runway_.start.xy();
                const double along = dot(position_.xy() - start, line);
                const double braking = tas_ * tas_ / (2.0 * std::max(p_.brake_deceleration, 0.1));
                const double stop = std::max(landing_.stop_distance, along + braking);
                taxi_.start({start + line * along, start + line * stop}, true);
            }
            return;
        }
        phase_ = gentle ? FlightPhase::Landed : FlightPhase::TerrainContact;
        mode_ = FlightMode::Cruise;
        route_.clear();
        flaps_ = false;
        speed_brake_ = false;
    }
}

void FixedWingAgent::integrate_ground(const Control& c, const AtmosphereSample& atm, double dt) {
    const double m = mass_;
    const double w = m * kGravity;
    const double v = tas_;

    // Nose wheel steering towards the commanded heading: no tighter than the taxi turn
    // radius (no turning on the spot), gently at speed.
    const double steer_rate =
        std::min(v / std::max(p_.taxi_turn_radius, 1.0),
                 std::max(kGroundLateralAccel / std::max(v, 0.1), kSteerRateAtSpeed));
    heading_ = wrap_2pi(heading_ + std::clamp(wrap_pi(c.steer_heading - heading_),
                                              -steer_rate * dt, steer_rate * dt));
    double rotation_pitch = p_.rotation_pitch;
    if (rotation_pitch <= 0.0) {
        const double cl_liftoff = cl_max() / 1.3225;  // lift-off near 1.15 Vs
        rotation_pitch =
            std::clamp((cl_liftoff - cl0()) / p_.cl_alpha, deg2rad(5.0), deg2rad(12.0));
    }
    ground_pitch_ = approach(ground_pitch_, c.rotate ? rotation_pitch : 0.0, p_.rotation_rate * dt);
    alpha_ = ground_pitch_;

    const double qs = 0.5 * atm.density * v * v * p_.wing_area;
    const double cl = std::min(cl0() + p_.cl_alpha * alpha_, cl_max());
    const double lift = qs * cl;
    double cd = cd0() + p_.induced_drag_k * cl * cl;
    if (speed_brake_) cd += p_.speedbrake_dcd0;
    const double aero_drag = qs * cd;
    // Runway slope along the direction of travel, estimated from the terrain samples.
    const double slope = terrain_.valid() ? std::atan(terrain_.slope()) : 0.0;
    const double normal = std::max(w * std::cos(slope) - lift, 0.0);
    double friction = kWheelBrakeFriction * normal;  // parked: brakes hold the aircraft
    if (c.taxi) {
        friction = p_.rolling_friction * normal + std::min(c.brake * m, 0.4 * normal);
    } else if (mode_ == FlightMode::Takeoff) {
        friction = p_.rolling_friction * normal;
    } else if (mode_ == FlightMode::Land) {
        // Landing roll: brakes as configured, limited by the weight on the wheels.
        friction = std::min(p_.brake_deceleration * m, 0.4 * normal) + p_.rolling_friction * normal;
    }
    const double net = throttle_ * thrust_available(v, atm) - aero_drag - w * std::sin(slope);
    // At rest the aircraft only starts rolling once thrust overcomes friction.
    const double v_dot = (v > 1e-3 ? net - friction : std::max(net - friction, 0.0)) / m;

    tas_ = std::max(0.0, v + v_dot * dt);
    if (mode_ == FlightMode::Land && tas_ < 0.5) {
        // Stopped after landing: parked, ready for another takeoff.
        tas_ = 0.0;
        phase_ = FlightPhase::Landed;
        mode_ = FlightMode::Cruise;
        flaps_ = false;
        speed_brake_ = false;
    }
    const Vec2 step = heading_vector(heading_) * (tas_ * std::cos(slope) * dt);
    position_.x += step.x;
    position_.y += step.y;
    position_.z = ground_elevation() + p_.gear_height;
    load_factor_ = lift / w;
    gamma_ = slope;  // the velocity follows the runway surface; lift-off starts from here
    bank_ = 0.0;

    if (mode_ == FlightMode::Takeoff && lift > w) {
        on_ground_ = false;
        phase_ = FlightPhase::InitialClimb;
    }
}

void FixedWingAgent::update_output(const AtmosphereSample& atm, const Vec3& wind) {
    const Vec2 dir = heading_vector(heading_);
    const Vec3 air =
        Vec3{dir.x * std::cos(gamma_), dir.y * std::cos(gamma_), std::sin(gamma_)} * tas_;
    out_.position = position_;
    out_.velocity = on_ground_ ? air : air + wind;
    out_.attitude.heading = heading_;
    out_.attitude.roll = bank_;
    out_.attitude.pitch = on_ground_ ? gamma_ + ground_pitch_ : gamma_ + alpha_ * std::cos(bank_);
    out_.airspeed = tas_;
    set_air_data(out_, atm);
    out_.mass = mass_;
    out_.fuel = fuel_modelled() ? fuel_ : kUnknown;
    out_.fuel_flow = fuel_flow_;
    out_.ground_speed = out_.velocity.xy().length();
    out_.vertical_speed = out_.velocity.z;
    out_.track = out_.ground_speed > 0.1 ? heading_of(out_.velocity.xy()) : heading_;
    out_.flight_path_angle = gamma_;
    out_.angle_of_attack = alpha_;
    set_air_angles(out_, air, false);
    out_.load_factor = load_factor_;
    out_.throttle = throttle_;
    out_.mach = tas_ / atm.speed_of_sound;
    if (terrain_.valid()) {
        out_.height_above_terrain = position_.z - terrain_.current();
    } else {
        out_.height_above_terrain =
            runway_elevation_ > kNoGround ? position_.z - runway_elevation_ : kUnknown;
    }
    out_.on_ground = on_ground_;
    out_.avoiding_traffic = avoid_sense_ != 0;
    out_.speed_brake = speed_brake_;
    out_.flaps = flaps_;
    out_.gear_down = gear_down_;
    out_.phase = phase_;
}

void FixedWingAgent::settle_relocation(const Relocation& r) {
    // Without terrain input, a ground reference from far away is meaningless.
    if (r.jump && !on_ground_ && !terrain_.valid() && mode_ != FlightMode::Land)
        runway_elevation_ = kNoGround;
    const double ground = ground_elevation();
    const double wheel_height = position_.z - p_.gear_height - ground;

    if (on_ground_) {
        if (r.new_altitude && wheel_height > kRelocateLiftOffHeight) {
            start_flying_here();
            return;
        }
        // Stays on the ground (the host may only have moved it horizontally).
        if (!terrain_.valid()) runway_elevation_ = position_.z - p_.gear_height;
        position_.z = ground_elevation() + p_.gear_height;
        if (mode_ == FlightMode::Takeoff) runway_elevation_ = ground_elevation();
        if (r.jump && phase_ == FlightPhase::TerrainContact) phase_ = FlightPhase::Parked;
        return;
    }
    if (ground > kNoGround && wheel_height < 0.0) {
        position_.z = ground + p_.gear_height + kRelocateGroundClearance;
        gamma_ = std::max(gamma_, 0.0);
    }
    // An initial climb continues to the climb height above the new ground.
    if (mode_ == FlightMode::Takeoff && r.jump && terrain_.valid())
        runway_elevation_ = terrain_.current();
}

void FixedWingAgent::start_flying_here() {
    on_ground_ = false;
    lining_up_ = false;
    taxi_.clear();
    tas_ = std::max(tas_, cruise_speed_at(isa(position_.z)));
    ground_pitch_ = 0.0;
    flaps_ = false;
    speed_brake_ = false;
    gear_down_ = !p_.retractable_gear;
    trim_level(isa(position_.z));
    phase_ = FlightPhase::Airborne;
    if (mode_ == FlightMode::Takeoff) {
        takeoff_origin_ = position_.xy();
        finish_takeoff(position_.z);
    } else {
        hold_course(heading_, position_.xy(), position_.z, kUnknown, AltitudeRef::Absolute);
    }
}

// --- Helpers --------------------------------------------------------------------------

void FixedWingAgent::trim_level(const AtmosphereSample& atm) {
    gamma_ = 0.0;
    bank_ = 0.0;
    load_factor_ = 1.0;
    const double t_req = drag(tas_, 1.0, atm, false);
    throttle_ = std::clamp(t_req / std::max(thrust_available(tas_, atm), 1e-6),
                           p_.engine.idle_throttle, 1.0);
    const double qs = 0.5 * atm.density * tas_ * tas_ * p_.wing_area;
    alpha_ = (mass_ * kGravity / qs - cl0()) / p_.cl_alpha;
}

L1Settings FixedWingAgent::l1_settings(double ground_speed) const {
    L1Settings s{p_.l1_period, p_.l1_damping, 0.0};
    s.min_distance = kL1TurnFactor * l1_distance_for_turn_radius(s, turn_radius(ground_speed));
    return s;
}

double FixedWingAgent::ground_elevation() const {
    return terrain_.valid() ? terrain_.current() : runway_elevation_;
}

double FixedWingAgent::altitude_target_now(double lookahead) const {
    if (altitude_ref_ != AltitudeRef::AboveTerrain) return altitude_target_;
    const double ground = terrain_.valid() ? terrain_.extrapolate(lookahead)
                                           : std::max(runway_elevation_, 0.0);
    return ground + altitude_target_;
}

double FixedWingAgent::engine_lapse(const AtmosphereSample& atm) const {
    double lapse = density_lapse(atm.sigma, p_.engine.density_exponent);
    // Turbocharged / flat-rated: the rating holds up to the critical altitude.
    if (p_.engine.critical_altitude > 0.0) lapse = std::min(1.0, lapse / rated_lapse_);
    return lapse;
}

double FixedWingAgent::thrust_available(double tas, const AtmosphereSample& atm) const {
    const EngineParams& e = p_.engine;
    if (fuel_modelled() && fuel_ <= 0.0) return 0.0;  // out of fuel
    const double lapse = engine_lapse(atm);
    if (e.type == EngineType::Jet) {
        const double mach = tas / atm.speed_of_sound;
        return e.max_thrust * lapse * std::clamp(1.0 - e.mach_lapse * mach, 0.2, 2.0);
    }
    const double power_thrust = e.prop_efficiency * e.max_power * lapse / std::max(tas, 1.0);
    return std::min(e.static_thrust * lapse, power_thrust);
}

double FixedWingAgent::drag(double tas, double load_factor, const AtmosphereSample& atm,
                            bool brake) const {
    const double qs = 0.5 * atm.density * tas * tas * p_.wing_area;
    if (qs < 1e-6) return 0.0;
    const double cl = std::min(load_factor * mass_ * kGravity / qs, 1.2 * cl_max());
    double cd = cd0() + p_.induced_drag_k * cl * cl;
    if (p_.wave_drag > 0.0 && p_.mach_critical < 1.0) {
        const double x = std::clamp((tas / atm.speed_of_sound - p_.mach_critical) /
                                        (1.0 - p_.mach_critical), 0.0, 1.0);
        cd += p_.wave_drag * x * x;
    }
    if (brake) cd += p_.speedbrake_dcd0;
    if (gear_down_ && p_.retractable_gear) cd += p_.gear_dcd0;
    return qs * cd;
}

double FixedWingAgent::cl0() const { return p_.cl0 + (flaps_ ? p_.flaps_dcl0 : 0.0); }
double FixedWingAgent::cl_max() const { return p_.cl_max + (flaps_ ? p_.flaps_dcl_max : 0.0); }
double FixedWingAgent::cd0() const { return p_.cd0 + (flaps_ ? p_.flaps_dcd0 : 0.0); }

double FixedWingAgent::stall_speed(double density, bool flaps) const {
    const double cl = p_.cl_max + (flaps ? p_.flaps_dcl_max : 0.0);
    return std::sqrt(2.0 * mass_ * kGravity / (density * p_.wing_area * cl));
}

double FixedWingAgent::min_drag_speed(double density) const {
    const double cl = std::sqrt(cd0() / std::max(p_.induced_drag_k, 1e-6));
    return std::sqrt(2.0 * mass_ * kGravity / (density * p_.wing_area * cl));
}

double FixedWingAgent::fuel_flow(double tas, const AtmosphereSample& atm) const {
    const EngineParams& e = p_.engine;
    if (!fuel_modelled() || fuel_ <= 0.0) return 0.0;
    if (e.type == EngineType::Propeller) {
        // Shaft power: the throttle share of the power available here.
        return e.fuel_consumption * throttle_ * e.max_power * engine_lapse(atm);
    }
    const double full = thrust_available(tas, atm);
    const double thrust = throttle_ * full;
    const double dry = e.dry_thrust_fraction * full;
    if (thrust <= dry || e.dry_thrust_fraction >= 1.0) return e.fuel_consumption * thrust;
    // Afterburner: from the dry flow at full dry thrust to the full afterburner flow.
    const double share = (thrust - dry) / std::max(full - dry, 1e-9);
    const double dry_flow = e.fuel_consumption * dry;
    return dry_flow + share * (e.afterburner_consumption * full - dry_flow);
}

void FixedWingAgent::burn_fuel(double tas, const AtmosphereSample& atm, double dt) {
    // Parked or stopped after landing: engines off.
    const bool engines_off = on_ground_ && (phase_ == FlightPhase::Parked ||
                                            phase_ == FlightPhase::Landed ||
                                            phase_ == FlightPhase::TerrainContact);
    fuel_flow_ = engines_off ? 0.0 : fuel_flow(tas, atm);
    if (fuel_flow_ > 0.0) set_fuel(fuel_ - fuel_flow_ * dt);
}

double FixedWingAgent::sustained_speed(const AtmosphereSample& atm, double v_min, double v_max) {
    if (std::abs(position_.z - sustained_altitude_) > kClimbSpeedRefresh) {
        auto excess = [&](double v) { return thrust_available(v, atm) - drag(v, 1.0, atm, false); };
        double lo = std::max(v_min, min_drag_speed(atm.density));
        double hi = std::max(lo, v_max);
        if (excess(hi) >= 0.0) {
            lo = hi;
        } else if (excess(lo) > 0.0) {
            for (int i = 0; i < 30; ++i) {
                const double mid = 0.5 * (lo + hi);
                (excess(mid) > 0.0 ? lo : hi) = mid;
            }
        }
        sustained_speed_ = lo;
        sustained_altitude_ = position_.z;
    }
    return std::clamp(sustained_speed_, v_min, v_max);
}

double FixedWingAgent::lead_bank_limit(const AtmosphereSample& atm, double v_min, double v_max) {
    // In a turn of radius R a follower d to the outside flies (1 + d/R) times faster at the
    // same turn rate, one to the inside (1 - d/R) times slower; d/R = d g tan(bank) / v^2.
    const double v = std::max(tas_, v_min);
    const double a = lead_extent_ * kGravity / (v * v);  // d/R per unit tan(bank)
    const double follower_max = std::isfinite(lead_follower_speed_)
                                    ? lead_follower_speed_
                                    : kFollowerSpeedShare * sustained_speed(atm, v_min, v_max);
    const double fast = std::min(follower_max / v - 1.0, 1.0 - 1.1 * v_min / v);
    double tan_bank = std::max(fast, 0.0) / a;
    // The outside follower turns at the leader's rate with at most its own bank limit:
    // tan(bank) (1 + a tan(bank)) <= tan(bank_max).
    const double tan_max = 0.9 * std::tan(p_.bank_max);
    tan_bank = std::min(tan_bank, (std::sqrt(1.0 + 4.0 * a * tan_max) - 1.0) / (2.0 * a));
    return std::clamp(std::atan(tan_bank), deg2rad(5.0), p_.bank_max);
}

double FixedWingAgent::climb_rate_at(double tas, const AtmosphereSample& atm) const {
    const double v = std::max(tas, kMinAirspeed);
    return (thrust_available(v, atm) - drag(v, 1.0, atm, false)) * v / (mass_ * kGravity);
}

double FixedWingAgent::climb_speed(const AtmosphereSample& atm, double v_min, double v_max) {
    if (p_.climb_speed > 0.0) return std::clamp(p_.climb_speed / std::sqrt(atm.sigma), v_min, v_max);
    // Best climb rate speed of the model, refreshed as the altitude changes.
    if (std::abs(position_.z - best_climb_altitude_) > kClimbSpeedRefresh) {
        constexpr int kSamples = 24;
        double best_rate = -1e9;
        for (int i = 0; i <= kSamples; ++i) {
            const double v = v_min + (v_max - v_min) * i / kSamples;
            const double rate = climb_rate_at(v, atm);
            if (rate > best_rate) {
                best_rate = rate;
                best_climb_speed_ = v;
            }
        }
        best_climb_altitude_ = position_.z;
    }
    return std::clamp(best_climb_speed_, v_min, v_max);
}

double FixedWingAgent::turn_radius(double speed) const {
    return speed * speed / (kGravity * std::tan(std::max(p_.bank_max, deg2rad(5.0))));
}

double FixedWingAgent::rotate_speed(double density) const {
    return p_.rotate_speed > 0.0 ? p_.rotate_speed : 1.08 * stall_speed(density, true);
}

double FixedWingAgent::loiter_radius_default() const {
    // A gentle orbit at the speed it will be flown at: at most 30 deg of bank.
    const AtmosphereSample atm = isa(position_.z, temperature_offset_);
    const double v = speed_commanded_
                         ? speed_target_
                         : true_airspeed_from_calibrated(loiter_calibrated_speed(), atm);
    return 1.1 * v * v / (kGravity * std::tan(std::min(p_.bank_max, kLoiterBank)));
}

double FixedWingAgent::cruise_speed_at(const AtmosphereSample& atm) const {
    if (cruise_calibrated_ <= 0.0) return p_.cruise_speed;
    return std::min(p_.cruise_speed, true_airspeed_from_calibrated(cruise_calibrated_, atm));
}

double FixedWingAgent::default_speed(const AtmosphereSample& atm) const {
    if (mode_ == FlightMode::Loiter)
        return true_airspeed_from_calibrated(loiter_calibrated_speed(), atm);
    return cruise_speed_at(atm);
}

double FixedWingAgent::loiter_calibrated_speed() const {
    if (p_.loiter_speed > 0.0) return p_.loiter_speed;
    // Best endurance: the minimum drag speed for jets, a bit slower for propeller aircraft
    // (as equivalent airspeeds, at sea level density), with a margin above the stall in
    // the orbit's bank.
    const double rho = isa(0.0).density;
    const double factor = p_.engine.type == EngineType::Jet ? 1.0 : 0.8;
    const double cl = std::sqrt(p_.cd0 / std::max(p_.induced_drag_k, 1e-6));
    const double v_md = std::sqrt(2.0 * mass_ * kGravity / (rho * p_.wing_area * cl));
    return std::max(factor * v_md, kLoiterStallMargin * stall_speed(rho, false));
}

double FixedWingAgent::approach_speed(double density) const {
    return p_.approach_speed_factor * stall_speed(density, true);
}

Vec3 FixedWingAgent::aim_point() const {
    const Vec2 d = heading_vector(runway_.heading) * p_.touchdown_distance;
    return {runway_.start.x + d.x, runway_.start.y + d.y, runway_.start.z};
}

}  // namespace fm
