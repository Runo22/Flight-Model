#include "fm/fixed_wing.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "fm/guidance.hpp"

namespace fm {

namespace {
constexpr double kMaxSubstep = 0.02;       // s
constexpr double kMinAirspeed = 1.0;       // m/s, numerical floor
constexpr double kNoGround = -1e9;         // no ground reference known
constexpr double kLowBankHeight = 30.0;    // m, bank is restricted below this height
constexpr double kLowBankLimit = deg2rad(10.0);
constexpr double kTakeoffBankLimit = deg2rad(15.0);
constexpr double kWheelBrakeFriction = 0.3;
constexpr double kStallMargin = 1.3;       // minimum speed = 1.3 Vs
constexpr double kTurnStallMargin = 1.2;   // bank limited to keep n <= n_stall / 1.2
}  // namespace

FixedWingAgent::FixedWingAgent(FixedWingParams params, const InitialConditions& ic)
    : p_(std::move(params)), terrain_(std::max(1.0, p_.cruise_speed * 0.25)) {
    position_ = ic.position;
    heading_ = wrap_2pi(ic.heading);
    heading_target_ = heading_;
    speed_target_ = p_.cruise_speed;
    runway_elevation_ = kNoGround;
    on_ground_ = ic.on_ground;

    const AtmosphereSample atm = isa(position_.z);
    if (on_ground_) {
        // On the ground the given z is the ground elevation.
        runway_elevation_ = ic.position.z;
        position_.z = ic.position.z + p_.gear_height;
        tas_ = std::max(0.0, ic.speed);
        throttle_ = p_.engine.idle_throttle;
        phase_ = FlightPhase::Parked;
    } else {
        tas_ = ic.speed > 0.0 ? ic.speed : p_.cruise_speed;
        speed_target_ = tas_;
        const double t_req = drag(tas_, 1.0, atm, false);
        throttle_ = std::clamp(t_req / std::max(thrust_available(tas_, atm), 1e-6),
                               p_.engine.idle_throttle, 1.0);
        const double qs = 0.5 * atm.density * tas_ * tas_ * p_.wing_area;
        alpha_ = (p_.mass * kGravity / qs - cl0()) / p_.cl_alpha;
        phase_ = FlightPhase::Airborne;
    }
    altitude_target_ = position_.z;
    resolved_altitude_target_ = position_.z;
    update_output(atm, {});
}

// --- Commands -------------------------------------------------------------------------

void FixedWingAgent::cruise() {
    hold(heading_, resolved_altitude_target_, speed_target_);
}

void FixedWingAgent::hold(double heading, double altitude, double speed, AltitudeRef altitude_ref) {
    mode_ = FlightMode::Cruise;
    route_.clear();
    heading_target_ = wrap_2pi(heading);
    if (altitude_ref != AltitudeRef::Keep) {
        altitude_target_ = altitude;
        altitude_ref_ = altitude_ref;
    }
    if (!std::isnan(speed) && speed > 0.0) speed_target_ = speed;
}

void FixedWingAgent::set_speed(double speed) {
    if (!std::isnan(speed) && speed > 0.0) speed_target_ = speed;
}

void FixedWingAgent::go_to(const Vec3& point, const GoToOptions& options) {
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

void FixedWingAgent::follow_route(Route route) {
    if (route.waypoints.empty()) {
        cruise();
        return;
    }
    if (on_ground_) {
        // A parked aircraft takes off along its current heading first.
        TakeoffPlan plan;
        plan.runway = {position_, heading_};
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
    if (altitude_ref != AltitudeRef::Keep) {
        altitude_target_ = center.z;
        altitude_ref_ = altitude_ref;
    }
}

bool FixedWingAgent::takeoff(const TakeoffPlan& plan) {
    if (!on_ground_ || phase_ == FlightPhase::TerrainContact) return false;
    takeoff_ = plan;
    takeoff_stage_ = TakeoffStage::Roll;
    mode_ = FlightMode::Takeoff;
    route_.clear();
    heading_target_ = wrap_2pi(plan.runway.heading);
    runway_elevation_ = ground_elevation();
    flaps_ = true;
    phase_ = FlightPhase::TakeoffRoll;
    return true;
}

// --- Simulation -----------------------------------------------------------------------

void FixedWingAgent::step(double dt, const Environment& env) {
    if (!(dt > 0.0)) return;
    terrain_.update(position_.xy(), env.terrain_elevation);
    const int substeps = std::max(1, static_cast<int>(std::ceil(dt / kMaxSubstep)));
    const double h = dt / substeps;
    AtmosphereSample atm{};
    for (int i = 0; i < substeps; ++i) {
        atm = isa(position_.z, env.temperature_offset);
        const Command cmd = guidance(atm, env.wind);
        const Control c = autopilot(cmd, atm);
        integrate(c, atm, env.wind, h);
    }
    update_output(atm, env.wind);
}

FixedWingAgent::Command FixedWingAgent::guidance(const AtmosphereSample& atm, const Vec3& wind) {
    Command cmd;
    cmd.heading = heading_target_;
    cmd.speed = speed_target_;
    cmd.bank_limit = p_.bank_max;

    const Vec2 air_h = heading_vector(heading_) * (tas_ * std::cos(gamma_));
    const Vec2 vg = on_ground_ ? air_h : air_h + wind.xy();
    const double gs = std::max(vg.length(), 1.0);
    const double lookahead = gs * p_.altitude_time_constant;

    if (on_ground_ && mode_ != FlightMode::Takeoff) {
        // Parked, landed or crashed: idle and brake.
        cmd.heading = heading_;
        cmd.altitude = position_.z;
        cmd.speed = 0.0;
        cmd.bank_limit = 0.0;
        return cmd;
    }

    if (mode_ == FlightMode::GoTo || mode_ == FlightMode::Route) {
        if (route_.active()) guide_route(cmd, vg, lookahead);
        // guide_route may finish the route and switch to Cruise/Loiter.
    }
    switch (mode_) {
        case FlightMode::Cruise:
            cmd.heading = heading_target_;
            cmd.altitude = altitude_target_now(lookahead);
            break;
        case FlightMode::Loiter: {
            const L1Settings l1{p_.l1_period, p_.l1_damping};
            cmd.use_accel = true;
            cmd.lateral_accel = l1_loiter(loiter_center_.xy(), loiter_radius_, loiter_clockwise_,
                                          position_.xy(), vg, l1);
            cmd.altitude = altitude_target_now(lookahead);
            break;
        }
        case FlightMode::Takeoff:
            guide_takeoff(cmd, atm);
            break;
        default:
            break;
    }
    cmd.speed = speed_target_;
    if (mode_ == FlightMode::Takeoff && flaps_) cmd.speed = 1.25 * stall_speed(atm.density, true);
    resolved_altitude_target_ = cmd.altitude;

    if (!on_ground_) {
        // Terrain floor from the predicted terrain ahead.
        if (terrain_.valid() && mode_ != FlightMode::Takeoff) {
            const double floor = terrain_.predicted_max(gs * p_.terrain_lookahead) +
                                 p_.min_terrain_clearance;
            cmd.altitude = std::max(cmd.altitude, floor);
            cmd.terrain_alert = position_.z < floor;
        }
        const double ground = ground_elevation();
        if (position_.z - p_.gear_height - ground < kLowBankHeight)
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
        const Vec2 next_dir = (next->position.xy() - wp.position.xy()).normalized();
        const double turn = std::min(std::abs(angle_right(lp.direction, next_dir)), deg2rad(150.0));
        const double next_len = (next->position.xy() - wp.position.xy()).length();
        switch_distance = std::min({turn_radius(gs) * std::tan(turn / 2.0), 0.5 * lp.length,
                                    0.5 * next_len});
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
    if (!std::isnan(target.speed) && target.speed > 0.0) speed_target_ = target.speed;
    const L1Settings l1{p_.l1_period, p_.l1_damping};
    cmd.use_accel = true;
    cmd.lateral_accel = l1_leg(route_.leg_start(), route_.leg_end(), pos, vg, l1);
    cmd.altitude = route_.altitude_target(lp, terrain_, lookahead, resolved_altitude_target_);
}

void FixedWingAgent::finish_route(EndBehavior behavior) {
    const Waypoint last = route_.target();
    const double last_altitude = RouteTracker::resolve_altitude(
        last, terrain_, 0.0, resolved_altitude_target_);
    const bool agl = last.altitude_ref == AltitudeRef::AboveTerrain;
    const double radius = route_.route().loiter_radius;
    route_.clear();
    if (behavior == EndBehavior::ContinueStraight) {
        hold(heading_, agl ? last.position.z : last_altitude, speed_target_,
             agl ? AltitudeRef::AboveTerrain : AltitudeRef::Absolute);
        return;
    }
    // Fixed-wing aircraft cannot hover or land vertically: orbit the last waypoint.
    Vec3 center = last.position;
    if (!agl) center.z = last_altitude;
    loiter(center, radius, true, agl ? AltitudeRef::AboveTerrain : AltitudeRef::Absolute);
}

void FixedWingAgent::guide_takeoff(Command& cmd, const AtmosphereSample& atm) {
    const double climb_height =
        takeoff_.climb_height > 0.0 ? takeoff_.climb_height : p_.takeoff_climb_height;
    cmd.heading = wrap_2pi(takeoff_.runway.heading);
    cmd.altitude = runway_elevation_ + climb_height;

    if (on_ground_) {
        cmd.full_thrust = true;
        cmd.bank_limit = 0.0;
        cmd.rotate = tas_ >= rotate_speed(atm.density);
        phase_ = cmd.rotate ? FlightPhase::Rotation : FlightPhase::TakeoffRoll;
        return;
    }

    takeoff_stage_ = TakeoffStage::Climb;
    phase_ = FlightPhase::InitialClimb;
    cmd.bank_limit = std::min(p_.bank_max, kTakeoffBankLimit);
    const double height = position_.z - p_.gear_height - runway_elevation_;
    if (flaps_ && height > p_.flaps_up_height) flaps_ = false;

    if (height >= climb_height - 15.0) {
        phase_ = FlightPhase::Airborne;
        flaps_ = false;
        if (takeoff_.then) {
            Route r = std::move(*takeoff_.then);
            takeoff_.then.reset();
            follow_route(std::move(r));
        } else {
            hold(takeoff_.runway.heading, runway_elevation_ + climb_height, speed_target_);
        }
    }
}

FixedWingAgent::Control FixedWingAgent::autopilot(const Command& cmd, const AtmosphereSample& atm) {
    Control c;
    c.rotate = cmd.rotate;
    if (on_ground_) {
        c.throttle = cmd.full_thrust ? 1.0 : p_.engine.idle_throttle;
        c.bank = cmd.heading;  // ground steering target (heading)
        return c;
    }

    const double g = kGravity;
    const double m = p_.mass;
    const double v = std::max(tas_, kMinAirspeed);

    // Speed envelope
    const double v_min = kStallMargin * stall_speed(atm.density, flaps_);
    const double v_max = std::max(v_min, std::min(p_.max_speed / std::sqrt(atm.sigma),
                                                  p_.mach_max * atm.speed_of_sound));
    const double v_cmd = std::clamp(cmd.speed, v_min, v_max);

    // Lateral: bank from lateral acceleration or heading error, limited by stall margin.
    const double qs = 0.5 * atm.density * v * v * p_.wing_area;
    const double n_stall = qs * cl_max() / (m * g);
    const double n_turn = std::min(p_.load_factor_max, n_stall / kTurnStallMargin);
    const double bank_limit =
        std::min(cmd.bank_limit, n_turn > 1.0 ? std::acos(1.0 / n_turn) : deg2rad(5.0));
    double bank_cmd = 0.0;
    if (cmd.use_accel) {
        bank_cmd = std::atan(cmd.lateral_accel / g);
    } else {
        const double turn_rate = wrap_pi(cmd.heading - heading_) / p_.heading_time_constant;
        bank_cmd = std::atan(v * turn_rate / g);
    }
    c.bank = std::clamp(bank_cmd, -bank_limit, bank_limit);

    // Energy management: thrust controls total energy, flight path splits it between
    // altitude and speed (TECS-like, solved with the model inverse).
    const bool has_brake = p_.speedbrake_dcd0 > 0.0;
    const double n_now = std::cos(gamma_) / std::max(std::cos(bank_), 0.2);
    const double t_max = thrust_available(v, atm);
    const double t_idle = p_.engine.idle_throttle * t_max;
    const double excess = (t_max - drag(v, n_now, atm, false)) / m;
    const double idle_accel = (t_idle - drag(v, n_now, atm, has_brake)) / m;

    double accel_cap = 0.5 * std::max(excess, 0.0);
    if (v < v_min) accel_cap = std::max(excess, 0.0);
    else if (cmd.terrain_alert) accel_cap = 0.0;
    const double decel_cap = std::max(-idle_accel, 0.0);
    const double vdot_cmd =
        std::clamp((v_cmd - v) / p_.speed_time_constant, -decel_cap, accel_cap);

    const double hdot_cmd = std::clamp((cmd.altitude - position_.z) / p_.altitude_time_constant,
                                       -p_.descent_rate_max, p_.climb_rate_max);
    double gamma_cmd = std::asin(std::clamp(hdot_cmd / v, -0.9, 0.9));
    gamma_cmd = std::clamp(gamma_cmd, -p_.descent_angle_max, p_.climb_angle_max);

    const double gamma_lo = std::min(std::asin(std::clamp((idle_accel - vdot_cmd) / g, -0.9, 0.9)), 0.0);
    double gamma_hi = std::asin(std::clamp((excess - vdot_cmd) / g, -0.9, 0.9));
    if (v >= v_min) gamma_hi = std::max(gamma_hi, std::min(gamma_cmd, 0.0));
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

void FixedWingAgent::integrate(const Control& c, const AtmosphereSample& atm, const Vec3& wind,
                               double dt) {
    throttle_ = lag(throttle_, c.throttle, p_.engine.spool_time, dt);
    if (on_ground_) {
        integrate_ground(c, atm, dt);
        return;
    }
    const double g = kGravity;
    const double m = p_.mass;
    const double w = m * g;
    speed_brake_ = c.speed_brake;

    const double roll_rate = std::clamp((c.bank - bank_) / p_.roll_time_constant,
                                        -p_.roll_rate_max, p_.roll_rate_max);
    bank_ += roll_rate * dt;

    const double v = std::max(tas_, kMinAirspeed);
    const double qs = 0.5 * atm.density * v * v * p_.wing_area;
    const double n_hi = std::min(p_.load_factor_max, qs * cl_max() / w);
    const double n_lo = std::min(std::max(p_.load_factor_min, -0.5 * qs * cl_max() / w), n_hi);
    const double cos_bank = std::max(std::cos(bank_), 0.05);
    const double gamma_rate_cmd = (c.gamma - gamma_) / p_.gamma_time_constant;
    const double n = std::clamp((std::cos(gamma_) + v * gamma_rate_cmd / g) / cos_bank, n_lo, n_hi);
    load_factor_ = n;

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
        phase_ = gentle ? FlightPhase::Landed : FlightPhase::TerrainContact;
        mode_ = FlightMode::Cruise;
        route_.clear();
        flaps_ = false;
        speed_brake_ = false;
    }
}

void FixedWingAgent::integrate_ground(const Control& c, const AtmosphereSample& atm, double dt) {
    const double g = kGravity;
    const double m = p_.mass;
    const double w = m * g;
    const double v = tas_;

    // Nose wheel steering towards the commanded heading (stored in c.bank on the ground).
    heading_ = wrap_2pi(heading_ + std::clamp(wrap_pi(c.bank - heading_), -deg2rad(5.0) * dt,
                                              deg2rad(5.0) * dt));
    double rotation_pitch = p_.rotation_pitch;
    if (rotation_pitch <= 0.0) {
        const double cl_lof = (cl_max() / 1.3225);  // lift-off near 1.15 Vs
        rotation_pitch = std::clamp((cl_lof - cl0()) / p_.cl_alpha, deg2rad(5.0), deg2rad(12.0));
    }
    ground_pitch_ = approach(ground_pitch_, c.rotate ? rotation_pitch : 0.0, p_.rotation_rate * dt);
    alpha_ = ground_pitch_;

    const double qs = 0.5 * atm.density * v * v * p_.wing_area;
    const double cl = std::min(cl0() + p_.cl_alpha * alpha_, cl_max());
    const double lift = qs * cl;
    const double aero_drag = qs * (cd0() + p_.induced_drag_k * cl * cl);
    const double normal = std::max(w - lift, 0.0);
    const bool braking = mode_ != FlightMode::Takeoff;
    const double friction = (braking ? kWheelBrakeFriction : p_.rolling_friction) * normal;
    const double thrust = throttle_ * thrust_available(v, atm);

    double v_dot = (thrust - aero_drag) / m;
    if (v > 1e-3 || std::abs(thrust - aero_drag) > friction) {
        v_dot -= (v > 1e-3 ? friction : std::copysign(friction, thrust - aero_drag)) / m;
    } else {
        v_dot = 0.0;  // static friction holds the aircraft
    }
    tas_ = std::max(0.0, v + v_dot * dt);
    const Vec2 step = heading_vector(heading_) * (tas_ * dt);
    position_.x += step.x;
    position_.y += step.y;
    position_.z = ground_elevation() + p_.gear_height;
    load_factor_ = lift / w;
    gamma_ = 0.0;
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
    out_.attitude.pitch = on_ground_ ? ground_pitch_ : gamma_ + alpha_ * std::cos(bank_);
    out_.airspeed = tas_;
    out_.ground_speed = out_.velocity.xy().length();
    out_.vertical_speed = out_.velocity.z;
    out_.track = out_.ground_speed > 0.1 ? heading_of(out_.velocity.xy()) : heading_;
    out_.flight_path_angle = gamma_;
    out_.angle_of_attack = alpha_;
    out_.load_factor = load_factor_;
    out_.throttle = throttle_;
    out_.mach = tas_ / atm.speed_of_sound;
    out_.height_above_terrain =
        terrain_.valid() ? position_.z - terrain_.current()
                         : (runway_elevation_ > kNoGround ? position_.z - runway_elevation_ : kUnknown);
    out_.on_ground = on_ground_;
    out_.speed_brake = speed_brake_;
    out_.flaps = flaps_;
    out_.phase = phase_;
}

// --- Helpers --------------------------------------------------------------------------

double FixedWingAgent::ground_elevation() const {
    return terrain_.valid() ? terrain_.current() : runway_elevation_;
}

double FixedWingAgent::altitude_target_now(double lookahead) const {
    if (altitude_ref_ != AltitudeRef::AboveTerrain) return altitude_target_;
    const double ground = terrain_.valid() ? terrain_.extrapolate(lookahead)
                                           : std::max(runway_elevation_, 0.0);
    return ground + altitude_target_;
}

double FixedWingAgent::thrust_available(double tas, const AtmosphereSample& atm) const {
    const EngineParams& e = p_.engine;
    const double lapse = std::pow(atm.sigma, e.density_exponent);
    if (e.type == EngineType::Jet) {
        const double mach = tas / atm.speed_of_sound;
        return e.max_thrust * lapse * std::clamp(1.0 - e.mach_lapse * mach, 0.2, 1.5);
    }
    const double power_thrust = e.prop_efficiency * e.max_power * lapse / std::max(tas, 1.0);
    return std::min(e.static_thrust * lapse, power_thrust);
}

double FixedWingAgent::drag(double tas, double load_factor, const AtmosphereSample& atm,
                            bool brake) const {
    const double qs = 0.5 * atm.density * tas * tas * p_.wing_area;
    if (qs < 1e-6) return 0.0;
    const double cl = std::min(load_factor * p_.mass * kGravity / qs, 1.2 * cl_max());
    double cd = cd0() + p_.induced_drag_k * cl * cl;
    if (p_.wave_drag > 0.0 && p_.mach_critical < 1.0) {
        const double x = std::clamp((tas / atm.speed_of_sound - p_.mach_critical) /
                                        (1.0 - p_.mach_critical), 0.0, 1.0);
        cd += p_.wave_drag * x * x;
    }
    if (brake) cd += p_.speedbrake_dcd0;
    return qs * cd;
}

double FixedWingAgent::cl0() const { return p_.cl0 + (flaps_ ? p_.flaps_dcl0 : 0.0); }
double FixedWingAgent::cl_max() const { return p_.cl_max + (flaps_ ? p_.flaps_dcl_max : 0.0); }
double FixedWingAgent::cd0() const { return p_.cd0 + (flaps_ ? p_.flaps_dcd0 : 0.0); }

double FixedWingAgent::stall_speed(double density, bool flaps) const {
    const double cl = p_.cl_max + (flaps ? p_.flaps_dcl_max : 0.0);
    return std::sqrt(2.0 * p_.mass * kGravity / (density * p_.wing_area * cl));
}

double FixedWingAgent::turn_radius(double speed) const {
    return speed * speed / (kGravity * std::tan(std::max(p_.bank_max, deg2rad(5.0))));
}

double FixedWingAgent::rotate_speed(double density) const {
    return p_.rotate_speed > 0.0 ? p_.rotate_speed : 1.08 * stall_speed(density, true);
}

double FixedWingAgent::loiter_radius_default() const {
    return 1.2 * turn_radius(std::max(tas_, speed_target_));
}

}  // namespace fm
