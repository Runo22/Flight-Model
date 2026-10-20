#include "fm/ship.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace fm {

namespace {
constexpr double kMaxSubstep = 0.1;          // s
constexpr double kAddedMass = 1.05;          // surge added mass factor
constexpr double kSpeedTau = 20.0;           // s, speed loop
constexpr double kHeelTau = 5.0;             // s
constexpr double kStationSpeed = 2.5;        // m/s, highest speed while keeping station
constexpr double kLeaderTimeout = 2.0;       // s without leader updates before giving up
constexpr double kFormationAlongGain = 0.01; // 1/s, along-track slot error -> speed
constexpr double kHeadingBandwidth = 1.5;    // heading loop natural frequency * yaw lag
constexpr double kHeadingDamping = 0.9;
constexpr double kLookahead = 3.0;           // line-of-sight distance / (T' * L)
constexpr double kLandMargin = 0.5;          // m, terrain this far above the water is land
}  // namespace

ShipAgent::ShipAgent(ShipParams params, const InitialConditions& ic)
    : p_(std::move(params)), terrain_(std::max(1.0, 0.5 * p_.length)) {
    position_ = {ic.position.x, ic.position.y, p_.water_level};
    heading_ = wrap_2pi(ic.heading);
    speed_ = std::clamp(ic.speed, 0.0, p_.max_speed);
    speed_target_ = speed_;
    thrust_ = resistance(speed_);
    course_target_ = heading_;
    course_anchor_ = position_.xy();
    hold_point_ = position_.xy();
    update_output();
}

// --- Commands -------------------------------------------------------------------------

void ShipAgent::cruise() {
    const double course = std::abs(speed_) > 0.5 ? heading_ - drift_ : heading_;
    hold(course, 0.0, speed_target_);
}

void ShipAgent::hold(double course, double, double speed, AltitudeRef) {
    mode_ = FlightMode::Cruise;
    steering_ = false;
    route_.clear();
    course_target_ = wrap_2pi(course);
    course_anchor_ = position_.xy();
    set_speed(speed);
}

void ShipAgent::steer(double turn_rate) {
    if (std::isnan(turn_rate)) return;
    mode_ = FlightMode::Cruise;
    route_.clear();
    steering_ = true;
    steer_rate_ = turn_rate;
}

void ShipAgent::set_speed(double speed) {
    if (std::isfinite(speed)) speed_target_ = std::clamp(speed, -p_.max_astern_speed, p_.max_speed);
}

void ShipAgent::ensure_moving() {
    if (speed_target_ <= 0.0) speed_target_ = p_.cruise_speed;
}

void ShipAgent::go_to(const Vec3& point, const GoToOptions& options) {
    Route r;
    Waypoint wp;
    wp.position = point;
    wp.speed = options.speed;
    wp.fly_over = true;
    r.waypoints.push_back(wp);
    r.on_end = options.on_arrival;
    follow_route(std::move(r));
    if (mode_ == FlightMode::Route) mode_ = FlightMode::GoTo;
}

void ShipAgent::follow_route(Route route) {
    steering_ = false;
    if (route.waypoints.empty()) {
        cruise();
        return;
    }
    route_.start(std::move(route), position_, p_.water_level);
    mode_ = FlightMode::Route;
    ensure_moving();
}

void ShipAgent::loiter(const Vec3& center, double radius, bool clockwise, AltitudeRef) {
    steering_ = false;
    route_.clear();
    if (radius <= 0.0) {
        mode_ = FlightMode::Hover;
        hold_point_ = center.xy();
        hold_here_ = false;
        return;
    }
    mode_ = FlightMode::Loiter;
    loiter_center_ = center;
    loiter_radius_ = std::max(radius, 1.2 * turning_radius());
    loiter_clockwise_ = clockwise;
    ensure_moving();
}

void ShipAgent::hover() {
    steering_ = false;
    mode_ = FlightMode::Hover;
    route_.clear();
    hold_point_ = position_.xy();
    hold_here_ = true;
}

void ShipAgent::join_formation(const FormationSlot& slot) {
    steering_ = false;
    mode_ = FlightMode::Formation;
    route_.clear();
    slot_ = slot;
    leader_tracker_.reset();
    leader_age_ = 0.0;
    course_anchor_ = position_.xy();
    course_target_ = heading_;
}

void ShipAgent::set_leader_state(const LeaderState& leader) {
    leader_ = leader;
    leader_age_ = 0.0;
}

bool ShipAgent::relocate(const Vec3& position, double heading) {
    const Vec3 at{position.x, position.y, p_.water_level};
    const Relocation r = classify_relocation(position_, heading_, at, heading);
    if ((at - position_).xy().length() > 2.0 * terrain_.spacing()) terrain_.reset();
    position_ = at;
    heading_ = wrap_2pi(heading_ + r.turn);
    aground_ = false;  // checked again with the terrain at the new place
    if (r.jump) {
        const Vec2 here = position_.xy();
        switch (mode_) {
            case FlightMode::Cruise:
            case FlightMode::Formation:
                course_target_ = wrap_2pi(course_target_ + r.turn);
                course_anchor_ = here;
                leader_tracker_.reset();
                break;
            case FlightMode::GoTo:
            case FlightMode::Route:
                route_.restart_leg(position_, p_.water_level);
                break;
            case FlightMode::Hover:
                if (hold_here_) hold_point_ = here;
                break;
            default:
                break;
        }
    }
    update_output();
    return true;
}

// --- Simulation -----------------------------------------------------------------------

void ShipAgent::step(double dt, const Environment& input) {
    if (!(dt > 0.0) || !std::isfinite(dt)) return;
    const Environment env = sanitized(input);
    terrain_.update(position_.xy(), env.terrain_elevation);
    if (!aground_ && terrain_.valid() && terrain_.current() > p_.water_level + kLandMargin) {
        aground_ = true;  // sailed onto land
        speed_ = 0.0;
        turn_rate_ = 0.0;
        thrust_ = 0.0;
    }

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
    for (int i = 0; i < substeps; ++i) {
        SlotReference ref = slot_ref;
        ref.position += ref.velocity * (h * i);
        integrate(guidance(ref), h);
        time_ += h;
        update_output();
    }
}

ShipAgent::Command ShipAgent::guidance(const SlotReference& slot_ref) {
    Command cmd;
    cmd.speed = speed_target_;
    if (aground_) {
        cmd.speed = 0.0;
        return cmd;
    }
    if ((mode_ == FlightMode::GoTo || mode_ == FlightMode::Route) && route_.active()) {
        guide_route(cmd);  // may finish the route and switch mode
        if (mode_ == FlightMode::GoTo || mode_ == FlightMode::Route) return cmd;
        cmd.speed = speed_target_;
    }
    switch (mode_) {
        case FlightMode::Cruise:
            cmd.turn_rate = steering_ ? steer_rate_
                                      : steer_to(line_course(course_anchor_, course_target_), 0.0);
            break;
        case FlightMode::Loiter:
            guide_circle(cmd);
            break;
        case FlightMode::Hover:
            keep_station(cmd, hold_point_);
            break;
        case FlightMode::Formation:
            if (leader_.valid) {
                guide_formation(cmd, slot_ref);
            } else {
                cmd.turn_rate = steer_to(line_course(course_anchor_, course_target_), 0.0);
            }
            break;
        default:
            break;
    }
    return cmd;
}

void ShipAgent::guide_route(Command& cmd) {
    const Vec2 pos = position_.xy();
    const EndBehavior end = route_.route().on_end;
    LegProgress lp = route_.progress(pos);
    auto stops_at_target = [&] {
        return route_.is_last() && end != EndBehavior::ContinueStraight &&
               !(end == EndBehavior::Loiter && route_.route().loiter_radius > 0.0);
    };

    // Fly-by: start turning before the waypoint, early enough for the steering lag (the
    // ship runs on ~U * T before its turn rate builds up), so the turn joins the next leg
    // from the inside.
    double switch_distance = 0.0;
    const Waypoint& wp = route_.target();
    if (const Waypoint* next = route_.next(); next && !wp.fly_over) {
        const Vec2 next_leg = next->position.xy() - wp.position.xy();
        const double turn =
            std::min(std::abs(angle_right(lp.direction, next_leg.normalized())), deg2rad(150.0));
        const double lag_distance = 1.5 * std::abs(speed_) * yaw_lag() * (2.0 * turn / kPi);
        switch_distance = std::min({turning_radius() * std::tan(0.5 * turn) + lag_distance,
                                    0.5 * lp.length, 0.5 * next_leg.length()});
    }
    if (stops_at_target()) switch_distance = arrival_radius();
    if (lp.remaining() <= switch_distance) {
        if (!route_.advance(position_, p_.water_level)) {
            finish_route(end);
            return;
        }
        lp = route_.progress(pos);
    }

    set_speed(route_.target().speed);
    const Vec2 a = route_.leg_start();
    // Far behind the start of the leg: head for the start point first.
    const double course = (pos - a).length() > lookahead() && lp.along < 0.0
                              ? heading_of(a - pos)
                              : line_course(a, heading_of(lp.direction));
    cmd.turn_rate = steer_to(course, 0.0);
    cmd.speed = speed_target_;
    if (stops_at_target()) cmd.speed = std::min(cmd.speed, stopping_speed(lp.remaining()));
}

void ShipAgent::finish_route(EndBehavior behavior) {
    const Waypoint last = route_.target();
    const Vec2 leg = route_.leg_end() - route_.leg_start();
    const double radius = route_.route().loiter_radius;
    route_.clear();
    if (behavior == EndBehavior::ContinueStraight) {
        mode_ = FlightMode::Cruise;
        course_target_ = leg.length() > 1.0 ? heading_of(leg) : heading_;
        course_anchor_ = last.position.xy();
        return;
    }
    if (behavior == EndBehavior::Loiter && radius > 0.0) {
        loiter(last.position, radius, true, AltitudeRef::Absolute);
        return;
    }
    // Stop and keep station at the last waypoint.
    mode_ = FlightMode::Hover;
    hold_point_ = last.position.xy();
    hold_here_ = false;
}

void ShipAgent::keep_station(Command& cmd, const Vec2& point) const {
    const Vec2 to = point - position_.xy();
    const double distance = to.length();
    if (distance <= arrival_radius()) {
        cmd.speed = 0.0;
        cmd.turn_rate = steer_to(heading_, 0.0);  // stop turning
        return;
    }
    const double bearing = heading_of(to);
    if (distance > lookahead()) {  // far off (moved away): sail back
        cmd.speed = std::min(p_.cruise_speed, stopping_speed(distance));
        cmd.turn_rate = steer_to(bearing, 0.0);
        return;
    }
    // Close: ahead or slowly astern, whichever end faces the point, slow enough to turn
    // onto it (half the radius that passes through it); pivot while it is abeam.
    const double error = wrap_pi(bearing - heading_);
    const double facing = std::cos(error);
    const double off_axis = std::abs(std::sin(error));
    const double turnable = p_.pivot_rate > 0.0
                                ? 0.25 * p_.pivot_rate * distance / std::max(off_axis, 1e-3)
                                : p_.cruise_speed;
    const double limit = facing >= 0.0 ? p_.cruise_speed : kStationSpeed;
    cmd.speed = std::min({limit, stopping_speed(distance), turnable}) * facing;
    cmd.turn_rate = steer_to(facing >= 0.0 ? bearing : bearing + kPi, 0.0);
}

void ShipAgent::guide_circle(Command& cmd) const {
    const double sign = loiter_clockwise_ ? 1.0 : -1.0;
    const Vec2 from_center = position_.xy() - loiter_center_.xy();
    const double distance = from_center.length();
    const Vec2 out = distance > 1e-6 ? from_center / distance : Vec2{1.0, 0.0};
    // Along the circle, turned in by the line-of-sight angle to it.
    const double offset = distance - loiter_radius_;
    const double course = heading_of(right_of(out) * sign) + sign * std::atan(offset / lookahead());
    const double on_circle = std::clamp(1.0 - std::abs(offset) / lookahead(), 0.0, 1.0);
    cmd.turn_rate = steer_to(course, sign * on_circle * std::abs(speed_) / loiter_radius_);
}

void ShipAgent::guide_formation(Command& cmd, const SlotReference& ref) const {
    const Vec2 pos = position_.xy();
    const Vec2 err = ref.position.xy() - pos;
    const double along = dot(err, heading_vector(ref.heading));
    const double leader_speed = ref.velocity.xy().length();
    if (leader_speed < 0.5) {  // the leader stopped: keep station at the slot
        keep_station(cmd, ref.position.xy());
        return;
    }
    if (err.length() > lookahead() + turning_radius()) {
        // Join: head for where the slot will be, a bit faster (or slower) than the leader.
        const double t_go = std::clamp(err.length() / std::max(std::abs(speed_), 1.0), 0.0, 600.0);
        const Vec2 aim = ref.position.xy() + ref.velocity.xy() * t_go;
        cmd.turn_rate = steer_to(heading_of(aim - pos), 0.0);
        const double closing = (along >= 0.0 ? 0.25 : -0.25) * p_.max_speed;
        cmd.speed = std::clamp(leader_speed + closing, 0.5 * leader_speed, p_.max_speed);
    } else {
        // Station: follow the slot line, anticipate the leader's turns, match its speed.
        cmd.turn_rate = steer_to(line_course(ref.position.xy(), ref.heading), ref.turn_rate);
        cmd.speed = std::clamp(leader_speed + kFormationAlongGain * along, 0.0, p_.max_speed);
    }
    // Never closer to the leader than the minimum separation.
    const double min_sep =
        slot_.min_separation > 0.0 ? slot_.min_separation : p_.formation_min_separation;
    const Vec2 to_leader = leader_.position.xy() - pos;
    if (to_leader.length() < min_sep && dot(to_leader, heading_vector(heading_)) > 0.0)
        cmd.speed = std::min(cmd.speed, 0.5 * leader_speed);
}

void ShipAgent::integrate(const Command& cmd, double dt) {
    if (aground_) {
        speed_ = 0.0;
        turn_rate_ = 0.0;
        thrust_ = 0.0;
        return;
    }
    const double mass = kAddedMass * p_.displacement;
    const double rate_limit = max_turn_rate(std::abs(speed_));
    const double turning = rate_limit > 0.0 ? std::clamp(turn_rate_ / rate_limit, -1.0, 1.0) : 0.0;
    // A full-rudder turn costs turn_speed_loss of the speed: extra resistance ~ turning^2.
    const double loss = std::clamp(p_.turn_speed_loss, 0.0, 0.9);
    const double turn_drag = 1.0 + (1.0 / ((1.0 - loss) * (1.0 - loss)) - 1.0) * turning * turning;

    // Speed: thrust for the commanded speed, limited by the propulsion, against resistance.
    const double target = std::clamp(cmd.speed, -p_.max_astern_speed, p_.max_speed);
    const double wanted = resistance(target) * turn_drag + mass * (target - speed_) / kSpeedTau;
    const double ahead = thrust_available(std::abs(speed_));
    const double astern = p_.astern_thrust_factor * p_.bollard_pull;
    thrust_ = lag(thrust_, std::clamp(wanted, -astern, ahead), p_.propulsion_time_constant, dt);
    speed_ += (thrust_ - resistance(speed_) * turn_drag) / mass * dt;

    // Steering: the turn rate follows the command with the Nomoto lag, within the limit.
    turn_rate_ = lag(turn_rate_, std::clamp(cmd.turn_rate, -rate_limit, rate_limit), yaw_lag(), dt);
    heading_ = wrap_2pi(heading_ + turn_rate_ * dt);

    // The bow points inside the turn (drift angle); the ship moves along heading - drift.
    drift_ = drift_angle(turn_rate_);
    position_ += Vec3{std::sin(heading_ - drift_), std::cos(heading_ - drift_), 0.0} * (speed_ * dt);
    position_.z = p_.water_level;

    // Outward heel in turns, growing with the square of the speed.
    const double speed_ratio = speed_ / std::max(p_.max_speed, 1e-3);
    heel_ = lag(heel_, -p_.heel_max * turning * speed_ratio * speed_ratio, kHeelTau, dt);
    phase_ = std::abs(speed_) < 0.2 && std::abs(target) < 0.2 ? FlightPhase::Parked
                                                              : FlightPhase::Underway;
}

void ShipAgent::update_output() {
    const double track = heading_ - drift_;
    out_.position = position_;
    out_.velocity = Vec3{std::sin(track), std::cos(track), 0.0} * speed_;
    out_.attitude.heading = heading_;
    out_.attitude.roll = heel_;
    out_.attitude.pitch = 0.0;
    if (p_.roll_amplitude > 0.0 && p_.roll_period > 0.0)
        out_.attitude.roll += p_.roll_amplitude * std::sin(kTwoPi * time_ / p_.roll_period);
    if (p_.pitch_amplitude > 0.0 && p_.pitch_period > 0.0)
        out_.attitude.pitch = p_.pitch_amplitude * std::sin(kTwoPi * time_ / p_.pitch_period + 1.0);
    out_.airspeed = std::abs(speed_);  // speed through the water
    out_.ground_speed = std::abs(speed_);
    out_.vertical_speed = 0.0;
    out_.track = std::abs(speed_) > 0.1 ? wrap_2pi(speed_ >= 0.0 ? track : track + kPi) : heading_;
    out_.flight_path_angle = 0.0;
    out_.angle_of_attack = 0.0;
    out_.load_factor = 1.0;
    out_.throttle = thrust_ >= 0.0 ? thrust_ / std::max(thrust_available(std::abs(speed_)), 1.0)
                                   : thrust_ / std::max(p_.astern_thrust_factor * p_.bollard_pull, 1.0);
    out_.mach = 0.0;
    out_.height_above_terrain = terrain_.valid() ? p_.water_level - terrain_.current() : kUnknown;
    out_.on_ground = aground_;
    out_.speed_brake = false;
    out_.flaps = false;
    out_.gear_down = false;
    out_.phase = aground_ ? FlightPhase::TerrainContact : phase_;
}

// --- Helpers --------------------------------------------------------------------------

double ShipAgent::max_turn_rate(double speed) const {
    return std::max(speed / std::max(turning_radius(), 1.0), p_.pivot_rate);
}

double ShipAgent::thrust_available(double speed) const {
    const double power_thrust =
        p_.propulsive_efficiency * p_.propulsion_power / std::max(speed, 0.1);
    return std::min(p_.bollard_pull, power_thrust);
}

double ShipAgent::resistance(double speed) const {
    // Quadratic hull resistance, sized so that full thrust holds max_speed.
    const double k = thrust_available(p_.max_speed) / (p_.max_speed * p_.max_speed);
    return k * speed * std::abs(speed);
}

double ShipAgent::steer_to(double course, double feedforward) const {
    // PD on the heading for the first-order steering response: natural frequency
    // kHeadingBandwidth / T, damping kHeadingDamping. In a turn the bow points inside the
    // track by the drift angle.
    const double rate_limit = max_turn_rate(std::abs(speed_));
    const double ff = std::clamp(feedforward, -rate_limit, rate_limit);
    const double heading = course + drift_angle(ff);
    const double t = yaw_lag();
    const double kp = kHeadingBandwidth * kHeadingBandwidth / t;
    const double kd = std::max(2.0 * kHeadingDamping * kHeadingBandwidth - 1.0, 0.0);
    return ff + kp * wrap_pi(heading - heading_) - kd * (turn_rate_ - ff);
}

double ShipAgent::line_course(const Vec2& point, double course) const {
    const double crosstrack = dot(position_.xy() - point, right_of(heading_vector(course)));
    return course - std::atan(crosstrack / lookahead());
}

double ShipAgent::lookahead() const {
    if (p_.path_lookahead > 0.0) return p_.path_lookahead;
    return std::max(kLookahead * p_.steering_time_constant * p_.length, turning_radius());
}

double ShipAgent::yaw_lag() const {
    return std::clamp(p_.steering_time_constant * p_.length / std::max(std::abs(speed_), 1.0),
                      5.0, 120.0);
}

double ShipAgent::drift_angle(double turn_rate) const {
    const double rate_limit = max_turn_rate(std::abs(speed_));
    const double turning = rate_limit > 0.0 ? std::clamp(turn_rate / rate_limit, -1.0, 1.0) : 0.0;
    return p_.drift_angle_max * turning * std::min(1.0, std::abs(speed_) / 2.0);
}

double ShipAgent::stopping_speed(double distance) const {
    return std::sqrt(2.0 * p_.brake_deceleration * std::max(distance - 0.5 * arrival_radius(), 0.0));
}

double ShipAgent::arrival_radius() const { return std::max(10.0, 0.1 * p_.length); }

}  // namespace fm
