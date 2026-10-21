#include "fm/flight_agent.hpp"

#include <cmath>
#include <utility>

namespace fm {

namespace {
AgentVariant make_impl(const VehicleParams& params, const InitialConditions& ic) {
    return std::visit(
        [&](const auto& p) -> AgentVariant {
            using Agent =
                typename detail::agent_for<std::decay_t<decltype(p)>, AgentVariant>::type;
            return AgentVariant(std::in_place_type<Agent>, p, ic);
        },
        params);
}

// Commands with non-finite values are ignored: they would poison the state for good.
// NaN stays allowed where it means "keep" or "default" (speeds, radii, headings of leaders).
bool usable_altitude(double z, AltitudeRef ref) {
    return ref == AltitudeRef::Keep || std::isfinite(z);
}
bool usable(const Waypoint& w) {
    return is_finite(w.position.xy()) && usable_altitude(w.position.z, w.altitude_ref);
}
bool usable(const Runway& r) { return is_finite(r.start) && std::isfinite(r.heading); }
bool usable(const Route& r) {
    for (const Waypoint& w : r.waypoints)
        if (!usable(w)) return false;
    return !std::isinf(r.loiter_radius) && (!r.runway || usable(*r.runway));
}
// Speed commands in CAS or Mach: models without their own support get the true airspeed at
// their current altitude.
template <typename M>
void set_speed_in(M& m, double speed, SpeedRef ref) {
    if constexpr (requires { m.set_speed(speed, ref); }) {
        m.set_speed(speed, ref);
    } else {
        m.set_speed(std::isfinite(speed) ? true_airspeed(speed, ref, isa(m.state().position.z))
                                         : speed);
    }
}

bool usable(const TakeoffPlan& p) {
    return !std::isinf(p.climb_height) && (!p.runway || usable(*p.runway)) &&
           (!p.then || usable(*p.then));
}
}  // namespace

FlightAgent::FlightAgent(const VehicleParams& params, const InitialConditions& ic)
    : impl_(make_impl(params, ic)) {}

VehicleKind FlightAgent::kind() const {
    return std::visit([](const auto& m) { return std::decay_t<decltype(m)>::kind; }, impl_);
}

void FlightAgent::cruise() {
    std::visit([](auto& m) { m.cruise(); }, impl_);
}

void FlightAgent::hold(double course, double altitude, double speed, AltitudeRef altitude_ref,
                       SpeedRef speed_ref) {
    if (!std::isfinite(course) || !usable_altitude(altitude, altitude_ref)) return;
    std::visit(
        [&](auto& m) {
            if (speed_ref == SpeedRef::True) {
                m.hold(course, altitude, speed, altitude_ref);
            } else {
                m.hold(course, altitude, kUnknown, altitude_ref);  // speed below
                set_speed_in(m, speed, speed_ref);
            }
        },
        impl_);
}

void FlightAgent::set_speed(double speed, SpeedRef speed_ref) {
    std::visit([&](auto& m) { set_speed_in(m, speed, speed_ref); }, impl_);
}

void FlightAgent::go_to(const Vec3& point, const GoToOptions& options) {
    if (!is_finite(point.xy()) || !usable_altitude(point.z, options.altitude_ref)) return;
    std::visit([&](auto& m) { m.go_to(point, options); }, impl_);
}

void FlightAgent::follow_route(Route route) {
    if (!usable(route)) return;
    std::visit([&](auto& m) { m.follow_route(std::move(route)); }, impl_);
}

void FlightAgent::loiter(const Vec3& center, double radius, bool clockwise,
                         AltitudeRef altitude_ref) {
    if (!is_finite(center.xy()) || !usable_altitude(center.z, altitude_ref) || std::isinf(radius))
        return;
    std::visit([&](auto& m) { m.loiter(center, radius, clockwise, altitude_ref); }, impl_);
}

bool FlightAgent::takeoff(const TakeoffPlan& plan) {
    if (!usable(plan)) return false;
    return std::visit([&](auto& m) { return m.takeoff(plan); }, impl_);
}

void FlightAgent::hover() {
    std::visit([](auto& m) { m.hover(); }, impl_);
}

bool FlightAgent::land() {
    return std::visit([](auto& m) { return m.land(); }, impl_);
}

bool FlightAgent::land(const Runway& runway) {
    if (!usable(runway)) return false;
    return std::visit([&](auto& m) { return m.land(runway); }, impl_);
}

bool FlightAgent::land_on(const DeckSpot& spot) {
    if (!is_finite(spot.offset) || !std::isfinite(spot.heading)) return false;
    return std::visit(
        [&](auto& m) {
            if constexpr (requires { { m.land_on(spot) } -> std::same_as<bool>; })
                return m.land_on(spot);
            else
                return false;  // the model cannot land on a deck
        },
        impl_);
}

void FlightAgent::set_fuel(double fuel) {
    if (!std::isfinite(fuel)) return;
    std::visit(
        [&](auto& m) {
            if constexpr (requires { m.set_fuel(fuel); }) m.set_fuel(fuel);
        },
        impl_);
}

bool FlightAgent::tracks_leader() const {
    return std::visit(
        [](const auto& m) {
            if constexpr (requires { { m.deck_operation() } -> std::same_as<bool>; })
                if (m.deck_operation()) return true;
            return m.mode() == FlightMode::Formation;
        },
        impl_);
}

void FlightAgent::join_formation(const FormationSlot& slot) {
    if (!is_finite(slot.offset) || std::isinf(slot.min_separation)) return;
    std::visit([&](auto& m) { m.join_formation(slot); }, impl_);
}

void FlightAgent::set_leader_state(const LeaderState& leader) {
    if (!is_finite(leader.position)) return;  // no update: followers time out if it lasts
    LeaderState l = leader;
    if (!is_finite(l.velocity)) l.velocity = {};
    if (std::isinf(l.heading)) l.heading = kUnknown;
    std::visit([&](auto& m) { m.set_leader_state(l); }, impl_);
}

bool FlightAgent::relocate(const Vec3& position, double heading) {
    constexpr double kMaxHorizontal = 2.0e7;  // m, half way around the Earth
    constexpr double kMaxAltitude = 1.0e5;    // m, edge of space
    const bool finite = std::isfinite(position.x) && std::isfinite(position.y) &&
                        std::isfinite(position.z) && !std::isinf(heading);
    if (!finite || std::abs(position.x) > kMaxHorizontal ||
        std::abs(position.y) > kMaxHorizontal || std::abs(position.z) > kMaxAltitude)
        return false;
    return std::visit([&](auto& m) { return m.relocate(position, heading); }, impl_);
}

void FlightAgent::step(double dt, const Environment& env) {
    std::visit([&](auto& m) { m.step(dt, env); }, impl_);
}

const FlightState& FlightAgent::state() const {
    return std::visit([](const auto& m) -> const FlightState& { return m.state(); }, impl_);
}

FlightMode FlightAgent::mode() const {
    return std::visit([](const auto& m) { return m.mode(); }, impl_);
}

LeaderState FlightAgent::as_leader() const {
    const FlightState& s = state();
    return {s.position, s.velocity, s.attitude.heading, true};
}

InitialConditions FlightAgent::snapshot() const {
    const FlightState& s = state();
    InitialConditions ic;
    ic.position = s.position;
    ic.heading = s.attitude.heading;
    ic.speed = s.airspeed;
    ic.on_ground = s.on_ground;
    ic.fuel = s.fuel;
    if (s.on_ground) {
        // On the ground the initial z is the ground elevation.
        ic.position.z -= std::visit([](const auto& m) { return double(m.ground_offset()); }, impl_);
    }
    return ic;
}

}  // namespace fm
