#include "fm/flight_agent.hpp"

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
}  // namespace

FlightAgent::FlightAgent(const VehicleParams& params, const InitialConditions& ic)
    : impl_(make_impl(params, ic)) {}

VehicleKind FlightAgent::kind() const {
    return std::visit([](const auto& m) { return std::decay_t<decltype(m)>::kind; }, impl_);
}

void FlightAgent::cruise() {
    std::visit([](auto& m) { m.cruise(); }, impl_);
}

void FlightAgent::hold(double course, double altitude, double speed, AltitudeRef altitude_ref) {
    std::visit([&](auto& m) { m.hold(course, altitude, speed, altitude_ref); }, impl_);
}

void FlightAgent::set_speed(double speed) {
    std::visit([&](auto& m) { m.set_speed(speed); }, impl_);
}

void FlightAgent::go_to(const Vec3& point, const GoToOptions& options) {
    std::visit([&](auto& m) { m.go_to(point, options); }, impl_);
}

void FlightAgent::follow_route(Route route) {
    std::visit([&](auto& m) { m.follow_route(std::move(route)); }, impl_);
}

void FlightAgent::loiter(const Vec3& center, double radius, bool clockwise,
                         AltitudeRef altitude_ref) {
    std::visit([&](auto& m) { m.loiter(center, radius, clockwise, altitude_ref); }, impl_);
}

bool FlightAgent::takeoff(const TakeoffPlan& plan) {
    return std::visit([&](auto& m) { return m.takeoff(plan); }, impl_);
}

void FlightAgent::hover() {
    std::visit([](auto& m) { m.hover(); }, impl_);
}

bool FlightAgent::land() {
    return std::visit([](auto& m) { return m.land(); }, impl_);
}

bool FlightAgent::land(const Runway& runway) {
    return std::visit([&](auto& m) { return m.land(runway); }, impl_);
}

void FlightAgent::join_formation(const FormationSlot& slot) {
    std::visit([&](auto& m) { m.join_formation(slot); }, impl_);
}

void FlightAgent::set_leader_state(const LeaderState& leader) {
    std::visit([&](auto& m) { m.set_leader_state(leader); }, impl_);
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
    if (s.on_ground) {
        // On the ground the initial z is the ground elevation.
        ic.position.z -= std::visit([](const auto& m) { return double(m.ground_offset()); }, impl_);
    }
    return ic;
}

}  // namespace fm
