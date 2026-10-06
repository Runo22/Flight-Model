#include "fm/flight_agent.hpp"

#include <utility>

namespace fm {

namespace {
std::variant<FixedWingAgent, RotorcraftAgent> make_impl(const VehicleParams& params,
                                                        const InitialConditions& ic) {
    if (const auto* fw = std::get_if<FixedWingParams>(&params)) return FixedWingAgent(*fw, ic);
    return RotorcraftAgent(std::get<RotorcraftParams>(params), ic);
}
}  // namespace

FlightAgent::FlightAgent(const VehicleParams& params, const InitialConditions& ic)
    : impl_(make_impl(params, ic)) {}

FlightAgent::FlightAgent(FixedWingParams params, const InitialConditions& ic)
    : impl_(std::in_place_type<FixedWingAgent>, std::move(params), ic) {}

FlightAgent::FlightAgent(RotorcraftParams params, const InitialConditions& ic)
    : impl_(std::in_place_type<RotorcraftAgent>, std::move(params), ic) {}

VehicleKind FlightAgent::kind() const {
    return std::holds_alternative<FixedWingAgent>(impl_) ? VehicleKind::FixedWing
                                                         : VehicleKind::Rotorcraft;
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
    if (auto* rc = rotorcraft()) {
        rc->hover();
        return;
    }
    fixed_wing()->loiter(state().position, 0.0, true, AltitudeRef::Absolute);
}

bool FlightAgent::land() {
    if (auto* rc = rotorcraft()) return rc->land();
    return false;
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
        const double gear = fixed_wing() ? fixed_wing()->params().gear_height
                                         : rotorcraft()->params().gear_height;
        ic.position.z -= gear;  // on the ground the initial z is the ground elevation
    }
    return ic;
}

}  // namespace fm
