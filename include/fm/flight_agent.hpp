// Single entry point for all vehicle models. FlightAgent owns one concrete model and
// forwards commands to it; the host never needs to know which model is running.
#pragma once

#include <concepts>
#include <variant>

#include "fm/fixed_wing.hpp"
#include "fm/rotorcraft.hpp"

namespace fm {

// Interface every vehicle model provides. A new model (for example a 6-DOF fixed-wing or a
// cheap kinematic level-of-detail model) satisfies this concept and joins the variant below.
template <typename T>
concept FlightModel = requires(T m, const T cm, double d, const Vec3& p, const Environment& env,
                               Route r, const GoToOptions& go, const TakeoffPlan& plan,
                               const FormationSlot& slot, const LeaderState& leader) {
    m.cruise();
    m.hold(d, d, d, AltitudeRef::Absolute);
    m.set_speed(d);
    m.go_to(p, go);
    m.follow_route(std::move(r));
    m.loiter(p, d, true, AltitudeRef::Absolute);
    { m.takeoff(plan) } -> std::same_as<bool>;
    m.join_formation(slot);
    m.set_leader_state(leader);
    m.step(d, env);
    { cm.state() } -> std::same_as<const FlightState&>;
    { cm.mode() } -> std::same_as<FlightMode>;
};

static_assert(FlightModel<FixedWingAgent>);
static_assert(FlightModel<RotorcraftAgent>);

enum class VehicleKind { FixedWing, Rotorcraft };

class FlightAgent {
public:
    FlightAgent(const VehicleParams& params, const InitialConditions& ic);
    FlightAgent(FixedWingParams params, const InitialConditions& ic);
    FlightAgent(RotorcraftParams params, const InitialConditions& ic);

    VehicleKind kind() const;

    // --- Commands (see FixedWingAgent / RotorcraftAgent for details) ----------------
    void cruise();
    void hold(double course, double altitude, double speed,
              AltitudeRef altitude_ref = AltitudeRef::Absolute);
    void set_speed(double speed);
    void go_to(const Vec3& point, const GoToOptions& options = {});
    void follow_route(Route route);
    void loiter(const Vec3& center, double radius = 0.0, bool clockwise = true,
                AltitudeRef altitude_ref = AltitudeRef::Absolute);
    bool takeoff(const TakeoffPlan& plan = {});
    // Rotorcraft holds position; fixed-wing aircraft orbit the current position.
    void hover();
    // Rotorcraft vertical landing. Fixed-wing landing is not implemented yet (returns false).
    bool land();
    void join_formation(const FormationSlot& slot);
    void set_leader_state(const LeaderState& leader);

    // --- Simulation -----------------------------------------------------------------
    void step(double dt, const Environment& env);
    const FlightState& state() const;
    FlightMode mode() const;

    // State of this agent as seen by its followers.
    LeaderState as_leader() const;
    // Starting conditions that reproduce the current state; used to switch to another
    // model or parameter set (e.g. a level-of-detail change) without a jump.
    InitialConditions snapshot() const;

    FixedWingAgent* fixed_wing() { return std::get_if<FixedWingAgent>(&impl_); }
    RotorcraftAgent* rotorcraft() { return std::get_if<RotorcraftAgent>(&impl_); }
    const FixedWingAgent* fixed_wing() const { return std::get_if<FixedWingAgent>(&impl_); }
    const RotorcraftAgent* rotorcraft() const { return std::get_if<RotorcraftAgent>(&impl_); }

private:
    std::variant<FixedWingAgent, RotorcraftAgent> impl_;
};

}  // namespace fm
