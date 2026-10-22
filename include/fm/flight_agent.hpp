// Single entry point for all vehicle models. FlightAgent owns one concrete model and
// forwards commands to it; the host never needs to know which model is running.
//
// Adding a vehicle type (details in docs/EXTENDING.md):
//   1. params struct  -> fm::VehicleParams    (vehicle_params.hpp)
//   2. VehicleKind value                       (vehicle_params.hpp)
//   3. agent class satisfying fm::FlightModel  (your .hpp/.cpp)
//   4. agent class    -> fm::AgentVariant      (below)
// Nothing else dispatches on concrete types.
#pragma once

#include <concepts>
#include <span>
#include <type_traits>
#include <variant>

#include "fm/fixed_wing.hpp"
#include "fm/rotorcraft.hpp"
#include "fm/ship.hpp"

namespace fm {

// Interface every vehicle model provides. Commands a vehicle cannot perform are accepted
// and ignored (or answered with false), never an error.
template <typename T>
concept FlightModel = requires(T m, const T cm, double d, const Vec3& p, const Environment& env,
                               Route r, const GoToOptions& go, const TakeoffPlan& plan,
                               const Runway& runway, const FormationSlot& slot,
                               const LeaderState& leader) {
    typename T::params_type;
    { T::kind } -> std::convertible_to<VehicleKind>;
    m.cruise();
    m.hold(d, d, d, AltitudeRef::Absolute);
    m.set_speed(d);
    m.go_to(p, go);
    m.follow_route(std::move(r));
    m.loiter(p, d, true, AltitudeRef::Absolute);
    { m.takeoff(plan) } -> std::same_as<bool>;
    m.hover();
    { m.land() } -> std::same_as<bool>;
    { m.land(runway) } -> std::same_as<bool>;
    m.join_formation(slot);
    m.set_leader_state(leader);
    { m.relocate(p, d) } -> std::same_as<bool>;
    m.step(d, env);
    { cm.state() } -> std::same_as<const FlightState&>;
    { cm.mode() } -> std::same_as<FlightMode>;
    { cm.ground_offset() } -> std::convertible_to<double>;
};

// The registered vehicle models. Order does not matter.
using AgentVariant = std::variant<FixedWingAgent, RotorcraftAgent, ShipAgent>;

namespace detail {
template <typename V>
struct all_models;
template <typename... A>
struct all_models<std::variant<A...>> : std::bool_constant<(FlightModel<A> && ...)> {};

// The agent whose params_type is P (void when none is registered).
template <typename P, typename V>
struct agent_for {
    using type = void;
};
template <typename P, typename First, typename... Rest>
struct agent_for<P, std::variant<First, Rest...>> {
    using type = std::conditional_t<std::is_same_v<typename First::params_type, P>, First,
                                    typename agent_for<P, std::variant<Rest...>>::type>;
};

template <typename V>
struct all_params_registered;
template <typename... P>
struct all_params_registered<std::variant<P...>>
    : std::bool_constant<(!std::is_void_v<typename agent_for<P, AgentVariant>::type> && ...)> {};
}  // namespace detail

static_assert(detail::all_models<AgentVariant>::value,
              "every agent in AgentVariant must satisfy fm::FlightModel");
static_assert(detail::all_params_registered<VehicleParams>::value,
              "every params type in VehicleParams needs an agent in AgentVariant");

class FlightAgent {
public:
    FlightAgent(const VehicleParams& params, const InitialConditions& ic);
    // Construct from any registered params type, e.g. FlightAgent(presets::multirotor(), ic).
    template <typename P>
        requires(!std::is_void_v<typename detail::agent_for<std::decay_t<P>, AgentVariant>::type>)
    FlightAgent(P&& params, const InitialConditions& ic)
        : impl_(std::in_place_type<typename detail::agent_for<std::decay_t<P>, AgentVariant>::type>,
                std::forward<P>(params), ic) {}

    VehicleKind kind() const;

    // --- Commands (see the concrete agents for details) ------------------------------
    // Commands with non-finite positions, courses or altitudes are ignored (takeoff/land
    // return false); NaN speeds keep the current speed. Bad step inputs (NaN wind, infinite
    // terrain, non-finite dt) are ignored too, so host data never poisons the state.
    void cruise();
    // `speed` in `speed_ref`: TAS (m/s), CAS (m/s, what the airspeed indicator shows) or
    // Mach. Fixed-wing aircraft hold CAS / Mach targets through altitude changes; other
    // vehicles convert them once at the current altitude.
    void hold(double course, double altitude, double speed,
              AltitudeRef altitude_ref = AltitudeRef::Absolute,
              SpeedRef speed_ref = SpeedRef::True);
    void set_speed(double speed, SpeedRef speed_ref = SpeedRef::True);
    void go_to(const Vec3& point, const GoToOptions& options = {});
    void follow_route(Route route);
    void loiter(const Vec3& center, double radius = 0.0, bool clockwise = true,
                AltitudeRef altitude_ref = AltitudeRef::Absolute);
    // Fixed-wing: taxi along plan.taxi, line up on plan.runway / plan.heading and take off.
    bool takeoff(const TakeoffPlan& plan = {});
    // Starts the run of a takeoff planned with TakeoffPlan::hold. Others: false.
    bool release_takeoff();
    // Fixed-wing on the ground: taxi through `path` (z ignored) and stop at its last point,
    // arriving along `final_heading` (rad, NaN = any), e.g. into a hangar after landing.
    // Others, or non-finite points: false.
    bool taxi(std::span<const Vec3> path, double final_heading = kUnknown);
    // Rotorcraft holds position; fixed-wing aircraft orbit the current position; ships stop
    // and keep station.
    void hover();
    // Rotorcraft: vertical landing here. Fixed-wing: needs a runway (returns false).
    // Ships: false (use hover() to stop).
    bool land();
    // Fixed-wing: approach and landing on the runway (`options`: straight-in or overhead
    // pattern, where the landing roll ends). Rotorcraft: fly to the runway start at the
    // current altitude and land vertically there. Ships: false.
    bool land(const Runway& runway, const LandingOptions& options = {});
    // Rotorcraft: land on a spot of a moving deck (ship flight deck) and ride on it until
    // takeoff(), which lifts off with the deck's velocity. The carrier's state must arrive
    // with set_leader_state() every step (fm::ecs::land_on does it). Other vehicles (and
    // rotorcraft on the ground): false.
    bool land_on(const DeckSpot& spot);
    void join_formation(const FormationSlot& slot);
    void set_leader_state(const LeaderState& leader);
    // Whether the model uses set_leader_state() now (formation, deck operations).
    bool tracks_leader() const;
    // Leading a formation whose widest follower is `lateral_extent` m to the side: fixed-wing
    // leaders limit their bank and speed so the followers can stay in their slots. Call
    // every step (fm::ecs::follow does it); expires after 2 s. 0 = not leading.
    // `rejoining`: a follower is still far from its slot; the leader slows down for it.
    void set_formation_lead(double lateral_extent, bool rejoining = false);
    // Vehicles around this one, every step before step(), for collision avoidance
    // (fixed-wing: climbs or descends away from a predicted conflict). Leave out its
    // formation partners. The Flecs module does it for all agents.
    void set_traffic(std::span<const Traffic> traffic);

    // The host moved the vehicle (teleport, respawn, editor move): continue from `position`
    // and `heading` (rad, NaN = keep) instead of jumping back. Speed is kept.
    //  - Corrections (up to 50 m, 20 m vertically, 10 deg) only move the vehicle.
    //  - Larger jumps restart the current task here: the route leg starts at the new
    //    position, straight-and-level flight continues on the new line (holding the new
    //    altitude), positions held "here" move along, a landing approach starts over,
    //    followers rejoin.
    //  - On the ground: placed at a new altitude more than 5 m above the ground it starts
    //    flying (fixed-wing at cruise speed at least), otherwise it stays on the ground.
    //    In the air: placed below the ground it is lifted just above it.
    // The ground at the new position is taken from the next step's terrain input.
    // Returns false and changes nothing for non-finite or absurd input (|z| > 100 km).
    bool relocate(const Vec3& position, double heading = kUnknown);

    // --- Simulation -----------------------------------------------------------------
    void step(double dt, const Environment& env);
    const FlightState& state() const;
    FlightMode mode() const;

    // Fuel on board in kg (refuelling): models with fuel (fixed-wing with fuel_capacity).
    void set_fuel(double fuel);

    // State of this agent as seen by its followers.
    LeaderState as_leader() const;
    // Starting conditions that reproduce the current state; used to switch to another
    // model or parameter set (e.g. a level-of-detail change) without a jump.
    InitialConditions snapshot() const;

    // The concrete model, or nullptr: agent.as<FixedWingAgent>().
    template <typename A>
    A* as() { return std::get_if<A>(&impl_); }
    template <typename A>
    const A* as() const { return std::get_if<A>(&impl_); }
    FixedWingAgent* fixed_wing() { return as<FixedWingAgent>(); }
    RotorcraftAgent* rotorcraft() { return as<RotorcraftAgent>(); }
    ShipAgent* ship() { return as<ShipAgent>(); }
    const FixedWingAgent* fixed_wing() const { return as<FixedWingAgent>(); }
    const RotorcraftAgent* rotorcraft() const { return as<RotorcraftAgent>(); }
    const ShipAgent* ship() const { return as<ShipAgent>(); }

    // Calls f(model) with the concrete model, for features only one type has.
    template <typename F>
    decltype(auto) visit(F&& f) { return std::visit(std::forward<F>(f), impl_); }
    template <typename F>
    decltype(auto) visit(F&& f) const { return std::visit(std::forward<F>(f), impl_); }

private:
    AgentVariant impl_;
};

}  // namespace fm
