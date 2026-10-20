// Multirotor agent: thrust-vector dynamics (the vehicle tilts to accelerate, quadratic drag
// sets the top speed) with velocity-based guidance for hover, routes, landing and formation.
#pragma once

#include <array>
#include <optional>

#include "fm/atmosphere.hpp"
#include "fm/flight_state.hpp"
#include "fm/formation.hpp"
#include "fm/relocation.hpp"
#include "fm/route.hpp"
#include "fm/route_tracker.hpp"
#include "fm/terrain.hpp"
#include "fm/vehicle_params.hpp"

namespace fm {

class RotorcraftAgent {
public:
    using params_type = RotorcraftParams;
    static constexpr VehicleKind kind = VehicleKind::Rotorcraft;

    RotorcraftAgent(RotorcraftParams params, const InitialConditions& ic);

    // --- Commands -------------------------------------------------------------------
    // Straight and level on the current track; hovers when (almost) stationary.
    void cruise();
    void hold(double course, double altitude, double speed,
              AltitudeRef altitude_ref = AltitudeRef::Absolute);
    void set_speed(double speed);
    // Hold the current position.
    void hover();
    void go_to(const Vec3& point, const GoToOptions& options = {});
    void follow_route(Route route);
    // Orbit `center`; radius <= 0 hovers above it.
    void loiter(const Vec3& center, double radius = 0.0, bool clockwise = true,
                AltitudeRef altitude_ref = AltitudeRef::Absolute);
    // Vertical takeoff to plan.climb_height (or params.takeoff_height), then plan.then.
    bool takeoff(const TakeoffPlan& plan);
    // Vertical landing at the current position.
    bool land();
    // Fly to the runway start at the current altitude and land vertically there.
    bool land(const Runway& runway);
    void join_formation(const FormationSlot& slot);
    void set_leader_state(const LeaderState& leader);
    // The host moved the vehicle: continue from `position` / `heading` (NaN = keep), see
    // FlightAgent::relocate. The velocity turns with the vehicle; the task is kept.
    bool relocate(const Vec3& position, double heading);

    // --- Simulation -----------------------------------------------------------------
    void step(double dt, const Environment& env);

    const FlightState& state() const { return out_; }
    FlightMode mode() const { return mode_; }
    const RotorcraftParams& params() const { return p_; }
    // Height of the reference point above the ground when standing on it.
    double ground_offset() const { return p_.gear_height; }
    const RouteTracker& route() const { return route_; }

private:
    struct Command {
        Vec3 velocity{};      // desired ground velocity, ENU
        double yaw = 0.0;
        bool motors_off = false;
    };

    Command guidance(const SlotReference& slot_ref);
    void guide_route(Command& cmd);
    void finish_route(EndBehavior behavior);
    void finish_takeoff(double altitude);
    void settle_relocation(const Relocation& r);  // once the ground at the new place is known
    void integrate(const Command& cmd, const AtmosphereSample& atm, const Vec3& wind, double dt);
    void update_output(const AtmosphereSample& atm, const Vec3& wind);

    Vec2 position_hold_velocity(const Vec2& target, double max_speed) const;
    double lateral_acceleration_max() const;
    void limit_turn(Vec2& v_h) const;
    void limit_sideways(Vec2& v_h) const;

    // Helicopter power (engine_power and rotor_disc_area set).
    bool helicopter() const { return p_.engine_power > 0.0 && p_.rotor_disc_area > 0.0; }
    double power_available(double density) const;
    double power_required(double airspeed, double climb, double density) const;
    double ground_effect(double airspeed, double density) const;  // induced power factor
    void update_power_curve(double density);
    double max_climb_rate(double airspeed) const;  // m/s, negative = cannot hold altitude
    void limit_by_power(Vec2& v_h, double& vz) const;
    // Position error -> velocity gain giving a critically damped stop with the velocity loop.
    double position_gain() const { return 1.0 / (4.0 * p_.velocity_time_constant); }
    double vertical_speed_to(double altitude) const;
    double ground_elevation() const;
    double altitude_target_now() const;

    RotorcraftParams p_;
    FlightState out_{};

    // Dynamic state
    Vec3 position_{};
    Vec3 velocity_{};
    Attitude attitude_{};
    double thrust_ = 0.0;
    bool on_ground_ = false;
    FlightPhase phase_ = FlightPhase::Airborne;

    // Targets
    FlightMode mode_ = FlightMode::Hover;
    double course_target_ = 0.0;
    Vec2 course_anchor_{};
    double speed_target_ = 0.0;
    double altitude_target_ = 0.0;
    AltitudeRef altitude_ref_ = AltitudeRef::Absolute;
    double resolved_altitude_target_ = 0.0;
    Vec2 hold_point_{};
    bool hold_here_ = true;   // Hover from hover(): the held point moves along when relocated
    double yaw_target_ = 0.0;
    double hold_timer_ = 0.0;            // s left at a waypoint with hold_time
    bool waiting_ = false;

    RouteTracker route_;
    Vec3 loiter_center_{};
    double loiter_radius_ = 0.0;
    bool loiter_clockwise_ = true;

    TakeoffPlan takeoff_{};
    double ground_reference_ = 0.0;

    FormationSlot slot_{};
    LeaderState leader_{};
    LeaderTracker leader_tracker_;
    double leader_age_ = 0.0;
    double frame_time_ = 0.0;

    // Climb rate over airspeed at the current altitude, refreshed every step.
    struct PowerCurve {
        static constexpr int kSamples = 32;
        double speed_step = 1.0;                  // m/s between samples
        std::array<double, kSamples + 1> climb{};  // m/s, best steady climb rate per airspeed
        double best_speed = 0.0;                  // m/s, speed of the best climb rate
        double max_speed = 1e9;                   // m/s, fastest level flight
    };
    PowerCurve power_{};

    TerrainMonitor terrain_;
    Vec3 wind_{};                          // last wind, for outputs between steps
    std::optional<Relocation> relocation_; // settled at the start of the next step
};

}  // namespace fm
