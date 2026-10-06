// Fixed-wing aircraft agent: point-mass (3-DOF) flight dynamics with derived attitude,
// an energy-based autopilot and navigation modes. Jets, propeller aircraft, airliners and
// fixed-wing UAVs differ only by FixedWingParams.
#pragma once

#include <optional>

#include "fm/atmosphere.hpp"
#include "fm/flight_state.hpp"
#include "fm/route.hpp"
#include "fm/route_tracker.hpp"
#include "fm/terrain.hpp"
#include "fm/vehicle_params.hpp"

namespace fm {

class FixedWingAgent {
public:
    FixedWingAgent(FixedWingParams params, const InitialConditions& ic);

    // --- Commands -------------------------------------------------------------------
    // Straight and level on the current heading, altitude and speed target.
    void cruise();
    void hold(double heading, double altitude, double speed,
              AltitudeRef altitude_ref = AltitudeRef::Absolute);
    void set_speed(double speed);
    void go_to(const Vec3& point, const GoToOptions& options = {});
    void follow_route(Route route);
    // radius 0 = derived from turn performance.
    void loiter(const Vec3& center, double radius = 0.0, bool clockwise = true,
                AltitudeRef altitude_ref = AltitudeRef::Absolute);
    // Runway takeoff. Only valid while on the ground; returns false otherwise.
    bool takeoff(const TakeoffPlan& plan);

    // --- Simulation -----------------------------------------------------------------
    void step(double dt, const Environment& env);

    const FlightState& state() const { return out_; }
    FlightMode mode() const { return mode_; }
    const FixedWingParams& params() const { return p_; }
    const RouteTracker& route() const { return route_; }

    // --- Performance helpers --------------------------------------------------------
    double stall_speed(double density, bool flaps) const;
    double turn_radius(double speed) const;
    double rotate_speed(double density) const;

private:
    struct Command {
        bool use_accel = false;
        double heading = 0.0;
        double lateral_accel = 0.0;
        double altitude = 0.0;
        double speed = 0.0;
        double bank_limit = 0.0;
        bool full_thrust = false;
        bool rotate = false;
        bool terrain_alert = false;
    };
    struct Control {
        double bank = 0.0;
        double gamma = 0.0;
        double throttle = 0.0;
        bool rotate = false;
        bool speed_brake = false;
    };
    enum class TakeoffStage { Roll, Climb };

    Command guidance(const AtmosphereSample& atm, const Vec3& wind);
    void guide_route(Command& cmd, const Vec2& ground_velocity, double lookahead);
    void finish_route(const EndBehavior behavior);
    void guide_takeoff(Command& cmd, const AtmosphereSample& atm);
    Control autopilot(const Command& cmd, const AtmosphereSample& atm);
    void integrate(const Control& c, const AtmosphereSample& atm, const Vec3& wind, double dt);
    void integrate_ground(const Control& c, const AtmosphereSample& atm, double dt);
    void update_output(const AtmosphereSample& atm, const Vec3& wind);

    double ground_elevation() const;
    double altitude_target_now(double lookahead) const;
    double thrust_available(double tas, const AtmosphereSample& atm) const;
    double drag(double tas, double load_factor, const AtmosphereSample& atm, bool brake) const;
    double cl0() const;
    double cl_max() const;
    double cd0() const;
    double loiter_radius_default() const;

    FixedWingParams p_;
    FlightState out_{};

    // Dynamic state
    Vec3 position_{};
    double tas_ = 0.0;
    double heading_ = 0.0;   // air-relative heading
    double gamma_ = 0.0;     // air-relative flight path angle
    double bank_ = 0.0;
    double throttle_ = 0.0;
    double ground_pitch_ = 0.0;
    double alpha_ = 0.0;
    double load_factor_ = 1.0;
    bool on_ground_ = false;
    bool flaps_ = false;
    bool speed_brake_ = false;
    FlightPhase phase_ = FlightPhase::Airborne;

    // Targets
    FlightMode mode_ = FlightMode::Cruise;
    double heading_target_ = 0.0;
    double altitude_target_ = 0.0;      // absolute, or AGL when altitude_ref_ is AboveTerrain
    AltitudeRef altitude_ref_ = AltitudeRef::Absolute;
    double resolved_altitude_target_ = 0.0;
    double speed_target_ = 0.0;
    double speed_integrator_ = 0.0;

    RouteTracker route_;
    Vec3 loiter_center_{};
    double loiter_radius_ = 0.0;
    bool loiter_clockwise_ = true;

    TakeoffPlan takeoff_{};
    TakeoffStage takeoff_stage_ = TakeoffStage::Roll;
    double runway_elevation_ = 0.0;

    TerrainMonitor terrain_;
};

}  // namespace fm
