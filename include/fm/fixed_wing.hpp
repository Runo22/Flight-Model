// Fixed-wing aircraft agent: point-mass (3-DOF) flight dynamics with derived attitude,
// an energy-based autopilot and navigation modes. Jets, propeller aircraft, airliners and
// fixed-wing UAVs differ only by FixedWingParams.
#pragma once

#include "fm/atmosphere.hpp"
#include "fm/flight_state.hpp"
#include "fm/formation.hpp"
#include "fm/guidance.hpp"
#include "fm/route.hpp"
#include "fm/route_tracker.hpp"
#include "fm/terrain.hpp"
#include "fm/vehicle_params.hpp"

namespace fm {

class FixedWingAgent {
public:
    FixedWingAgent(FixedWingParams params, const InitialConditions& ic);

    // --- Commands -------------------------------------------------------------------
    // Straight and level: keeps the current ground track, altitude target and speed.
    void cruise();
    // Straight and level on a new course (ground track) starting from here.
    void hold(double course, double altitude, double speed,
              AltitudeRef altitude_ref = AltitudeRef::Absolute);
    void set_speed(double speed);
    void go_to(const Vec3& point, const GoToOptions& options = {});
    void follow_route(Route route);
    // radius 0 = derived from turn performance.
    void loiter(const Vec3& center, double radius = 0.0, bool clockwise = true,
                AltitudeRef altitude_ref = AltitudeRef::Absolute);
    // Runway takeoff from the current position along plan.runway (default: current heading).
    // Only valid while on the ground; returns false otherwise.
    bool takeoff(const TakeoffPlan& plan);
    // Keep `slot` relative to a leader. Feed the leader with set_leader_state() every
    // step; without updates for a few seconds the aircraft falls back to cruise().
    void join_formation(const FormationSlot& slot);
    void set_leader_state(const LeaderState& leader);

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
        double heading = 0.0;          // used when !use_accel (and for ground steering)
        double lateral_accel = 0.0;    // m/s^2, positive = right
        double altitude = 0.0;
        double speed = 0.0;
        double speed_time_constant = 0.0;  // 0 = params default
        double bank_limit = 0.0;
        bool full_thrust = false;
        bool rotate = false;
        bool terrain_alert = false;
    };
    struct Control {
        double bank = 0.0;
        double gamma = 0.0;
        double throttle = 0.0;
        double steer_heading = 0.0;  // ground steering target
        bool rotate = false;
        bool speed_brake = false;
    };

    Command guidance(const AtmosphereSample& atm, const Vec3& wind, const SlotReference& slot_ref);
    void guide_route(Command& cmd, const Vec2& ground_velocity, double lookahead);
    void finish_route(EndBehavior behavior);
    void guide_takeoff(Command& cmd, const AtmosphereSample& atm);
    void guide_formation(Command& cmd, const Vec2& ground_velocity, const SlotReference& ref);
    Control autopilot(const Command& cmd, const AtmosphereSample& atm);
    void integrate(const Control& c, const AtmosphereSample& atm, const Vec3& wind, double dt);
    void integrate_ground(const Control& c, const AtmosphereSample& atm, double dt);
    void update_output(const AtmosphereSample& atm, const Vec3& wind);

    void hold_course(double course, const Vec2& anchor, double altitude, double speed,
                     AltitudeRef altitude_ref);
    L1Settings l1_settings(double ground_speed) const;
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
    double course_target_ = 0.0;
    Vec2 course_anchor_{};               // a point on the held ground track
    double altitude_target_ = 0.0;       // absolute, or AGL when altitude_ref_ is AboveTerrain
    AltitudeRef altitude_ref_ = AltitudeRef::Absolute;
    double resolved_altitude_target_ = 0.0;
    double speed_target_ = 0.0;

    RouteTracker route_;
    Vec3 loiter_center_{};
    double loiter_radius_ = 0.0;
    bool loiter_clockwise_ = true;

    TakeoffPlan takeoff_{};
    Vec2 takeoff_origin_{};
    double takeoff_heading_ = 0.0;
    double runway_elevation_ = 0.0;

    FormationSlot slot_{};
    LeaderState leader_{};
    LeaderTracker leader_tracker_;
    double leader_age_ = 0.0;            // s since the last leader update
    double frame_time_ = 0.0;            // s since the start of the current step

    TerrainMonitor terrain_;
};

}  // namespace fm
