// Fixed-wing aircraft agent: point-mass (3-DOF) flight dynamics with derived attitude,
// an energy-based autopilot and navigation modes. Jets, propeller aircraft, airliners and
// fixed-wing UAVs differ only by FixedWingParams.
#pragma once

#include <optional>
#include <span>

#include "fm/atmosphere.hpp"
#include "fm/derived_outputs.hpp"
#include "fm/flight_state.hpp"
#include "fm/formation.hpp"
#include "fm/guidance.hpp"
#include "fm/relocation.hpp"
#include "fm/route.hpp"
#include "fm/route_tracker.hpp"
#include "fm/terrain.hpp"
#include "fm/traffic.hpp"
#include "fm/vehicle_params.hpp"

namespace fm {

class FixedWingAgent {
public:
    using params_type = FixedWingParams;
    static constexpr VehicleKind kind = VehicleKind::FixedWing;

    FixedWingAgent(FixedWingParams params, const InitialConditions& ic);

    // --- Commands -------------------------------------------------------------------
    // Straight and level: keeps the current ground track, altitude target and speed.
    void cruise();
    // Straight and level on a new course (ground track) starting from here.
    void hold(double course, double altitude, double speed,
              AltitudeRef altitude_ref = AltitudeRef::Absolute);
    // Speed target in TAS (m/s), CAS (m/s) or Mach. CAS and Mach targets are held while
    // the altitude changes (the true airspeed follows).
    void set_speed(double speed, SpeedRef ref = SpeedRef::True);
    // Fuel on board (kg, clamped to the capacity): refuelling, or a different load. Empty
    // tanks stop the engines; fuel restarts them.
    void set_fuel(double fuel);
    void go_to(const Vec3& point, const GoToOptions& options = {});
    void follow_route(Route route);
    // radius 0 = derived from turn performance.
    void loiter(const Vec3& center, double radius = 0.0, bool clockwise = true,
                AltitudeRef altitude_ref = AltitudeRef::Absolute);
    // Runway takeoff from the current position along plan.runway (default: current heading).
    // Only valid while on the ground; returns false otherwise.
    bool takeoff(const TakeoffPlan& plan);
    // Approach and landing on `runway` (start = threshold, heading = landing direction):
    // positioning to the extended centerline, 3 deg glide path, flare, touchdown and
    // braking. Unstable approaches go around and try again. Returns false on the ground.
    bool land(const Runway& runway);
    // Without a runway a fixed-wing aircraft cannot land: returns false.
    bool land() { return false; }
    // Fixed-wing aircraft cannot hover: orbits the current position instead.
    void hover();
    // Keep `slot` relative to a leader. Feed the leader with set_leader_state() every
    // step; without updates for a few seconds the aircraft falls back to cruise().
    void join_formation(const FormationSlot& slot);
    // Leading a formation whose widest follower is `lateral_extent` m to the side: banks
    // only as far as the followers can follow (outside: speed and bank limit, inside:
    // minimum speed) and keeps a speed margin. Refreshed every step by the host (the Flecs
    // module does it); expires after 2 s without a refresh. 0 = not leading.
    void set_formation_lead(double lateral_extent);
    // Vehicles around this one (not its formation partners), every step before step(): a
    // predicted conflict is resolved vertically (FixedWingParams::avoidance_*). Without
    // updates for 3 s the resolution ends.
    void set_traffic(std::span<const Traffic> traffic);
    void set_leader_state(const LeaderState& leader);
    // The host moved the aircraft: continue from `position` / `heading` (NaN = keep), see
    // FlightAgent::relocate. Speed, flight path and the current task are kept.
    bool relocate(const Vec3& position, double heading);

    // --- Simulation -----------------------------------------------------------------
    void step(double dt, const Environment& env);

    const FlightState& state() const { return out_; }
    FlightMode mode() const { return mode_; }
    const FixedWingParams& params() const { return p_; }
    // Height of the reference point above the ground when standing on it.
    double ground_offset() const { return p_.gear_height; }
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
        bool terrain_floor = true;        // keep the minimum terrain clearance
        bool climb_mode = false;          // a large climb may slow to the climb speed
        double vertical_speed_ff = 0.0;   // m/s, feedforward added to the altitude loop
        bool idle_thrust = false;         // flare: idle, flight path from vertical speed only
        double gamma_time_constant = 0.0; // s, 0 = params default (flare uses a quicker one)
    };
    struct Control {
        double bank = 0.0;
        double gamma = 0.0;
        double throttle = 0.0;
        double steer_heading = 0.0;  // ground steering target
        double gamma_time_constant = 0.0;  // s, 0 = params default
        bool rotate = false;
        bool speed_brake = false;
    };

    Command guidance(const AtmosphereSample& atm, const Vec3& wind, const SlotReference& slot_ref);
    void guide_route(Command& cmd, const Vec2& ground_velocity, double lookahead);
    void finish_route(EndBehavior behavior);
    void guide_takeoff(Command& cmd, const AtmosphereSample& atm);
    void guide_formation(Command& cmd, const Vec2& ground_velocity, const SlotReference& ref);
    void guide_landing(Command& cmd, const AtmosphereSample& atm, const Vec2& ground_velocity);
    Control autopilot(const Command& cmd, const AtmosphereSample& atm);
    void integrate(const Control& c, const AtmosphereSample& atm, const Vec3& wind, double dt);
    void integrate_ground(const Control& c, const AtmosphereSample& atm, double dt);
    void update_output(const AtmosphereSample& atm, const Vec3& wind);
    void finish_takeoff(double altitude);
    void trim_level(const AtmosphereSample& atm);
    void settle_relocation(const Relocation& r);  // once the ground at the new place is known
    void start_flying_here();                     // a grounded aircraft placed in the air

    void hold_course(double course, const Vec2& anchor, double altitude, double speed,
                     AltitudeRef altitude_ref);
    L1Settings l1_settings(double ground_speed) const;
    double ground_elevation() const;
    double altitude_target_now(double lookahead) const;
    double engine_lapse(const AtmosphereSample& atm) const;  // thrust / power share left
    double thrust_available(double tas, const AtmosphereSample& atm) const;
    double drag(double tas, double load_factor, const AtmosphereSample& atm, bool brake) const;
    double cl0() const;
    double cl_max() const;
    double cd0() const;
    double loiter_radius_default() const;
    double approach_speed(double density) const;
    double min_drag_speed(double density) const;  // best climb angle, roughly
    double climb_speed(const AtmosphereSample& atm, double v_min, double v_max);
    double climb_rate_at(double tas, const AtmosphereSample& atm) const;  // m/s, full power
    // Fastest steady level flight within the operating limits (cached per altitude band).
    double sustained_speed(const AtmosphereSample& atm, double v_min, double v_max);
    double lead_bank_limit(const AtmosphereSample& atm, double v_min, double v_max);
    bool fuel_modelled() const { return p_.fuel_capacity > 0.0; }
    double fuel_flow(double tas, const AtmosphereSample& atm) const;  // kg/s at throttle_
    void burn_fuel(double tas, const AtmosphereSample& atm, double dt);
    Vec3 aim_point() const;  // glide path touchdown aim point on the runway

    FixedWingParams p_;
    double rated_lapse_ = 1.0;  // engine density lapse at the critical altitude
    MotionTracker motion_;
    // Collision avoidance: +1 climbing, -1 descending away from traffic, 0 none.
    int avoid_sense_ = 0;
    double avoid_altitude_ = 0.0;   // m, altitude that clears the conflict
    bool conflict_now_ = false;     // the last traffic update still predicts a conflict
    double avoid_clear_time_ = 0.0; // s without a conflict
    double lead_extent_ = 0.0;  // m, widest follower offset (formation lead)
    double lead_age_ = 0.0;     // s since the last refresh
    double sustained_speed_ = 0.0;
    double sustained_altitude_ = -1e9;
    double fuel_ = 0.0;       // kg
    double mass_ = 0.0;       // kg, with the fuel left
    double fuel_flow_ = 0.0;  // kg/s
    SpeedRef speed_ref_ = SpeedRef::True;  // speed_target_ follows speed_command_ in this
    double speed_command_ = 0.0;
    double temperature_offset_ = 0.0;     // K, last ISA deviation
    bool climbing_ = false;     // climb mode: full power at the climb speed
    double best_climb_speed_ = 0.0;              // m/s TAS, cached ...
    double best_climb_altitude_ = -1e9;          // ... for this altitude
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
    bool gear_down_ = true;
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
    bool loiter_here_ = false;  // orbit from hover(): moves along when relocated

    TakeoffPlan takeoff_{};
    Vec2 takeoff_origin_{};
    double takeoff_heading_ = 0.0;

    enum class LandingStage { Positioning, Final, Flare, GoAround, Rollout };
    Runway runway_{};
    LandingStage landing_stage_ = LandingStage::Positioning;
    double flare_height_ = 0.0;  // m, wheel height where the flare started (lag included)
    double flare_sink_ = 0.0;    // m/s, sink rate at the flare start
    double runway_elevation_ = 0.0;

    FormationSlot slot_{};
    LeaderState leader_{};
    LeaderTracker leader_tracker_;
    double leader_age_ = 0.0;            // s since the last leader update
    double frame_time_ = 0.0;            // s since the start of the current step

    TerrainMonitor terrain_;
    Vec3 wind_{};                          // last wind, for outputs between steps
    std::optional<Relocation> relocation_; // settled at the start of the next step
};

}  // namespace fm
