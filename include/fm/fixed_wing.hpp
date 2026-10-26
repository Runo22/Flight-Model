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
#include "fm/taxi_path.hpp"
#include "fm/terrain.hpp"
#include "fm/traffic.hpp"
#include "fm/vehicle_params.hpp"

namespace fm {

class FixedWingAgent {
public:
    using params_type = FixedWingParams;
    static constexpr VehicleKind kind = VehicleKind::FixedWing;
    // Positioning / Final: straight-in. Initial, Break, Downwind, Base: overhead pattern.
    enum class LandingStage {
        Positioning, Initial, Break, Downwind, Base, Final, Flare, GoAround, Rollout
    };

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
    // Runway takeoff along plan.runway or plan.heading (default: current heading), taxiing
    // along plan.taxi and lining up first when needed (see TakeoffPlan). Only valid while
    // on the ground; returns false otherwise.
    bool takeoff(const TakeoffPlan& plan);
    // Starts the run of a takeoff planned with `hold` (at once when lined up, otherwise as
    // soon as it is). False when not on a takeoff on the ground.
    bool release_takeoff();
    // Taxi on the ground through `path` (z ignored) and stop at its last point, arriving
    // along `final_heading` (rad) when given, e.g. into a hangar or parking spot after
    // landing. Ends parked. Only valid on the ground; returns false otherwise.
    bool taxi(std::span<const Vec3> path, double final_heading = kUnknown);
    // Approach and landing on `runway` (start = threshold, heading = landing direction):
    // positioning to the extended centerline, 3 deg glide path, flare, touchdown and
    // braking. Unstable approaches go around and try again. Returns false on the ground.
    // LandingOptions select the overhead pattern and where the landing roll ends.
    // Overhead: an aircraft flying along the runway towards the break point (e.g. a
    // wingman next to its leader) starts on its own initial line, others fly to the
    // initial point first.
    bool land(const Runway& runway, const LandingOptions& options = {});
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
    // Form up first, then go: `rejoin_distance` is how far the furthest follower is from
    // its slot (m). Beyond 300 m the leader flies at most 80% of its cruise speed; beyond
    // 2 km it holds (orbits where it is) until the followers have joined, then carries on
    // with its task (cruise, go-to, route). `follower_speed`: what the slowest follower can
    // hold in formation (TAS, m/s, NaN = unknown); the leader keeps below it (75% while one
    // rejoins), but never below its own minimum speed. A follower too slow for that is not
    // waited for: it falls behind.
    void set_formation_lead(double lateral_extent, double rejoin_distance = 0.0,
                            double follower_speed = kUnknown);
    // Holding for followers to join (see set_formation_lead).
    bool waiting_for_formation() const { return lead_hold_; }
    // Fastest true airspeed (m/s) this aircraft can hold in formation at its altitude; NaN
    // when not in formation.
    double formation_speed_limit() const { return formation_speed_limit_; }
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
    LandingStage landing_stage() const { return landing_stage_; }

    // --- Performance helpers --------------------------------------------------------
    double stall_speed(double density, bool flaps) const;
    // Overhead pattern on `runway`: the default initial speed (m/s TAS), and how much
    // further along the runway (m) an aircraft breaks to touch down `interval` seconds
    // after one breaking earlier at `initial_speed`.
    double overhead_initial_speed(const Runway& runway) const;
    double break_spacing(const Runway& runway, double interval, double initial_speed) const;
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
        bool taxi = false;                // ground: taxi_accel with thrust and wheel brakes
        double taxi_accel = 0.0;          // m/s^2
        double taxi_max_brake = 0.0;      // m/s^2, 0 = taxi default
    };
    struct Control {
        double bank = 0.0;
        double gamma = 0.0;
        double throttle = 0.0;
        double steer_heading = 0.0;  // ground steering target
        double gamma_time_constant = 0.0;  // s, 0 = params default
        bool rotate = false;
        bool speed_brake = false;
        bool taxi = false;
        double brake = 0.0;  // m/s^2, wheel brake deceleration while taxiing
    };

    Command guidance(const AtmosphereSample& atm, const Vec3& wind, const SlotReference& slot_ref);
    void guide_route(Command& cmd, const Vec2& ground_velocity, double lookahead);
    void finish_route(EndBehavior behavior);
    void guide_takeoff(Command& cmd, const AtmosphereSample& atm);
    void guide_taxi(Command& cmd);
    void taxi_controls(double accel, double max_brake, const AtmosphereSample& atm,
                       Control& c) const;
    void guide_pattern(Command& cmd, const AtmosphereSample& atm, const Vec2& ground_velocity);
    void update_lead_hold(const AtmosphereSample& atm);
    double break_radius(double speed) const;  // m, overhead break turn
    TaxiPath::Settings taxi_settings() const;
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
    // Speed flown without a command: the cruise schedule, or the endurance speed in orbits.
    double default_speed(const AtmosphereSample& atm) const;
    double cruise_speed_at(const AtmosphereSample& atm) const;  // TAS of the cruise schedule
    double loiter_calibrated_speed() const;
    double approach_speed(double density) const;
    // Slowest touchdown speed within touchdown_aoa (landing flaps); 0 without that limit.
    double touchdown_speed_min(double density) const;
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
    double lead_rejoin_distance_ = 0.0;  // m, furthest follower from its slot
    bool lead_hold_ = false;             // orbiting until the followers have joined
    double hold_joined_time_ = 0.0;      // s the followers have been close while holding
    Vec2 hold_center_{};
    double hold_radius_ = 0.0;
    double lead_follower_speed_ = kUnknown;  // m/s TAS, slowest follower's limit
    double formation_speed_limit_ = kUnknown;  // m/s TAS, as a follower
    bool speed_commanded_ = false;  // false: default_speed()
    double cruise_calibrated_ = 0.0;  // m/s, CAS of the cruise speed at the cruise altitude
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
    double gear_position_ = 1.0;  // 0 up .. 1 down
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
    Vec2 takeoff_origin_{};      // a point on the centerline
    double takeoff_heading_ = 0.0;
    bool lining_up_ = false;     // taxiing to the runway / lining up before the run
    bool takeoff_released_ = true;  // false: lined up and waiting (TakeoffPlan::hold)
    TaxiPath taxi_;              // taxi and line-up path

    Runway runway_{};
    LandingOptions landing_{};
    double initial_offset_ = 0.0;   // m right of the runway line, overhead initial leg
    double downwind_offset_ = 0.0;  // m right of the runway line, downwind leg
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
