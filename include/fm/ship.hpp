// Large surface ship (carrier, helicopter / UAV carrier, escort): propulsion against hull
// resistance, first-order (Nomoto) steering with a turning circle, drift angle, speed loss
// and outward heel in turns, station keeping. It stays on the water: position.z is always
// the water level and only x/y of commanded positions are used.
#pragma once

#include "fm/flight_state.hpp"
#include "fm/formation.hpp"
#include "fm/guidance.hpp"
#include "fm/relocation.hpp"
#include "fm/route.hpp"
#include "fm/route_tracker.hpp"
#include "fm/terrain.hpp"
#include "fm/vehicle_params.hpp"

namespace fm {

class ShipAgent {
public:
    using params_type = ShipParams;
    static constexpr VehicleKind kind = VehicleKind::Ship;

    // ic.speed: initial speed through the water (0 = stopped); ic.position.z is ignored.
    ShipAgent(ShipParams params, const InitialConditions& ic);

    // --- Commands -------------------------------------------------------------------
    // Straight ahead on the current track at the current speed.
    void cruise();
    // Course and speed (the altitude is ignored); speed 0 stops the ship.
    void hold(double course, double altitude, double speed,
              AltitudeRef altitude_ref = AltitudeRef::Absolute);
    // Speed through the water: 0 stops, negative goes astern, NaN keeps.
    void set_speed(double speed);
    // on_arrival: ContinueStraight keeps going, Loiter and Land stop and keep station there.
    void go_to(const Vec3& point, const GoToOptions& options = {});
    // Legs with fly-by turns. on_end: Loiter circles the last waypoint with
    // route.loiter_radius (0 = stop there), Land stops there, ContinueStraight keeps going.
    void follow_route(Route route);
    // Circle `center` (radius at least the turning radius); radius 0 = keep station there.
    void loiter(const Vec3& center, double radius = 0.0, bool clockwise = true,
                AltitudeRef altitude_ref = AltitudeRef::Absolute);
    bool takeoff(const TakeoffPlan&) { return false; }  // ships do not take off
    // Stop and keep station here.
    void hover();
    bool land() { return false; }
    bool land(const Runway&) { return false; }
    // Ship specific (agent.ship()->steer(...)): turn at `turn_rate` (rad/s, positive =
    // starboard; +-infinity = full rudder) until another command is given.
    void steer(double turn_rate);
    // Escort / convoy station relative to a leader.
    void join_formation(const FormationSlot& slot);
    void set_leader_state(const LeaderState& leader);
    // The host moved the ship (the altitude is ignored); see FlightAgent::relocate.
    bool relocate(const Vec3& position, double heading);

    // --- Simulation -----------------------------------------------------------------
    // Terrain above the water level counts as land: a ship sailing onto it runs aground and
    // stops (phase TerrainContact) until relocated.
    void step(double dt, const Environment& env);

    const FlightState& state() const { return out_; }
    FlightMode mode() const { return mode_; }
    const ShipParams& params() const { return p_; }
    double ground_offset() const { return 0.0; }  // reference point on the waterline
    const RouteTracker& route() const { return route_; }

    double turning_radius() const { return 0.5 * p_.turning_diameter; }
    // Highest turn rate at `speed` (rad/s): the turning circle, or pivoting at rest.
    double max_turn_rate(double speed) const;

private:
    struct Command {
        double speed = 0.0;      // m/s through the water, negative = astern
        double turn_rate = 0.0;  // rad/s, positive = starboard
    };

    Command guidance(const SlotReference& slot_ref);
    void guide_route(Command& cmd);
    void finish_route(EndBehavior behavior);
    void guide_circle(Command& cmd) const;
    void guide_formation(Command& cmd, const SlotReference& ref) const;
    void keep_station(Command& cmd, const Vec2& point) const;
    void integrate(const Command& cmd, double dt);
    void update_output();
    void ensure_moving();  // a stopped ship given somewhere to go uses its cruise speed

    // Heading autopilot: turn rate that brings the track to `course` (rad), on top of a
    // feedforward turn rate (circles, the leader's turns).
    double steer_to(double course, double feedforward) const;
    // Line-of-sight course that joins the line through `point` along `course`.
    double line_course(const Vec2& point, double course) const;
    double lookahead() const;  // m, line-of-sight distance
    double yaw_lag() const;    // s, steering (Nomoto) time constant at the current speed
    double drift_angle(double turn_rate) const;  // rad, bow inside the turn

    double thrust_available(double speed) const;  // N, ahead
    double resistance(double speed) const;        // N, signed, straight ahead
    double stopping_speed(double distance) const;  // m/s that still stops within distance
    double arrival_radius() const;

    ShipParams p_;
    FlightState out_{};

    // Dynamic state
    Vec3 position_{};
    double heading_ = 0.0;
    double speed_ = 0.0;      // m/s through the water along the track, negative = astern
    double turn_rate_ = 0.0;  // rad/s
    double drift_ = 0.0;      // rad, heading minus track
    double heel_ = 0.0;       // rad
    double thrust_ = 0.0;     // N, negative = astern
    double time_ = 0.0;       // s, for the swell
    bool aground_ = false;
    FlightPhase phase_ = FlightPhase::Parked;

    // Targets
    FlightMode mode_ = FlightMode::Cruise;
    double steer_rate_ = 0.0;
    bool steering_ = false;  // steer(): fixed turn rate instead of a course
    double course_target_ = 0.0;
    Vec2 course_anchor_{};
    double speed_target_ = 0.0;
    RouteTracker route_;
    Vec3 loiter_center_{};
    double loiter_radius_ = 0.0;
    bool loiter_clockwise_ = true;
    Vec2 hold_point_{};
    bool hold_here_ = true;  // station from hover(): moves along when relocated

    FormationSlot slot_{};
    LeaderState leader_{};
    LeaderTracker leader_tracker_;
    double leader_age_ = 0.0;

    TerrainMonitor terrain_;
};

}  // namespace fm
