// Performance measurements: fly a preset through the real agent and measure what a flight
// manual lists (speeds, climb, ceilings, takeoff and landing, turns). Used by the preset
// tests and by fm_performance_report.
#pragma once

#include <algorithm>
#include <cmath>
#include <functional>

#include "fm/fixed_wing.hpp"
#include "fm/rotorcraft.hpp"
#include "fm/ship.hpp"

namespace fmperf {

using namespace fm;

constexpr double kDt = 1.0 / 60.0;

inline double kt(double ms) { return ms / 0.514444; }  // m/s -> knots
inline double fpm(double ms) { return ms / 0.00508; }  // m/s -> ft/min
inline double ft(double m) { return m / 0.3048; }      // m -> ft

inline InitialConditions airborne(double altitude, double speed, double heading = 0.0) {
    InitialConditions ic;
    ic.position = {0.0, 0.0, altitude};
    ic.heading = heading;
    ic.speed = speed;
    return ic;
}

inline InitialConditions parked() {
    InitialConditions ic;
    ic.on_ground = true;
    return ic;
}

// Steps until `done` returns true or `max_seconds` passed. `terrain` NaN = no ground.
template <typename Agent>
double fly(Agent& a, double max_seconds, double terrain,
           const std::function<bool(const FlightState&, double)>& done = {}) {
    Environment env;
    env.terrain_elevation = terrain;
    double t = 0.0;
    while (t < max_seconds) {
        a.step(kDt, env);
        t += kDt;
        if (done && done(a.state(), t)) break;
    }
    return t;
}

// Bisection for the highest x in [lo, hi] where `ok(x)` holds (ok(lo) assumed true).
inline double highest(double lo, double hi, double tolerance, const std::function<bool(double)>& ok) {
    if (ok(hi)) return hi;
    while (hi - lo > tolerance) {
        const double mid = 0.5 * (lo + hi);
        (ok(mid) ? lo : hi) = mid;
    }
    return lo;
}

// --- Fixed-wing --------------------------------------------------------------------------

// The aircraft without the AI pilot's operating limits: what the airframe and engines can do.
inline FixedWingParams physical(FixedWingParams p, bool keep_speed_limits = false) {
    p.climb_rate_max = 1000.0;
    p.climb_angle_max = deg2rad(80.0);
    p.altitude_time_constant = 1.0;
    if (!keep_speed_limits) {
        p.max_speed = 1e4;
        p.mach_max = 10.0;
    }
    return p;
}

// Stall speed (1 g, true airspeed) at an altitude.
inline double stall_speed(const FixedWingParams& p, double altitude, bool flaps) {
    return FixedWingAgent(p, airborne(altitude, p.cruise_speed)).stall_speed(isa(altitude).density, flaps);
}

// Steady level-flight speed (TAS) at full power. keep_limits: with the preset's Vmo/Mmo.
inline double max_level_speed(const FixedWingParams& p, double altitude, bool keep_limits) {
    FixedWingAgent a(physical(p, keep_limits), airborne(altitude, p.cruise_speed));
    a.hold(0.0, altitude, 1e4);
    double last = 0.0;
    fly(a, 1500.0, kUnknown, [&](const FlightState& s, double t) {
        if (std::fmod(t, 20.0) < kDt) {
            const bool settled = std::abs(s.airspeed - last) < 0.1;
            last = s.airspeed;
            return settled && t > 40.0;
        }
        return false;
    });
    return a.state().airspeed;
}

// Energy height: altitude plus the altitude the speed is worth.
inline double energy_height(const FlightState& s) {
    return s.position.z + s.airspeed * s.airspeed / (2.0 * kGravity);
}

// Climb rate at full power at `altitude` and true airspeed `speed`, as specific excess
// power (climb rate plus speed gain in energy terms): the steady climb rate at that speed,
// even while the autopilot still splits the energy between climb and speed.
inline double climb_rate(const FixedWingParams& p, double altitude, double speed) {
    FixedWingAgent a(physical(p), airborne(altitude, speed));
    a.hold(0.0, altitude + 30000.0, speed);
    const double settle = 3.0 * p.gamma_time_constant + 2.0;
    fly(a, settle, kUnknown);
    const double e0 = energy_height(a.state());
    fly(a, 3.0, kUnknown);
    return (energy_height(a.state()) - e0) / 3.0;
}

struct Climb {
    double rate = 0.0;   // m/s
    double speed = 0.0;  // m/s TAS
};

// Best climb rate over the speed range at `altitude`.
inline Climb best_climb(const FixedWingParams& p, double altitude) {
    const double lo = 1.25 * stall_speed(p, altitude, false);
    const double a_sound = isa(altitude).speed_of_sound;
    const double hi = std::max(lo + 1.0, std::min(p.max_speed / std::sqrt(isa(altitude).sigma),
                                                  0.95 * std::min(p.mach_max, 1.5) * a_sound));
    Climb best{-1e9, lo};
    const int n = 12;
    for (int i = 0; i <= n; ++i) {
        const double v = lo + (hi - lo) * i / n;
        const double r = climb_rate(p, altitude, v);
        if (r > best.rate) best = {r, v};
    }
    // Refine around the best speed.
    const double step = (hi - lo) / n;
    for (double v = best.speed - step; v <= best.speed + step; v += step / 4.0) {
        if (v < lo || v > hi) continue;
        const double r = climb_rate(p, altitude, v);
        if (r > best.rate) best = {r, v};
    }
    return best;
}

// Service ceiling: highest altitude with 100 ft/min climb.
inline double service_ceiling(const FixedWingParams& p) {
    return highest(0.0, 30000.0, 50.0, [&](double h) { return best_climb(p, h).rate >= 0.508; });
}

struct Takeoff {
    double ground_roll = 0.0;    // m
    double liftoff_speed = 0.0;  // m/s TAS
    double to_50ft = 0.0;        // m, ground distance until the wheels are 15 m high
};

inline Takeoff takeoff(const FixedWingParams& p) {
    FixedWingAgent a(p, parked());
    a.takeoff({});
    Takeoff r;
    bool airborne_now = false;
    fly(a, 600.0, 0.0, [&](const FlightState& s, double) {
        if (!airborne_now && !s.on_ground) {
            airborne_now = true;
            r.ground_roll = s.position.y;
            r.liftoff_speed = s.airspeed;
        }
        if (s.position.z - p.gear_height >= 15.24) {
            r.to_50ft = s.position.y;
            return true;
        }
        return false;
    });
    return r;
}

struct Landing {
    double approach_speed = 0.0;   // m/s TAS, 2 km before the threshold
    double touchdown_speed = 0.0;  // m/s
    double sink_rate = 0.0;        // m/s at touchdown
    double ground_roll = 0.0;      // m, touchdown to stop
    double from_50ft = 0.0;        // m, from 15 m wheel height to stop
    bool landed = false;
};

inline Landing landing(const FixedWingParams& p) {
    const double distance = p.final_approach_distance + 6000.0;
    // Start south of the runway, landing northwards.
    FixedWingAgent b(p, [&] {
        InitialConditions ic = airborne(distance * std::tan(p.glide_slope) + 300.0,
                                        1.6 * stall_speed(p, 0.0, false));
        ic.position.y = -distance;
        return ic;
    }());
    b.land(Runway{{0.0, 0.0, 0.0}, 0.0});
    Landing r;
    double last_vs = 0.0;
    double y50 = 0.0;
    bool was_airborne = true;
    fly(b, 1500.0, 0.0, [&](const FlightState& s, double) {
        if (r.approach_speed == 0.0 && s.position.y > -2000.0 && s.position.x * s.position.x < 1e6)
            r.approach_speed = s.airspeed;
        if (y50 == 0.0 && s.position.z - p.gear_height < 15.24 && s.position.y > -3000.0)
            y50 = s.position.y;
        if (was_airborne && s.on_ground) {
            r.touchdown_speed = s.airspeed;
            r.sink_rate = -last_vs;
            r.ground_roll = -s.position.y;  // negative until stopped
        }
        was_airborne = !s.on_ground;
        last_vs = s.vertical_speed;
        if (s.phase == FlightPhase::Landed) {
            r.landed = true;
            r.ground_roll += s.position.y;
            r.from_50ft = s.position.y - y50;
            return true;
        }
        return false;
    });
    return r;
}

struct Turn {
    double rate = 0.0;          // rad/s, steady turn rate
    double bank = 0.0;          // rad
    double altitude_loss = 0.0; // m
    double speed_loss = 0.0;    // m/s
};

// A 180 degree course reversal at `altitude` and `speed` with the AI pilot's limits.
inline Turn course_reversal(const FixedWingParams& p, double altitude, double speed) {
    FixedWingAgent a(p, airborne(altitude, speed));
    a.hold(deg2rad(179.0), altitude, speed);
    Turn r;
    double last_heading = 0.0;
    fly(a, 120.0, kUnknown, [&](const FlightState& s, double t) {
        if (t > 0.5) r.rate = std::max(r.rate, std::abs(wrap_pi(s.attitude.heading - last_heading)) / kDt);
        last_heading = s.attitude.heading;
        r.bank = std::max(r.bank, std::abs(s.attitude.roll));
        r.altitude_loss = std::max(r.altitude_loss, altitude - s.position.z);
        r.speed_loss = std::max(r.speed_loss, speed - s.airspeed);
        return std::abs(wrap_pi(s.track - deg2rad(179.0))) < deg2rad(2.0);
    });
    return r;
}

// --- Rotorcraft ---------------------------------------------------------------------------

// Without the AI pilot's climb limit.
inline RotorcraftParams physical(RotorcraftParams p) {
    p.climb_rate_max = 1000.0;
    return p;
}

// Steady level-flight speed at full power (up to the preset's speed limit).
inline double max_level_speed(const RotorcraftParams& p, double altitude) {
    RotorcraftAgent a(p, airborne(altitude, 20.0));
    a.hold(0.0, altitude, 1000.0);
    double last = 0.0;
    fly(a, 600.0, kUnknown, [&](const FlightState& s, double t) {
        if (std::fmod(t, 10.0) < kDt) {
            const bool settled = std::abs(s.ground_speed - last) < 0.05;
            last = s.ground_speed;
            return settled && t > 30.0;
        }
        return false;
    });
    return a.state().ground_speed;
}

struct RotorClimb {
    double rate = 0.0;   // m/s
    double speed = 0.0;  // m/s, airspeed while climbing
};

// Best climb: asked for an unlimited climb, the agent slows to its best climb speed.
inline RotorClimb best_climb(const RotorcraftParams& p, double altitude) {
    RotorcraftAgent a(physical(p), airborne(altitude, p.cruise_speed));
    a.hold(0.0, altitude + 5000.0, p.max_speed);
    fly(a, 40.0, kUnknown);
    const double z0 = a.state().position.z;
    fly(a, 5.0, kUnknown);
    return {(a.state().position.z - z0) / 5.0, a.state().ground_speed};
}

// Vertical climb rate out of ground effect (negative: cannot hover there).
inline double vertical_climb(const RotorcraftParams& p, double altitude) {
    RotorcraftAgent a(physical(p), airborne(altitude, 0.0));
    a.loiter({0.0, 0.0, altitude + 5000.0}, 0.0);
    fly(a, 15.0, kUnknown);
    const double z0 = a.state().position.z;
    fly(a, 5.0, kUnknown);
    return (a.state().position.z - z0) / 5.0;
}

inline double hover_ceiling(const RotorcraftParams& p) {
    return highest(0.0, 9000.0, 25.0, [&](double h) { return vertical_climb(p, h) >= 0.1; });
}

inline double service_ceiling(const RotorcraftParams& p) {
    return highest(0.0, 9000.0, 25.0, [&](double h) { return best_climb(p, h).rate >= 0.508; });
}

struct Acceleration {
    double time = 0.0;        // s, from hover to 95% of the target speed
    double pitch_down = 0.0;  // rad, largest nose-down pitch
};

inline Acceleration accelerate(const RotorcraftParams& p, double speed) {
    RotorcraftAgent a(p, airborne(300.0, 0.0));
    a.hold(0.0, 300.0, speed);
    Acceleration r;
    r.time = fly(a, 300.0, kUnknown, [&](const FlightState& s, double) {
        r.pitch_down = std::max(r.pitch_down, -s.attitude.pitch);
        return s.ground_speed >= 0.95 * speed;
    });
    return r;
}

struct Stop {
    double time = 0.0;      // s, from `speed` to hover over a point
    double distance = 0.0;  // m, from the start of the deceleration
    double pitch_up = 0.0;  // rad, largest nose-up pitch
};

// Flies at `speed` towards a point and stops over it; measured from the start of braking.
inline Stop stop_at_point(const RotorcraftParams& p, double speed) {
    RotorcraftAgent a(p, airborne(300.0, speed));
    a.hold(0.0, 300.0, speed);
    fly(a, 20.0, kUnknown);
    const double target_y = a.state().position.y + 6000.0;
    GoToOptions opt;
    opt.on_arrival = EndBehavior::Loiter;
    opt.speed = speed;
    a.go_to({0.0, target_y, 300.0}, opt);
    Stop r;
    double brake_y = -1.0;
    double brake_t = 0.0;
    const double t = fly(a, 600.0, kUnknown, [&](const FlightState& s, double now) {
        if (brake_y < 0.0 && s.ground_speed < 0.97 * speed) {
            brake_y = s.position.y;
            brake_t = now;
        }
        r.pitch_up = std::max(r.pitch_up, s.attitude.pitch);
        return s.ground_speed < 0.3 && std::abs(s.position.y - target_y) < 2.0;
    });
    r.time = t - brake_t;
    r.distance = target_y - brake_y;
    return r;
}

struct Cruise {
    double pitch = 0.0;  // rad, at steady speed (negative = nose down)
};

inline Cruise cruise_attitude(const RotorcraftParams& p, double speed) {
    RotorcraftAgent a(p, airborne(300.0, speed));
    a.hold(0.0, 300.0, speed);
    fly(a, 60.0, kUnknown);
    return {a.state().attitude.pitch};
}

struct RotorTurn {
    double rate = 0.0;       // rad/s
    double bank = 0.0;       // rad
    double min_speed = 0.0;  // m/s during the turn
    double radius = 0.0;     // m, speed / rate
};

// A 180 degree course reversal at `speed`.
inline RotorTurn course_reversal(const RotorcraftParams& p, double speed) {
    RotorcraftAgent a(p, airborne(300.0, speed));
    a.hold(0.0, 300.0, speed);
    fly(a, 30.0, kUnknown);
    a.hold(deg2rad(180.0), 300.0, speed);
    RotorTurn r;
    r.min_speed = speed;
    double last_track = a.state().track;
    fly(a, 120.0, kUnknown, [&](const FlightState& s, double) {
        r.rate = std::max(r.rate, std::abs(wrap_pi(s.track - last_track)) / kDt);
        last_track = s.track;
        r.bank = std::max(r.bank, std::abs(s.attitude.roll));
        r.min_speed = std::min(r.min_speed, s.ground_speed);
        return std::abs(wrap_pi(s.track - deg2rad(180.0))) < deg2rad(2.0);
    });
    r.radius = r.rate > 0.0 ? speed / r.rate : 0.0;
    return r;
}

// --- Ships ---------------------------------------------------------------------------------

struct ShipAcceleration {
    double max_speed = 0.0;     // m/s, steady at full power
    double to_cruise = 0.0;     // s, from rest to the cruise speed
    double to_90_percent = 0.0; // s, from rest to 90% of the maximum speed
};

inline ShipAcceleration ship_acceleration(const ShipParams& p) {
    ShipAgent a(p, airborne(0.0, 0.0));
    a.hold(0.0, 0.0, p.max_speed);
    ShipAcceleration r;
    fly(a, 1800.0, kUnknown, [&](const FlightState& s, double t) {
        if (r.to_cruise == 0.0 && s.airspeed >= p.cruise_speed) r.to_cruise = t;
        if (r.to_90_percent == 0.0 && s.airspeed >= 0.9 * p.max_speed) r.to_90_percent = t;
        return false;
    });
    r.max_speed = a.state().airspeed;
    return r;
}

struct ShipStop {
    double time = 0.0;      // s
    double distance = 0.0;  // m, head reach
};

// Crash stop: full astern from full speed until the ship is stopped in the water.
inline ShipStop ship_crash_stop(const ShipParams& p) {
    ShipAgent a(p, airborne(0.0, p.max_speed));
    a.hold(0.0, 0.0, p.max_speed);
    fly(a, 60.0, kUnknown);
    const double y0 = a.state().position.y;
    a.set_speed(0.0);
    ShipStop r;
    r.time = fly(a, 1800.0, kUnknown, [](const FlightState& s, double) { return s.airspeed < 0.1; });
    r.distance = a.state().position.y - y0;
    return r;
}

struct TurningCircle {
    double advance = 0.0;          // m, along the original course at 90 deg of heading change
    double tactical_diameter = 0.0;  // m, across the original course at 180 deg
    double steady_diameter = 0.0;  // m
    double speed = 0.0;            // m/s in the steady turn
    double heel = 0.0;             // rad, negative = to port (outward in a starboard turn)
    double drift = 0.0;            // rad, heading minus track
};

// Full starboard rudder from full speed.
inline TurningCircle turning_circle(const ShipParams& p) {
    ShipAgent a(p, airborne(0.0, p.max_speed));
    a.hold(0.0, 0.0, p.max_speed);
    fly(a, 30.0, kUnknown);
    const Vec3 start = a.state().position;
    a.steer(1e9);  // full rudder
    TurningCircle r;
    double turned = 0.0;
    double last = a.state().attitude.heading;
    double x_min = 1e18;
    double x_max = -1e18;
    fly(a, 3600.0, kUnknown, [&](const FlightState& s, double) {
        turned += wrap_pi(s.attitude.heading - last);
        last = s.attitude.heading;
        const Vec3 d = s.position - start;
        if (r.advance == 0.0 && turned >= deg2rad(90.0)) r.advance = d.y;
        if (r.tactical_diameter == 0.0 && turned >= deg2rad(180.0)) r.tactical_diameter = d.x;
        if (turned > deg2rad(400.0)) {  // steady turn
            x_min = std::min(x_min, d.x);
            x_max = std::max(x_max, d.x);
            r.speed = s.airspeed;
            r.heel = s.attitude.roll;
            r.drift = wrap_pi(s.attitude.heading - s.track);
        }
        return turned > deg2rad(760.0);
    });
    r.steady_diameter = x_max - x_min;
    return r;
}

}  // namespace fmperf
