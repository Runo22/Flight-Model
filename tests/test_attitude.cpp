// Attitude continuity of the fixed-wing AI: what a game shows of the aircraft (pitch and roll)
// never jumps from one frame to the next, and moves at the rates an autopilot would use.
#include <algorithm>
#include <cmath>
#include <vector>

#include "fm/fixed_wing.hpp"
#include "fm/presets.hpp"
#include "harness.hpp"

using namespace fm;

namespace
{
    constexpr double kDt = 1.0 / 60.0;

    const FixedWingParams kAllFixedWing[] = {
        presets::single_engine_fighter(), presets::twin_engine_fighter(), presets::fast_jet(), presets::jet_trainer(), presets::turboprop_trainer(), presets::jet_uav(), presets::propeller_uav(), presets::twin_propeller_uav(), presets::airliner_jet(), presets::regional_turboprop(), presets::light_propeller()
    };

    double nav_bank_of(const FixedWingParams& p)
    {
        return std::min(p.nav_bank > 0.0 ? p.nav_bank : deg2rad(30.0), p.bank_max);
    }

    double nav_roll_rate_of(const FixedWingParams& p)
    {
        return std::min(p.nav_roll_rate > 0.0 ? p.nav_roll_rate : nav_bank_of(p) / 3.0, p.roll_rate_max);
    }

    struct TakeoffClimb
    {
        double max_pitch_step = 0.0;     // deg per frame
        double max_pitch = 0.0;          // deg, takeoff and the first minute of the route
        double min_route_vs = 1e9;       // m/s, first 30 s of the route
        double min_route_n = 1e9;        // g, first 30 s of the route
        double route_start_height = 0.0; // m above the runway
        bool route_started = false;
    };

    // Takeoff to the north, then a route to a waypoint 15 km ahead, 1,500 m above the runway.
    TakeoffClimb takeoff_then_climb(const FixedWingParams& p)
    {
        InitialConditions ic;
        ic.on_ground = true;
        FixedWingAgent a(p, ic);
        Route r;
        Waypoint w;
        w.position = { 3000.0, 15000.0, 1500.0 };
        r.waypoints = { w };
        TakeoffPlan plan;
        plan.runway = Runway { { 0, 0, 0 }, 0.0 };
        plan.then = r;
        a.takeoff(plan);
        TakeoffClimb out;
        double pitch = a.state().attitude.pitch;
        double route_time = -1.0;
        Environment env;
        env.terrain_elevation = 0.0;
        for (int i = 0; i < 600 * 60 && route_time < 60.0; ++i)
        {
            a.step(kDt, env);
            const FlightState& s = a.state();
            out.max_pitch_step = std::max(out.max_pitch_step, rad2deg(std::abs(s.attitude.pitch - pitch)));
            pitch = s.attitude.pitch;
            if (!s.on_ground)
            {
                out.max_pitch = std::max(out.max_pitch, rad2deg(pitch));
            }
            if (a.mode() == FlightMode::Route)
            {
                if (route_time < 0.0)
                {
                    route_time = 0.0;
                    out.route_started = true;
                    out.route_start_height = s.position.z - p.gear_height;
                }
                route_time += kDt;
                if (route_time < 30.0)
                {
                    out.min_route_vs = std::min(out.min_route_vs, s.vertical_speed);
                    out.min_route_n = std::min(out.min_route_n, s.load_factor);
                }
            }
        }
        return out;
    }

    struct RouteRoll
    {
        double max_roll_rate = 0.0;  // rad/s
        double max_roll_accel = 0.0; // rad/s^2
        int reversals = 0;           // roll direction changes with more than 2 deg of swing
        int turns = 0;
    };

    // An airborne route with turns of 45, 90 and 135 deg right, 90 left and a reversal (180).
    RouteRoll route_turns(const FixedWingParams& p, bool fly_over)
    {
        InitialConditions ic;
        ic.position = { 0, 0, 1500 };
        ic.heading = 0.0;
        ic.speed = p.cruise_speed;
        FixedWingAgent a(p, ic);
        Route r;
        double x = 0.0, y = 9000.0, heading = 0.0;
        const auto add = [&] {
            Waypoint w;
            w.position = { x, y, 1500.0 };
            w.fly_over = fly_over;
            r.waypoints.push_back(w);
        };
        add();
        for (const double turn : { 45.0, 90.0, 135.0, -90.0, 180.0 })
        {
            heading += deg2rad(turn);
            x += std::sin(heading) * 12000.0;
            y += std::cos(heading) * 12000.0;
            add();
        }
        a.follow_route(r);
        RouteRoll out;
        out.turns = 5;
        double roll = a.state().attitude.roll, rate = 0.0, extreme = roll;
        int direction = 0;
        for (int i = 0; i < 3000 * 60 && a.mode() == FlightMode::Route; ++i)
        {
            a.step(kDt, {});
            const double now = a.state().attitude.roll;
            const double new_rate = (now - roll) / kDt;
            out.max_roll_rate = std::max(out.max_roll_rate, std::abs(new_rate));
            out.max_roll_accel = std::max(out.max_roll_accel, std::abs(new_rate - rate) / kDt);
            const int d = new_rate > deg2rad(0.5) ? 1 : (new_rate < -deg2rad(0.5) ? -1 : 0);
            if (d != 0 && d != direction)
            {
                if (direction != 0 && std::abs(now - extreme) > deg2rad(2.0))
                {
                    ++out.reversals;
                }
                direction = d;
            }
            if (d == direction)
            {
                extreme = now;
            }
            roll = now;
            rate = new_rate;
        }
        return out;
    }
} // namespace

TEST_CASE("attitude: takeoff and the climb on the route without pitch jumps, every preset")
{
    // The pitch used to jump at lift-off (up to 16 deg in one frame: the load factor was set
    // at once, up to the stall), when the flaps came up and when the route began; the jets
    // then pulled up to 44 deg and pushed over at 0 g once on the route (its first leg
    // started at the takeoff's climb height, held there while turning towards the waypoint).
    for (const FixedWingParams& p : kAllFixedWing)
    {
        const TakeoffClimb c = takeoff_then_climb(p);
        CHECK(c.route_started);
        CHECK(c.max_pitch_step < 1.0);
        CHECK(c.max_pitch < 20.0);
        // Climbing on along the route from where the takeoff ended: no level-off, no
        // push-over (it was 0 g).
        CHECK(c.min_route_vs > 0.0);
        CHECK(c.min_route_n > 0.3); // a short unload to accelerate to the route's speed, no 0 g
    }
}

TEST_CASE("attitude: route turns roll in at the navigation roll rate, without snaps or chatter")
{
    // Every turn used to start at the full roll rate in one frame (a fighter: 0 to 240 deg/s),
    // and the turn towards a waypoint the navigation bank would miss chattered between two
    // guidance laws (35 to 52 deg of bank and back every few frames).
    for (const FixedWingParams& p : kAllFixedWing)
    {
        for (const bool fly_over : { false, true })
        {
            const RouteRoll r = route_turns(p, fly_over);
            const double rate = nav_roll_rate_of(p);
            CHECK(r.max_roll_rate < rate + deg2rad(0.5));
            CHECK(r.max_roll_accel < 1.05 * rate / 0.5 + deg2rad(1.0)); // the rate builds up in 0.5 s
            CHECK(r.reversals <= 3 * r.turns);
        }
    }
}

TEST_CASE("attitude: an airborne agent starts with its trimmed attitude")
{
    // The first frame shows the attitude the agent flies on, not a level one.
    for (const FixedWingParams& p : kAllFixedWing)
    {
        InitialConditions ic;
        ic.position = { 0, 0, 2000 };
        ic.speed = p.cruise_speed;
        FixedWingAgent a(p, ic);
        const double pitch = a.state().attitude.pitch;
        a.step(kDt, {});
        CHECK(std::abs(a.state().attitude.pitch - pitch) < deg2rad(0.1));
    }
}
