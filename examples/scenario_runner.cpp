// Headless scenarios that print a CSV trace (t, vehicle, position, attitude, speeds, mode).
// Usage: fm_scenarios <takeoff_route|formation|drone_survey|sortie|flight> > trace.csv
// sortie and flight check their own result and exit with 1 when it is wrong.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "fm/flight_agent.hpp"
#include "fm/presets.hpp"

using namespace fm;

namespace {

constexpr double kDt = 1.0 / 60.0;

void header() {
    std::printf("t,vehicle,x,y,z,heading_deg,pitch_deg,roll_deg,airspeed,vertical_speed,mode,phase,"
                "gear\n");
}

void row(double t, int id, const FlightAgent& a) {
    const FlightState& s = a.state();
    std::printf("%.2f,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%.2f\n", t, id, s.position.x,
                s.position.y, s.position.z, rad2deg(s.attitude.heading), rad2deg(s.attitude.pitch),
                rad2deg(s.attitude.roll), s.airspeed, s.vertical_speed, static_cast<int>(a.mode()),
                static_cast<int>(s.phase), s.gear_position);
}

double hilly_terrain(double x, double y) { return 100.0 + 80.0 * std::sin(x / 4000.0) * std::cos(y / 5000.0); }

Waypoint waypoint(double x, double y, double z) {
    Waypoint w;
    w.position = {x, y, z};
    return w;
}

// Airliner takes off from a runway, flies a route and lands back on the same runway.
void takeoff_route() {
    InitialConditions ic;
    ic.position = {0, 0, hilly_terrain(0, 0)};
    ic.heading = deg2rad(90.0);
    ic.on_ground = true;
    FlightAgent a(presets::airliner_jet(), ic);
    TakeoffPlan plan;
    plan.runway = Runway{ic.position, ic.heading};
    Route r;
    r.waypoints = {waypoint(25000, 0, 3000), waypoint(40000, 20000, 4000),
                   waypoint(20000, 40000, 4000)};
    r.on_end = EndBehavior::Land;
    r.runway = Runway{ic.position, ic.heading};
    plan.then = r;
    a.takeoff(plan);
    for (int i = 0; i < static_cast<int>(2400.0 / kDt); ++i) {
        Environment env;
        env.terrain_elevation = hilly_terrain(a.state().position.x, a.state().position.y);
        a.step(kDt, env);
        if (i % 60 == 0) row(i * kDt, 0, a);
    }
}

// Four light aircraft in a vic formation behind a leader that flies a box pattern.
void formation_flight() {
    const FixedWingParams p = presets::light_propeller();
    FlightAgent lead(p, {{0, 0, 1500}, 0.0, 50.0, false});
    lead.set_speed(50.0);
    Route box;
    box.waypoints = {waypoint(0, 6000, 1500), waypoint(6000, 6000, 1500),
                     waypoint(6000, 0, 1500), waypoint(0, 0, 1500)};
    box.loop = true;
    lead.follow_route(box);
    std::vector<FlightAgent> wing;
    for (int k = 0; k < 4; ++k) {
        wing.emplace_back(p, InitialConditions{{-300.0 * (k + 1), -500.0, 1500}, 0.0, 50.0, false});
        wing.back().join_formation(formation::vic(k, 50.0));
    }
    for (int i = 0; i < static_cast<int>(900.0 / kDt); ++i) {
        lead.step(kDt, {});
        for (auto& w : wing) {
            w.set_leader_state(lead.as_leader());
            w.step(kDt, {});
        }
        if (i % 60 == 0) {
            row(i * kDt, 0, lead);
            for (int k = 0; k < 4; ++k) row(i * kDt, k + 1, wing[k]);
        }
    }
}

// Quadcopter survey: takes off, flies a lawnmower pattern at 40 m above terrain, lands.
void drone_survey() {
    InitialConditions ic;
    ic.position = {0, 0, hilly_terrain(0, 0)};
    ic.on_ground = true;
    FlightAgent a(presets::multirotor(), ic);
    Route r;
    for (int lane = 0; lane < 4; ++lane) {
        const double x = lane * 40.0;
        Waypoint a1 = waypoint(x, lane % 2 ? 300 : 0, 40);
        Waypoint a2 = waypoint(x, lane % 2 ? 0 : 300, 40);
        a1.altitude_ref = a2.altitude_ref = AltitudeRef::AboveTerrain;
        r.waypoints.push_back(a1);
        r.waypoints.push_back(a2);
    }
    r.on_end = EndBehavior::Land;
    a.follow_route(r);
    for (int i = 0; i < static_cast<int>(240.0 / kDt); ++i) {
        Environment env;
        env.terrain_elevation = hilly_terrain(a.state().position.x, a.state().position.y);
        env.wind = {4.0, 1.0, 0.0};
        a.step(kDt, env);
        if (i % 30 == 0) row(i * kDt, 0, a);
    }
}

// Airfield of the sortie and flight scenarios: runway east from (0, 0) on flat ground.
constexpr double kField = 100.0;
const Runway kRunway{{0.0, 0.0, kField}, deg2rad(90.0)};

// Point `along` the runway from its start and `right` of it (south: -y).
Vec3 field(double along, double right) { return {along, -right, kField}; }

Environment field_env() {
    Environment env;
    env.terrain_elevation = kField;
    return env;
}

// F-16 sortie: out of its hangar along the taxiway, lined up and off, a route, back for an
// overhead break, the landing roll ends 1500 m down the runway, back into the hangar.
bool sortie() {
    InitialConditions ic;
    ic.position = field(-80, 150);  // hangar south of the runway, nose north
    ic.heading = 0.0;
    ic.on_ground = true;
    FlightAgent a(presets::single_engine_fighter(), ic);
    TakeoffPlan plan;
    plan.taxi = {field(-80, 60), field(0, 60)};
    plan.runway = kRunway;
    plan.climb_height = 600.0;
    Route r;
    r.waypoints = {waypoint(20000, 8000, 2500), waypoint(8000, 16000, 2500)};
    r.on_end = EndBehavior::Loiter;
    plan.then = r;
    a.takeoff(plan);
    bool landing = false;
    bool taxi_in = false;
    for (int i = 0; i < static_cast<int>(1200.0 / kDt); ++i) {
        a.step(kDt, field_env());
        if (i % 60 == 0) row(i * kDt, 0, a);
        if (!landing && a.mode() == FlightMode::Loiter) {
            LandingOptions o;
            o.pattern = LandingPattern::Overhead;
            o.break_distance = 300.0;  // m past the threshold
            o.stop_distance = 1500.0;  // m past the threshold
            landing = a.land(kRunway, o);
        }
        if (landing && !taxi_in && a.state().phase == FlightPhase::Landed) {
            const std::vector<Vec3> to_hangar{field(1550, 60), field(-80, 60), field(-80, 150)};
            taxi_in = a.taxi(to_hangar, deg2rad(180.0));  // nose first, south
        }
        if (taxi_in && a.state().phase == FlightPhase::Parked) {
            row(i * kDt, 0, a);
            break;
        }
    }
    return taxi_in && a.state().phase == FlightPhase::Parked &&
           (a.state().position - field(-80, 150)).xy().length() < 3.0;
}

// Four F-16s with the core API only (fm::ecs::takeoff_flight / land_flight do this in
// Flecs): taxi from their shelters, line up staggered and wait, brake releases 10 s apart,
// rejoin in echelon, then an overhead break with touchdowns 12 s apart on alternating sides
// of the runway.
bool flight() {
    const FixedWingParams p = presets::single_engine_fighter();
    constexpr int n = 4;
    constexpr double interval = 12.0;
    std::vector<FlightAgent> f;
    std::vector<Runway> lanes;  // landing: each on its half of the runway
    for (int k = 0; k < n; ++k) {
        const double along = (n - 1 - k) * 150.0;     // the leader furthest down the runway
        const double side = k % 2 == 0 ? -8.0 : 8.0;  // left / right half of the runway
        InitialConditions ic;
        ic.position = field(along, 250);  // shelter south of its line-up point, nose north
        ic.on_ground = true;
        f.emplace_back(p, ic);
        lanes.push_back(Runway{field(0, side), kRunway.heading});
        TakeoffPlan plan;
        plan.taxi = {field(along, 40)};
        plan.runway = Runway{field(along, side), kRunway.heading};  // line-up point
        plan.climb_height = 600.0;
        plan.hold = true;  // line up and wait
        if (k == 0) {
            Route r;
            r.waypoints = {waypoint(15000, 6000, 2000)};
            r.on_end = EndBehavior::Loiter;
            plan.then = r;
        }
        f.back().takeoff(plan);
    }
    std::vector<FormationSlot> slots;
    for (int k = 1; k < n; ++k) slots.push_back(formation::echelon_right(k - 1, 80.0));
    const auto slot_position = [&](int k) {
        const FlightState& l = f[0].state();
        const Vec2 fwd = heading_vector(l.attitude.heading);
        const Vec3& o = slots[k - 1].offset;
        const Vec2 at = l.position.xy() + right_of(fwd) * o.x + fwd * o.y;
        return Vec3{at.x, at.y, l.position.z + o.z};
    };
    double release = -1.0;
    bool landing = false;
    bool wingmen_landing = false;
    std::vector<double> touchdown(n, -1.0);
    for (int i = 0; i < static_cast<int>(1500.0 / kDt); ++i) {
        const double t = i * kDt;
        // Brake releases once everybody is lined up.
        const bool lined_up = std::all_of(f.begin(), f.end(), [](const FlightAgent& a) {
            return a.state().phase == FlightPhase::LineUp && a.state().ground_speed < 0.3;
        });
        if (release < 0.0 && lined_up) release = t;
        for (int k = 0; k < n; ++k) {
            if (release >= 0.0 && t >= release + k * 10.0 && t < release + k * 10.0 + kDt)
                f[k].release_takeoff();
        }
        // Wingmen join the leader once their takeoff is done; the leader slows for them.
        double extent = 0.0;
        bool rejoining = false;
        for (int k = 1; k < n; ++k) {
            FlightAgent& w = f[k];
            if (!landing && w.mode() == FlightMode::Cruise && !w.state().on_ground && release >= 0.0)
                w.join_formation(slots[k - 1]);
            if (w.mode() != FlightMode::Formation) continue;
            w.set_leader_state(f[0].as_leader());
            extent = std::max(extent, std::abs(slots[k - 1].offset.x));
            rejoining = rejoining || (w.state().position - slot_position(k)).length() > 300.0;
        }
        if (extent > 0.0 || rejoining) f[0].set_formation_lead(extent, rejoining);
        // Overhead recovery once joined up (the slots are already an echelon right, so the
        // break goes left).
        const Runway& runway = kRunway;
        if (!landing && t > 450.0) {
            LandingOptions o;
            o.pattern = LandingPattern::Overhead;
            o.break_distance = 300.0;
            o.initial_speed = f[0].fixed_wing()->overhead_initial_speed(runway);
            o.initial_offset = 8.0;  // its lane is 8 m left: fly the initial on the centerline
            o.stop_distance = 1400.0 + (n - 1) * 150.0;
            landing = f[0].land(lanes[0], o);
        }
        // The wingmen leave the echelon once the leader is established on the initial.
        if (landing && !wingmen_landing) {
            const FixedWingAgent& lead = *f[0].fixed_wing();
            const double offset = dot(lead.state().position.xy() - runway.start.xy(),
                                      right_of(heading_vector(runway.heading)));
            const bool established =
                lead.landing_stage() == FixedWingAgent::LandingStage::Initial &&
                std::abs(wrap_pi(lead.state().track - runway.heading)) < deg2rad(10.0) &&
                std::abs(offset) < 150.0;
            if (established) {
                const double speed = lead.overhead_initial_speed(runway);
                const double spacing = lead.break_spacing(runway, interval, speed);
                for (int k = 1; k < n; ++k) {
                    LandingOptions o;
                    o.pattern = LandingPattern::Overhead;
                    o.break_distance = 300.0 + k * spacing;  // later breaks, later touchdowns
                    o.initial_speed = speed;
                    o.initial_offset = slots[k - 1].offset.x - (k % 2 == 0 ? -8.0 : 8.0);
                    o.stop_distance = 1400.0 + (n - 1 - k) * 150.0;  // one behind the other
                    f[k].land(lanes[k], o);
                }
                wingmen_landing = true;
            }
        }
        for (int k = 0; k < n; ++k) {
            f[k].step(kDt, field_env());
            if (landing && touchdown[k] < 0.0 && f[k].state().on_ground) touchdown[k] = t;
            if (i % 60 == 0) row(t, k, f[k]);
        }
        const bool landed = std::all_of(f.begin(), f.end(), [](const FlightAgent& a) {
            return a.state().phase == FlightPhase::Landed;
        });
        if (landing && landed) break;
    }
    bool ok = touchdown[0] > 0.0;
    for (int k = 1; k < n; ++k)
        ok = ok && std::abs(touchdown[k] - touchdown[k - 1] - interval) < 0.25 * interval;
    return ok;
}

}  // namespace

int main(int argc, char** argv) {
    const char* name = argc > 1 ? argv[1] : "takeoff_route";
    header();
    if (std::strcmp(name, "takeoff_route") == 0) takeoff_route();
    else if (std::strcmp(name, "formation") == 0) formation_flight();
    else if (std::strcmp(name, "drone_survey") == 0) drone_survey();
    else if (std::strcmp(name, "sortie") == 0 || std::strcmp(name, "flight") == 0) {
        const bool ok = std::strcmp(name, "sortie") == 0 ? sortie() : flight();
        if (!ok) std::fprintf(stderr, "scenario '%s' did not end as expected\n", name);
        return ok ? 0 : 1;
    } else {
        std::fprintf(stderr, "unknown scenario '%s' (takeoff_route|formation|drone_survey|"
                             "sortie|flight)\n", name);
        return 1;
    }
    return 0;
}
