// Headless scenarios that print a CSV trace (t, vehicle, position, attitude, speeds, mode).
// Usage: fm_scenarios <takeoff_route|formation|drone_survey> > trace.csv
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
    std::printf("t,vehicle,x,y,z,heading_deg,pitch_deg,roll_deg,airspeed,vertical_speed,mode,phase\n");
}

void row(double t, int id, const FlightAgent& a) {
    const FlightState& s = a.state();
    std::printf("%.2f,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d\n", t, id, s.position.x,
                s.position.y, s.position.z, rad2deg(s.attitude.heading), rad2deg(s.attitude.pitch),
                rad2deg(s.attitude.roll), s.airspeed, s.vertical_speed, static_cast<int>(a.mode()),
                static_cast<int>(s.phase));
}

double hilly_terrain(double x, double y) { return 100.0 + 80.0 * std::sin(x / 4000.0) * std::cos(y / 5000.0); }

Waypoint waypoint(double x, double y, double z) {
    Waypoint w;
    w.position = {x, y, z};
    return w;
}

// Airliner takes off from a runway, flies a route and continues straight afterwards.
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
    plan.then = r;
    a.takeoff(plan);
    for (int i = 0; i < static_cast<int>(900.0 / kDt); ++i) {
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

}  // namespace

int main(int argc, char** argv) {
    const char* name = argc > 1 ? argv[1] : "takeoff_route";
    header();
    if (std::strcmp(name, "takeoff_route") == 0) takeoff_route();
    else if (std::strcmp(name, "formation") == 0) formation_flight();
    else if (std::strcmp(name, "drone_survey") == 0) drone_survey();
    else {
        std::fprintf(stderr, "unknown scenario '%s' (takeoff_route|formation|drone_survey)\n", name);
        return 1;
    }
    return 0;
}
