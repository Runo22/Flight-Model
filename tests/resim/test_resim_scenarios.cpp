// The live demos of examples/resim flown like in the game: the scenario is started once, then
// the world advances 60 frames a second and the scenario logic does the rest. The game's API
// is a stand-in (stub/). Each run checks the outcome the scenario is meant to show.
//
// Usage: fm_resim_scenarios <scenario> | --restart | --all | --list
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>

#include "fm/ecs/flight_module.hpp"
#include "helpers.hpp"
#include "scenarios.hpp"

namespace
{

    ecs_world_t* game_world = nullptr;

    int failures = 0;

    void check(bool ok, const char* what)
    {
        std::printf("  [%s] %s\n", ok ? " OK " : "FAIL", what);
        if (!ok)
        {
            ++failures;
        }
    }

    // The game: a world with the flight module, the transform binding and the demo helpers.
    struct Game
    {
        flecs::world world;
        Game()
        {
            game_world = world.c_ptr();
            fm::ecs::load(world);
            fm::ecs::bind_geo_transform<resim::core::Transform>(world);
            bind_world(world);
        }
        ~Game()
        {
            unbind_world();
            fm::ecs::unload(world);
            game_world = nullptr;
        }
        flecs::entity operator[](const char* name) const
        {
            return world.lookup(name);
        }

        // Frames for `seconds`, or until `done` holds; `each_frame` observes every frame.
        double run(double seconds, const std::function<bool()>& done = {}, const std::function<void()>& each_frame = {})
        {
            int frames = 0;
            for (; frames < static_cast<int>(seconds * 60.0); ++frames)
            {
                world.progress(1.0f / 60.0f);
                if (each_frame)
                {
                    each_frame();
                }
                if (done && done())
                {
                    break;
                }
            }
            return frames / 60.0;
        }
        // Until the scenario logic ends; false when it is still running after `seconds`.
        bool finish(const char* scenario, double seconds = 4000.0)
        {
            run(seconds, [&] { return !scenario_running(scenario); });
            return !scenario_running(scenario);
        }
    };

    bool orbits(flecs::entity e, double lat, double lon, double radius, double alt, double tolerance)
    {
        return pose(e).mode == fm::FlightMode::Loiter && std::abs(distance(e, lat, lon) - radius) < tolerance && std::abs(transform(e).alt_m - alt) < 50.0;
    }

    // --- One check per scenario -----------------------------------------------------------

    void anka3_tour()
    {
        Game g;
        check(demo::run_scenario("no_such_scenario") == "[ERR] Failed to find matching scenario: no_such_scenario", "an unknown name gives the error text");
        check(demo::run_scenario("anka3_tour") == demo::demo_scenarios[0].title, "run_scenario returns the title");
        // The route is shown from the start of the takeoff, with the same points once flown.
        bool shown_on_ground = true;
        resim::core::WaypointList on_ground;
        g.run(60.0, {}, [&] {
            const auto* l = g["Anka-3"].try_get<resim::core::WaypointList>();
            if (pose(g["Anka-3"]).mode == fm::FlightMode::Takeoff)
            {
                shown_on_ground = shown_on_ground && l && l->waypoints.size() == 2 && l->current_index == 0;
                on_ground = l ? *l : resim::core::WaypointList {};
            }
        });
        check(shown_on_ground, "WaypointList shows the route during the whole takeoff, on its first waypoint");
        const auto* flying = g["Anka-3"].try_get<resim::core::WaypointList>();
        const auto same = [](const resim::core::Waypoint& a, const resim::core::Waypoint& b) {
            return a.latitude_deg == b.latitude_deg && a.longitude_deg == b.longitude_deg && a.altitude_m == b.altitude_m;
        };
        check(pose(g["Anka-3"]).mode == fm::FlightMode::Route && flying && flying->current_index == 0 && flying->waypoints.size() == 2 && same(flying->waypoints[0], on_ground.waypoints[0]) && same(flying->waypoints[1], on_ground.waypoints[1]), "the same points once the route is flown");
        check(g.finish("anka3_tour"), "flew the route and landed (scenario logic ended)");
        const fm::Runway rwy = runway({ 40.06563462482789, 32.55461880413686, ankara::terrain(40.06563462482789, 32.55461880413686) }, 32.5);
        check(landed_on(g["Anka-3"], rwy, ankara::kAkinci), "Anka-3 landed on runway 03");
        const auto* list = g["Anka-3"].try_get<resim::core::WaypointList>();
        check(list && list->waypoints.size() == 2 && list->current_index == 1 && !list->loop, "WaypointList shows the route, on its last waypoint");
    }

    void formation_goto()
    {
        Game g;
        demo::run_scenario("formation_goto");
        g.run(900.0);
        flecs::entity kaan = g["Kaan"];
        check(pose(kaan).mode == fm::FlightMode::Loiter && distance(kaan, 40.0668, 32.5555) < 7000.0, "Kaan orbits Akinci");
        check(slot_error(g["Kaan Wing-1"], kaan, fm::formation::vic(0, 40.0)) < 60.0 && slot_error(g["Kaan Wing-2"], kaan, fm::formation::vic(1, 40.0)) < 60.0, "both Anka-3 in their vic slots");
        check(orbits(g["Hurkus Golbasi"], 39.99, 32.64, 2000.0, 6000.0, 300.0), "Hurkus orbits Golbasi at 6,000 m, 2 km radius");
    }

    void syria_orbit()
    {
        Game g;
        demo::run_scenario("syria_orbit");
        // Spawned airborne: the game's transform has the trimmed attitude at once (it used to
        // be level for the first frame, then jump to the model's pitch).
        const double pitch = transform(g["Hurjet Syria"]).pitch_deg;
        check(pitch > 0.5 && std::abs(pitch - fm::rad2deg(fm::ecs::agent(g["Hurjet Syria"])->state().attitude.pitch)) < 0.01, "spawned with its trimmed attitude");
        g.run(600.0);
        check(orbits(g["Hurjet Syria"], 36.79, 39.85, 5000.0, 10000.0, 300.0), "Hurjet orbits at 10,000 m, 5 km radius, 650 km from the origin");
    }

    void rejoin()
    {
        Game g;
        demo::run_scenario("rejoin");
        check(g.finish("rejoin"), "formed up, split and landed (scenario logic ended)");
        const fm::Runway akinci = runway(ankara::kAkinci);
        check(landed_on(g["Hurjet"], akinci, ankara::kAkinci) && landed_on(g["Anka-3 Rejoin"], akinci, ankara::kAkinci), "Hurjet and Anka-3 landed at Akinci");
        g.run(120.0);
        check(orbits(g["Kizilelma"], ankara::kMogan.lat, ankara::kMogan.lon, 4000.0, 3600.0, 400.0), "Kizilelma orbits Mogan lake at 3,600 m");
    }

    void circuit()
    {
        Game g;
        demo::run_scenario("circuit");
        g.run(1.0);
        const auto* list = g["Hurkus"].try_get<resim::core::WaypointList>();
        check(pose(g["Hurkus"]).phase == fm::FlightPhase::Taxi && list && list->waypoints.size() == 2 && list->current_index == 0, "the tour is shown while taxiing out");
        check(g.finish("circuit"), "taxi, takeoff, tour, overhead, taxi back (scenario logic ended)");
        flecs::entity hurkus = g["Hurkus"];
        check(pose(hurkus).phase == fm::FlightPhase::Parked && (position(hurkus) - field(ankara::kAkinci, 400.0, -250.0)).xy().length() < 3.0, "parked again on its apron spot");
    }

    void waypoints()
    {
        Game g;
        demo::run_scenario("waypoints");
        double max_bank = 0.0;
        flecs::entity anka = g["Anka"];
        const bool ended = [&] {
            g.run(5000.0, [&] { return !scenario_running("waypoints"); }, [&] {
            if (pose(anka).mode == fm::FlightMode::Route){
                max_bank = std::max(max_bank, std::abs(pose(anka).roll));
} });
            return !scenario_running("waypoints");
        }();
        check(ended && landed_on(anka, runway(ankara::kAkinci), ankara::kAkinci), "flew the tour and landed back at Akinci");
        check(max_bank < 26.0, "route turns at the 25 deg navigation bank");
        const auto* list = anka.try_get<resim::core::WaypointList>();
        check(list && list->waypoints.size() == 4 && list->current_index == 3, "WaypointList shows the four waypoints, on the last");
    }

    void flypast()
    {
        Game g;
        demo::run_scenario("flypast");
        check(g.finish("flypast"), "takeoff, flypast and recovery (scenario logic ended)");
        bool all = true;
        for (const char* v : { "Viper-1", "Viper-2", "Viper-3", "Viper-4" })
        {
            all = all && landed_on(g[v], runway(ankara::kAkinci), ankara::kAkinci);
        }
        check(all, "all four F-16s landed at Akinci");
    }

    void traffic()
    {
        Game g;
        demo::run_scenario("traffic");
        const char* names[] = { "Hurkus Traffic", "Anka-3 Traffic", "Anka Traffic", "Aksungur Traffic" };
        const double speeds[] = { 110.0, 150.0, 50.0, 52.0 }; // m/s TAS, commanded
        double closest[2] = { 1e9, 1e9 };                     // without, with collision avoidance
        int frames_in_second_pass = 0;
        bool second_pass_commanded = false;
        g.run(1000.0, [] { return !scenario_running("traffic"); }, [&] {
        const bool avoidance = g.world.try_get<fm::ecs::FlightSettings>()->collision_avoidance;
        for (int a = 0; a < 4; ++a){
            for (int b = a + 1; b < 4; ++b){
                closest[avoidance] = std::min(closest[avoidance], distance(g[names[a]], g[names[b]]));
}}
        // The second pass starts in the scenario logic (a system): 30 s in, every aircraft
        // flies its go_to at its commanded speed (spawn's setup reached the new agent).
        if (avoidance && ++frames_in_second_pass == 30 * 60) {
            second_pass_commanded = true;
            for (int k = 0; k < 4; ++k) {
                const fm::ecs::FlightPose& p = pose(g[names[k]]);
                std::printf("  second pass, 30 s in: %s %s at %.1f m/s (commanded %.0f)\n", names[k],
                            fm::to_string(p.mode), p.airspeed, speeds[k]);
                second_pass_commanded = second_pass_commanded && p.mode == fm::FlightMode::GoTo &&
                                        std::abs(p.airspeed - speeds[k]) < 0.05 * speeds[k];
            }
        } });
        std::printf("  closest approach: %.0f m without, %.0f m with collision avoidance\n", closest[0], closest[1]);
        check(!scenario_running("traffic"), "both encounters flown (scenario logic ended)");
        check(second_pass_commanded, "second pass: go_to at the commanded speeds");
        check(closest[0] < 30.0, "near collisions without collision avoidance");
        check(closest[1] > 100.0, "kept 100 m apart with collision avoidance");
        check(g.world.try_get<fm::ecs::FlightSettings>()->collision_avoidance, "collision avoidance on again");
        bool past = true;
        for (const char* n : names)
        {
            past = past && distance(g[n], ankara::kKizilay) > 2000.0;
        }
        check(past, "everyone flew on past Kizilay");
    }

    void lowlevel()
    {
        Game g;
        demo::run_scenario("lowlevel");
        double min_agl = 1e9;
        flecs::entity atak = g["T129"];
        g.run(2000.0, [] { return !scenario_running("lowlevel"); }, [&] {
        const std::size_t leg = fm::ecs::agent(atak)->rotorcraft()->route().index();
        if (pose(atak).mode == fm::FlightMode::Route && leg >= 1 && leg <= 2){
            min_agl = std::min(min_agl, height_above_terrain(atak));
} });
        check(!scenario_running("lowlevel"), "flew to Elmadag, hovered, landed (scenario logic ended)");
        check(min_agl > 15.0, "terrain following, never below 15 m");
        check(pose(atak).phase == fm::FlightPhase::Landed && distance(atak, 39.8550, 33.2700) < 5.0, "landed on the ridge");
    }

    void recovery()
    {
        Game g;
        demo::run_scenario("recovery");
        bool went_around = false;
        g.run(2400.0, [] { return !scenario_running("recovery"); }, [&] { went_around = went_around || pose(g["A400M"]).phase == fm::FlightPhase::GoAround || pose(g["Aksungur Recovery"]).phase == fm::FlightPhase::GoAround; });
        check(!scenario_running("recovery"), "both landed (scenario logic ended)");
        check(!went_around, "stabilised approaches, no go-around");
    }

    void relief()
    {
        Game g;
        demo::run_scenario("relief");
        check(g.finish("relief"), "relieved on station, Anka home (scenario logic ended)");
        check(landed_on(g["Anka Station"], runway(ankara::kAkinci), ankara::kAkinci), "Anka landed at Akinci");
        check(orbits(g["Aksungur Relief"], 39.8800, 32.4000, 2500.0, 2700.0, 500.0), "Aksungur stays on station, 300 m higher");
    }

    // Running a scenario again while its last run is half done starts it clean.
    void restart()
    {
        Game g;
        demo::run_scenario("rejoin");
        g.run(600.0); // split, landing sequence under way
        check(fm::ecs::flight_sequence_active(g["Hurjet"]), "landing sequence under way");
        demo::run_scenario("rejoin");
        check(!fm::ecs::flight_sequence_active(g["Hurjet"]) && g["Anka-3 Rejoin"].has<fm::ecs::FollowLeader>(), "restart: old sequence gone, the UAVs follow the leader again");
        check(g.finish("rejoin") && landed_on(g["Hurjet"], runway(ankara::kAkinci), ankara::kAkinci), "the second run lands too");
    }

    // Every scenario in one world at once, as in a busy session; and the frame time.
    void all()
    {
        Game g;
        for (const demo::Scenario& s : demo::demo_scenarios)
        {
            s.run();
        }
        const auto start = std::chrono::steady_clock::now();
        const double seconds = g.run(4000.0, [] {
            for (const demo::Scenario& s : demo::demo_scenarios)
            {
                if (scenario_running(s.name))
                {
                    return false;
                }
            }
            return true;
        });
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        std::printf("  %.0f s flown, %.3f ms per frame\n", seconds, ms / (seconds * 60.0));
        bool ended = true;
        for (const demo::Scenario& s : demo::demo_scenarios)
        {
            if (scenario_running(s.name))
            {
                std::printf("  still running: %s\n", s.name);
                ended = false;
            }
        }
        check(ended, "every scenario ended");
    }

    struct Test
    {
        const char* name;
        void (*run)();
    };

    const Test kTests[] = {
        { "anka3_tour", anka3_tour },
        { "formation_goto", formation_goto },
        { "syria_orbit", syria_orbit },
        { "rejoin", rejoin },
        { "circuit", circuit },
        { "waypoints", waypoints },
        { "flypast", flypast },
        { "traffic", traffic },
        { "lowlevel", lowlevel },
        { "recovery", recovery },
        { "relief", relief },
        { "--restart", restart },
        { "--all", all },
    };

} // namespace

namespace resim::core
{
    flecs::world get_world()
    {
        return flecs::world(game_world);
    }
} // namespace resim::core

int main(int argc, char** argv)
{
    if (argc < 2 || std::strcmp(argv[1], "--list") == 0)
    {
        for (const Test& t : kTests)
        {
            std::printf("%s\n", t.name);
        }
        return argc < 2 ? 2 : 0;
    }
    for (const Test& t : kTests)
    {
        if (std::strcmp(t.name, argv[1]) == 0)
        {
            std::printf("== %s\n", t.name);
            t.run();
            return failures == 0 ? 0 : 1;
        }
    }
    std::fprintf(stderr, "no such test; --list shows them\n");
    return 2;
}
