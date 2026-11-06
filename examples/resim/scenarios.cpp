#include "scenarios.hpp"
///////////////////////////////////////////////////////////////////////////////
// Standart Headers
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>
///////////////////////////////////////////////////////////////////////////////
// Resim Headers
#include "helpers.hpp"
#include <resim/core/components.hpp>
#include <resim/util/logger_utilities.hpp>
///////////////////////////////////////////////////////////////////////////////

namespace demo
{
    using resim::core::PlatformType;

    void anka3_tour()
    {
        // Akinci, runway 03 threshold.
        const double lat = 40.06563462482789, lon = 32.55461880413686, heading = 32.5;
        const fm::GeoPoint threshold { lat, lon, ankara::terrain(lat, lon) };
        flecs::entity uav = spawn("Anka-3", PlatformType::Anka3, { lat, lon, 0.0, 0.0, 0.0, heading }, 0.0, true);
        fm::Route route;
        route.waypoints = { wp(40.13, 32.70, 1500.0),
                            wp(40.17, 32.55, 600.0, fm::AltitudeRef::AboveTerrain) };
        route.on_end = fm::EndBehavior::Land;
        route.runway = runway(threshold, heading);
        fm::ecs::agent(uav)->follow_route(route); // parked: takes off first

        every_frame("anka3_tour", { uav }, [uav, airborne = false](double t) mutable {
            const fm::ecs::FlightPose& p = pose(uav);
            if (!airborne && !p.on_ground)
            {
                airborne = true;
                RESIM_LOG(agent_demo, info, "[anka3_tour] t={:.0f}s airborne", t);
            }
            if (airborne && p.phase == fm::FlightPhase::Landed)
            {
                RESIM_LOG(agent_demo, info, "[anka3_tour] t={:.0f}s landed back at Akinci", t);
                return true;
            }
            return false;
        });
    }

    void formation_goto()
    {
        flecs::entity lead = spawn("Kaan", PlatformType::Kaan, { 40.19, 32.05, 5000.0, 0.0, 0.0, 20.0 }, 270.0);
        for (int k = 0; k < 2; ++k)
        {
            flecs::entity wing = spawn(k == 0 ? "Kaan Wing-1" : "Kaan Wing-2", PlatformType::Anka3, { 40.15 - 0.01 * k, 32.04 - 0.01 * k, 5000.0, 0.0, 0.0, 30.0 }, 320.0);
            fm::ecs::follow(wing, lead, fm::formation::vic(k, 40.0));
        }

        fm::GoToOptions goto_opt;
        goto_opt.speed = 320.0;
        goto_opt.speed_ref = fm::SpeedRef::TrueKnots;
        goto_opt.altitude_ref = fm::AltitudeRef::Keep;
        goto_opt.on_arrival = fm::EndBehavior::Loiter;
        fm::ecs::agent(lead)->go_to(local(40.0668, 32.5555, 835.0), goto_opt); // Akinci

        const fm::GeoPoint golbasi { 39.99, 32.64, 10000.0 };
        flecs::entity hurkus = spawn("Hurkus Golbasi", PlatformType::Hurkus, { 39.90, 32.86, 10000.0, 0.0, 0.0, 270.0 }, 180.0);
        fm::ecs::agent(hurkus)->loiter(local(golbasi), 500.0);
        show_waypoints(hurkus, { golbasi });
    }

    void syria_orbit()
    {
        const fm::GeoPoint center { 36.79, 39.85, 10000.0 };
        flecs::entity hjt = spawn("Hurjet Syria", PlatformType::Hurjet, { 36.78, 39.85, 10000.0, 0.0, 0.0, 270.0 }, 350.0);
        fm::ecs::agent(hjt)->loiter(local(center), 5000.0);
        show_waypoints(hjt, { center });
    }

    void rejoin()
    {
        flecs::entity lead = spawn("Hurjet", PlatformType::Hurjet, { 39.72, 32.38, 3000.0, 0, 0, 70.0 }, 160.0);
        flecs::entity uav1 = spawn("Anka-3 Rejoin", PlatformType::Anka3, { 39.66, 32.28, 2500.0, 0, 0, 40.0 }, 140.0);
        flecs::entity uav2 = spawn("Kizilelma", PlatformType::KizilElma, { 39.80, 32.30, 3400.0, 0, 0, 130.0 }, 140.0);
        const fm::FormationSlot slot1 = fm::formation::echelon_right(0, 60.0);
        const fm::FormationSlot slot2 = fm::formation::echelon_right(1, 60.0);
        fm::ecs::follow(uav1, lead, slot1);
        fm::ecs::follow(uav2, lead, slot2);
        {
            fm::Route r; // as a formation to Mogan lake, orbiting there
            r.waypoints = { wp(ankara::kMogan, 3000.0) };
            r.on_end = fm::EndBehavior::Loiter;
            r.loiter_radius = 3500.0;
            fm::ecs::agent(lead)->follow_route(r);
        }
        const fm::Runway akinci = runway(ankara::kAkinci);

        every_frame("rejoin", { lead, uav1, uav2 }, [=, leader_waited = false, joined_since = -1.0, split = false, touchdown = std::array<double, 2> { -1.0, -1.0 }](double t) mutable {
            if (!split)
            {
                // Orbited while the UAVs were joining.
                leader_waited = leader_waited || fm::ecs::agent(lead)->fixed_wing()->waiting_for_formation();
                const double worst = std::max(slot_error(uav1, lead, slot1), slot_error(uav2, lead, slot2));
                const bool over_lake = pose(lead).mode == fm::FlightMode::Loiter;
                joined_since = worst < 100.0 ? (joined_since < 0.0 ? t : joined_since) : -1.0;
                if (over_lake && joined_since >= 0.0 && t - joined_since > 60.0)
                {
                    // In position: Kizilelma stays over the lake, the others recover.
                    fm::ecs::stop_following(uav2);
                    fm::ecs::agent(uav2)->loiter(local(ankara::kMogan.lat, ankara::kMogan.lon, 3600.0), 4000.0);
                    fm::ecs::FlightLanding landing;
                    landing.interval = 15.0;
                    fm::ecs::land_flight(lead, { uav1 }, akinci, landing);
                    split = true;
                    RESIM_LOG(agent_demo, info, "[rejoin] t={:.0f}s formed up over Mogan lake (worst slot {:.0f} m{}): split", t, worst, leader_waited ? ", the Hurjet waited for the UAVs" : "");
                }
                return false;
            }
            const flecs::entity flight[] = { lead, uav1 };
            for (int k = 0; k < 2; ++k)
            {
                if (touchdown[k] < 0.0 && pose(flight[k]).on_ground)
                {
                    touchdown[k] = t;
                    RESIM_LOG(agent_demo, info, "[rejoin] t={:.0f}s {} touched down", t, flight[k].name().c_str());
                }
            }
            if (landed_on(lead, akinci, ankara::kAkinci) && landed_on(uav1, akinci, ankara::kAkinci))
            {
                RESIM_LOG(agent_demo, info, "[rejoin] t={:.0f}s Hurjet and Anka-3 landed at Akinci, {:.0f} s apart; Kizilelma orbits Mogan lake", t, touchdown[1] - touchdown[0]);
                return true;
            }
            return false;
        });
    }

    void circuit()
    {
        const ankara::Airfield& f = ankara::kAkinci;
        const fm::Runway rwy = runway(f);
        const fm::Vec3 parking = field(f, 400.0, -250.0);
        const fm::GeoPoint apron = geo(parking);
        // Parked on the apron, nose towards the runway.
        flecs::entity hurkus = spawn("Hurkus", PlatformType::Hurkus, { apron.latitude, apron.longitude, 0.0, 0, 0, f.runway_heading + 90.0 }, 0.0, true);
        {
            fm::TakeoffPlan plan;
            plan.taxi = { field(f, 400.0, -120.0), field(f, -40.0, -120.0) };
            plan.runway = rwy;
            fm::Route tour;
            tour.waypoints = { wp(ankara::kKahramankazan, 1700.0),
                               wp(ankara::kEtimesgut, 1500.0) };
            plan.then = tour;
            fm::ecs::agent(hurkus)->takeoff(plan);
        }

        every_frame("circuit", { hurkus }, [=, lined_up = false, airborne = false, overhead = false, broke = false, taxi_in = false, closest = std::array<double, 2> { 1e9, 1e9 }](double t) mutable {
            const fm::ecs::FlightPose& p = pose(hurkus);
            if (!lined_up && p.phase == fm::FlightPhase::LineUp)
            {
                lined_up = true;
                RESIM_LOG(agent_demo, info, "[circuit] t={:.0f}s lined up on runway 03", t);
            }
            if (!airborne && !p.on_ground)
            {
                airborne = true;
                RESIM_LOG(agent_demo, info, "[circuit] t={:.0f}s airborne", t);
            }
            closest[0] = std::min(closest[0], distance(hurkus, ankara::kKahramankazan));
            closest[1] = std::min(closest[1], distance(hurkus, ankara::kEtimesgut));
            // Tour done: back for an overhead break; the roll ends 1,500 m down the runway.
            if (airborne && !overhead && p.mode == fm::FlightMode::Cruise)
            {
                fm::LandingOptions o;
                o.pattern = fm::LandingPattern::Overhead;
                o.break_distance = 500.0;
                o.stop_distance = 1500.0;
                overhead = fm::ecs::agent(hurkus)->land(rwy, o);
                RESIM_LOG(agent_demo, info, "[circuit] t={:.0f}s tour flown ({:.0f} m from Kahramankazan, {:.0f} m from Etimesgut): overhead recovery", t, closest[0], closest[1]);
            }
            if (overhead && !broke && p.mode == fm::FlightMode::Land && fm::ecs::agent(hurkus)->fixed_wing()->landing_stage() == fm::FixedWingAgent::LandingStage::Break)
            {
                broke = true;
                RESIM_LOG(agent_demo, info, "[circuit] t={:.0f}s break", t);
            }
            if (overhead && !taxi_in && p.phase == fm::FlightPhase::Landed)
            {
                const std::vector<fm::Vec3> to_apron { field(f, 1550.0, -120.0),
                                                       field(f, 400.0, -120.0),
                                                       parking };
                taxi_in = fm::ecs::agent(hurkus)->taxi(to_apron, rwy.heading - fm::kPi / 2.0); // nose in
                RESIM_LOG(agent_demo, info, "[circuit] t={:.0f}s landed: taxi back to the apron", t);
            }
            if (taxi_in && p.phase == fm::FlightPhase::Parked)
            {
                RESIM_LOG(agent_demo, info, "[circuit] t={:.0f}s parked, {:.1f} m from its spot", t, (position(hurkus) - parking).xy().length());
                return true;
            }
            return false;
        });
    }

    void waypoints()
    {
        const ankara::Airfield& f = ankara::kAkinci;
        const fm::Runway rwy = runway(f);
        const fm::GeoPoint start = geo(rwy.start);
        flecs::entity anka = spawn("Anka", PlatformType::Anka, { start.latitude, start.longitude, 0.0, 0, 0, f.runway_heading }, 0.0, true);
        static constexpr ankara::Place east { "east of Esenboga", 40.0900, 33.0700 };
        fm::Route r;
        r.waypoints = { wp(ankara::kKahramankazan, 1800.0), wp(east, 1900.0), wp(ankara::kElmadag, 400.0, fm::AltitudeRef::AboveTerrain), wp(ankara::kMogan, 1700.0) };
        r.waypoints[1].speed = 110.0; // knots CAS from here on
        r.waypoints[1].speed_ref = fm::SpeedRef::CalibratedKnots;
        r.waypoints[3].fly_over = true;
        r.on_end = fm::EndBehavior::Land;
        r.runway = rwy;
        {
            fm::TakeoffPlan plan;
            plan.runway = rwy;
            plan.then = r;
            fm::ecs::agent(anka)->takeoff(plan);
        }

        every_frame("waypoints", { anka }, [=, leg = std::size_t { 0 }, closest = std::array<double, 4> { 1e9, 1e9, 1e9, 1e9 }, agl_at_elmadag = 0.0, kcas_on_leg = 0.0, max_bank = 0.0](double t) mutable {
            static constexpr const ankara::Place* points[] = { &ankara::kKahramankazan, &east, &ankara::kElmadag, &ankara::kMogan };
            const fm::ecs::FlightPose& p = pose(anka);
            if (p.mode == fm::FlightMode::Route)
            {
                const std::size_t now = fm::ecs::agent(anka)->fixed_wing()->route().index();
                if (now != leg)
                {
                    RESIM_LOG(agent_demo, info, "[waypoints] t={:.0f}s {} passed ({:.0f} m), on to {}", t, points[leg]->name, closest[leg], points[now]->name);
                    leg = now;
                }
                for (int k = 0; k < 4; ++k)
                {
                    closest[k] = std::min(closest[k], distance(anka, *points[k]));
                }
                if (distance(anka, ankara::kElmadag) < 1500.0)
                {
                    agl_at_elmadag = height_above_terrain(anka);
                }
                // Level on the leg to Elmadag (the climb over the mountain is flown slower).
                if (leg == 2 && distance(anka, east) > 5000.0 && distance(anka, ankara::kElmadag) > 15000.0)
                {
                    kcas_on_leg = p.calibrated_airspeed / 0.514444;
                }
                max_bank = std::max(max_bank, std::abs(p.roll));
            }
            if (p.phase == fm::FlightPhase::Landed)
            {
                RESIM_LOG(agent_demo, info, "[waypoints] t={:.0f}s landed at Akinci. Mogan lake flown over at {:.0f} m, {:.0f} KCAS on the leg to Elmadag, {:.0f} m above Elmadag, max bank {:.0f} deg", t, closest[3], kcas_on_leg, agl_at_elmadag, max_bank);
                return true;
            }
            return false;
        });
    }

    void flypast()
    {
        const ankara::Airfield& f = ankara::kAkinci;
        const fm::Runway rwy = runway(f);
        std::vector<flecs::entity> vipers;
        for (int k = 0; k < 4; ++k)
        { // shelters east of the runway, noses towards it
            const fm::GeoPoint shelter = geo(field(f, 300.0 + 70.0 * k, 260.0));
            const std::string name = "Viper-" + std::to_string(k + 1);
            vipers.push_back(spawn(name.c_str(), PlatformType::F16, { shelter.latitude, shelter.longitude, 0.0, 0, 0, f.runway_heading - 90.0 }, 0.0, true));
        }
        // Diamond: two behind on the sides, the fourth in the slot behind the leader, lower.
        const std::vector<fm::FormationSlot> diamond = {
            { { -50.0, -50.0, 0.0 } }, { { 50.0, -50.0, 0.0 } }, { { 0.0, -100.0, -10.0 } }
        };
        static constexpr ankara::Place in { "flypast start", 39.9400, 32.6600 };
        static constexpr ankara::Place out { "flypast end", 39.9100, 33.0000 };
        {
            fm::TakeoffPlan plan;
            plan.runway = rwy;
            plan.taxi = { field(f, 300.0, 180.0), field(f, -20.0, 180.0), field(f, -20.0, 70.0) };
            fm::Route r;
            r.waypoints = { wp(in, 1550.0), wp(ankara::kAnitkabir, 1500.0), wp(out, 1600.0) };
            r.waypoints[0].speed = 300.0;
            r.waypoints[0].speed_ref = fm::SpeedRef::CalibratedKnots;
            r.waypoints[1].fly_over = true;
            plan.then = r;
            fm::ecs::FlightTakeoff options;
            options.interval = 10.0;
            options.slots = diamond;
            fm::ecs::takeoff_flight(vipers[0], { vipers[1], vipers[2], vipers[3] }, plan, options);
        }

        every_frame("flypast", vipers, [=, over_anitkabir = 1e9, worst_slot_there = 1e9, reported = false, recovering = false, touchdown = std::array<double, 4> { -1.0, -1.0, -1.0, -1.0 }](double t) mutable {
            const double d = distance(vipers[0], ankara::kAnitkabir); // leader's closest pass
            if (d < over_anitkabir)
            {
                over_anitkabir = d;
                worst_slot_there = 0.0;
                for (int k = 1; k < 4; ++k)
                {
                    worst_slot_there = std::max(worst_slot_there, slot_error(vipers[k], vipers[0], diamond[k - 1]));
                }
            }
            else if (!reported && over_anitkabir < 1000.0 && d > over_anitkabir + 500.0)
            {
                reported = true;
                RESIM_LOG(agent_demo, info, "[flypast] t={:.0f}s over Anitkabir: leader {:.0f} m off, worst slot {:.0f} m", t, over_anitkabir, worst_slot_there);
            }
            if (!recovering && !fm::ecs::flight_sequence_active(vipers[0]) && pose(vipers[0]).mode == fm::FlightMode::Cruise)
            {
                fm::ecs::FlightLanding landing;
                landing.interval = 12.0;
                recovering = fm::ecs::land_flight(vipers[0], { vipers[1], vipers[2], vipers[3] }, rwy, landing);
                RESIM_LOG(agent_demo, info, "[flypast] t={:.0f}s flypast done: overhead recovery", t);
            }
            if (!recovering)
            {
                return false;
            }
            for (int k = 0; k < 4; ++k)
            {
                if (touchdown[k] < 0.0 && pose(vipers[k]).on_ground)
                {
                    touchdown[k] = t;
                    RESIM_LOG(agent_demo, info, "[flypast] t={:.0f}s {} touched down", t, vipers[k].name().c_str());
                }
            }
            if (std::all_of(vipers.begin(), vipers.end(), [&](flecs::entity v) { return landed_on(v, rwy, f); }))
            {
                std::array<double, 4> sorted = touchdown;
                std::sort(sorted.begin(), sorted.end());
                double closest_spacing = 1e9;
                for (int k = 1; k < 4; ++k)
                {
                    closest_spacing = std::min(closest_spacing, sorted[k] - sorted[k - 1]);
                }
                RESIM_LOG(agent_demo, info, "[flypast] t={:.0f}s all four landed, touchdowns at least {:.0f} s apart", t, closest_spacing);
                return true;
            }
            return false;
        });
    }

    void traffic()
    {
        struct Inbound
        {
            const char* name;
            PlatformType type;
            double bearing; // deg true, from Kizilay to the start
            double speed;   // m/s
        };
        static constexpr Inbound inbound[] = { { "Hurkus Traffic", PlatformType::Hurkus, 0.0, 110.0 },
                                               { "Anka-3 Traffic", PlatformType::Anka3, 90.0, 150.0 },
                                               { "Anka Traffic", PlatformType::Anka, 180.0, 50.0 },
                                               { "Aksungur Traffic", PlatformType::Aksungur, 270.0, 52.0 } };
        stop_scenario("traffic"); // a previous run restores collision avoidance when it ends
        constexpr double arrival = 150.0; // s, all at the same time
        const fm::Vec3 centre = local(ankara::kKizilay.lat, ankara::kKizilay.lon, 2500.0);
        // (Re)starts the encounter; the same entities both times.
        const auto encounter = [centre](bool avoidance) {
            flecs::world world = sim_world();
            fm::ecs::set_collision_avoidance(world, avoidance);
            std::vector<flecs::entity> aircraft;
            for (const Inbound& in : inbound)
            {
                const fm::Vec2 dir = fm::heading_vector(fm::deg2rad(in.bearing));
                const fm::GeoPoint g = geo(centre + fm::Vec3 { dir.x, dir.y, 0.0 } * (in.speed * arrival));
                flecs::entity e = spawn(in.name, in.type, { g.latitude, g.longitude, 2500.0, 0, 0, in.bearing + 180.0 }, in.speed);
                fm::GoToOptions o;
                o.speed = in.speed;
                fm::ecs::agent(e)->go_to(centre, o); // and straight on
                aircraft.push_back(e);
            }
            RESIM_LOG(agent_demo, info, "[traffic] converging on Kizilay {} collision avoidance", avoidance ? "with" : "without");
            return aircraft;
        };
        const std::vector<flecs::entity> aircraft = encounter(false);
        every_frame(
            "traffic", aircraft, [=, avoidance = false, start = 0.0, closest = 1e9, avoided = false](double t) mutable {
                for (std::size_t a = 0; a < aircraft.size(); ++a)
                {
                    for (std::size_t b = a + 1; b < aircraft.size(); ++b)
                    {
                        closest = std::min(closest, distance(aircraft[a], aircraft[b]));
                    }
                }
                for (flecs::entity e : aircraft)
                {
                    avoided = avoided || fm::ecs::agent(e)->state().avoiding_traffic;
                }
                if (t - start < arrival + 90.0)
                {
                    return false;
                }
                RESIM_LOG(agent_demo, info, "[traffic] closest approach {} collision avoidance: {:.0f} m{}", avoidance ? "with" : "without", closest, avoided ? " (climbed / descended away)" : "");
                if (avoidance)
                {
                    return true;
                }
                encounter(true); // the same encounter again
                avoidance = true;
                start = t;
                closest = 1e9;
                return false;
            },
            [] {
                flecs::world world = sim_world();
                fm::ecs::set_collision_avoidance(world, true); // the default
            });
    }

    void lowlevel()
    {
        const ankara::Airfield& f = ankara::kGuvercinlik;
        flecs::entity atak = spawn("T129", PlatformType::T129, { f.lat, f.lon, 0.0, 0, 0, 110.0 }, 0.0, true);
        static constexpr ankara::Place foothills { "Elmadag foothills", 39.8700, 33.1000 };
        static constexpr ankara::Place ridge { "ridge east of Elmadag", 39.8550, 33.2700 };
        fm::Route r;
        r.waypoints = { wp(ankara::kEymir, 40.0, fm::AltitudeRef::AboveTerrain),
                        wp(foothills, 40.0, fm::AltitudeRef::AboveTerrain),
                        wp(ankara::kElmadag, 60.0, fm::AltitudeRef::AboveTerrain),
                        wp(ridge, 40.0, fm::AltitudeRef::AboveTerrain) };
        r.waypoints[2].hold_time = 60.0; // hover over the summit
        r.on_end = fm::EndBehavior::Land;
        fm::ecs::agent(atak)->follow_route(r);

        every_frame("lowlevel", { atak }, [=, min_agl = 1e9, max_agl = 0.0, hovered = false](double t) mutable {
            const fm::ecs::FlightPose& p = pose(atak);
            const std::size_t leg = fm::ecs::agent(atak)->rotorcraft()->route().index();
            if (p.mode == fm::FlightMode::Route && leg >= 1 && leg <= 2 && distance(atak, ankara::kEymir) > 1000.0)
            { // en route, past the climb
                min_agl = std::min(min_agl, height_above_terrain(atak));
                max_agl = std::max(max_agl, height_above_terrain(atak));
            }
            if (!hovered && distance(atak, ankara::kElmadag) < 30.0 && p.ground_speed < 1.0)
            {
                hovered = true;
                RESIM_LOG(agent_demo, info, "[lowlevel] t={:.0f}s hovering over the summit; {:.0f} to {:.0f} m above the terrain en route", t, min_agl, max_agl);
            }
            if (p.phase == fm::FlightPhase::Landed)
            {
                RESIM_LOG(agent_demo, info, "[lowlevel] t={:.0f}s landed on the ridge, {:.1f} m from the point", t, distance(atak, ridge));
                return true;
            }
            return false;
        });
    }

    void recovery()
    {
        const fm::Runway esb = runway(ankara::kEsenboga);
        const fm::Runway akn = runway(ankara::kAkinci);
        flecs::entity airliner = spawn("A400M", PlatformType::A400M, { ankara::kEsenboga.lat, ankara::kEsenboga.lon, ankara::kEsenboga.elevation + 2500.0, 0, 0, 200.0 }, 150.0);
        flecs::entity aksungur = spawn("Aksungur Recovery", PlatformType::Aksungur, { ankara::kAkinci.lat, ankara::kAkinci.lon, ankara::kAkinci.elevation + 2000.0, 0, 0, 300.0 }, 50.0);
        fm::ecs::agent(airliner)->land(esb);
        fm::ecs::agent(aksungur)->land(akn);

        every_frame("recovery", { airliner, aksungur }, [=, going_around = std::array<bool, 2> { false, false }, landed = std::array<bool, 2> { false, false }](double t) mutable {
            const flecs::entity aircraft[] = { airliner, aksungur };
            const bool on_runway[] = { landed_on(airliner, esb, ankara::kEsenboga), landed_on(aksungur, akn, ankara::kAkinci) };
            const char* fields[] = { "Esenboga", "Akinci" };
            for (int k = 0; k < 2; ++k)
            {
                const bool go_around = pose(aircraft[k]).phase == fm::FlightPhase::GoAround;
                if (go_around && !going_around[k])
                {
                    RESIM_LOG(agent_demo, info, "[recovery] t={:.0f}s {} goes around", t, aircraft[k].name().c_str());
                }
                going_around[k] = go_around;
                if (on_runway[k] && !landed[k])
                {
                    landed[k] = true;
                    RESIM_LOG(agent_demo, info, "[recovery] t={:.0f}s {} landed at {}", t, aircraft[k].name().c_str(), fields[k]);
                }
            }
            return landed[0] && landed[1];
        });
    }

    void relief()
    {
        const ankara::Airfield& f = ankara::kAkinci;
        const fm::Runway rwy = runway(f);
        static constexpr ankara::Place station { "station west of Ankara", 39.8800, 32.4000 };
        flecs::entity anka = spawn("Anka Station", PlatformType::Anka, { station.lat, station.lon + 0.03, 2400.0, 0, 0, 0.0 }, 46.0);
        fm::ecs::agent(anka)->loiter(local(station.lat, station.lon, 2400.0), 2500.0);
        show_waypoints(anka, { { station.lat, station.lon, 2400.0 } });
        const fm::GeoPoint start = geo(rwy.start);
        flecs::entity aksungur = spawn("Aksungur Relief", PlatformType::Aksungur, { start.latitude, start.longitude, 0.0, 0, 0, f.runway_heading }, 0.0, true);
        {
            fm::TakeoffPlan plan;
            plan.runway = rwy;
            fm::Route r; // to the station, orbiting there above the Anka
            r.waypoints = { wp(station, 2700.0) };
            r.on_end = fm::EndBehavior::Loiter;
            r.loiter_radius = 2500.0;
            plan.then = r;
            fm::ecs::agent(aksungur)->takeoff(plan);
        }

        every_frame("relief", { anka, aksungur }, [=, last = 0.0, both_on_station = 0.0, closest = 1e9, relieved = false](double t) mutable {
            const double dt = t - last;
            last = t;
            const bool anka_on = pose(anka).mode == fm::FlightMode::Loiter;
            const bool aksungur_on = pose(aksungur).mode == fm::FlightMode::Loiter && distance(aksungur, station) < 3000.0;
            if (anka_on && aksungur_on)
            {
                both_on_station += dt; // s with both orbiting the station
            }
            if (!pose(aksungur).on_ground)
            {
                closest = std::min(closest, distance(anka, aksungur));
            }
            // Relieved after a full lap together: the Anka goes home.
            if (!relieved && both_on_station > 240.0)
            {
                relieved = fm::ecs::agent(anka)->land(rwy);
                RESIM_LOG(agent_demo, info, "[relief] t={:.0f}s Aksungur on station ({:.0f} m apart at the closest): Anka returns to Akinci", t, closest);
            }
            if (relieved && landed_on(anka, rwy, f))
            {
                RESIM_LOG(agent_demo, info, "[relief] t={:.0f}s Anka landed at Akinci", t);
                return true;
            }
            return false;
        });
    }

    void escort()
    {
        flecs::world world = sim_world();
        const double lat0 = 39.9000, lon0 = 32.7600, yaw = 245.0; // west along the road
        const double speed = 15.0;                                 // m/s
        const double drive_time = 1500.0;                          // s, then the convoy stops
        const resim::core::Transform start { lat0, lon0, ankara::terrain(lat0, lon0), 0, 0, yaw };
        // TODO: the game's ground vehicle prefab, e.g. e.is_a(resim::core::get_core_prefab(...))
        flecs::entity convoy = world.entity("Convoy")
                                   .set<resim::core::Transform>(start)
                                   .set<fm::ecs::TrackedGeoPose>({ { start.latitude_deg, start.longitude_deg, start.alt_m }, start.yaw_deg });
        flecs::entity behind = spawn("T129 Escort-1", PlatformType::T129, { 39.94, 32.74, 1300.0, 0, 0, 200.0 }, 40.0);
        flecs::entity beside = spawn("T129 Escort-2", PlatformType::T129, { 39.92, 32.79, 1200.0, 0, 0, 245.0 }, 40.0);
        const fm::FormationSlot behind_slot { { 0.0, -300.0, 150.0 } };
        const fm::FormationSlot beside_slot { { 80.0, -20.0, 60.0 } };
        fm::ecs::follow(behind, convoy, behind_slot);
        fm::ecs::follow(beside, convoy, beside_slot);

        every_frame("escort", { convoy, behind, beside }, [=, last = 0.0, worst = std::array<double, 2> { 0.0, 0.0 }](double t) mutable {
            const double dt = t - last;
            last = t;
            // Game logic: the convoy drives on along its yaw, on the ground, and publishes its
            // position for its escorts.
            resim::core::Transform c = transform(convoy);
            const double d = speed * dt;
            const double heading = fm::deg2rad(c.yaw_deg);
            c.latitude_deg += d * std::cos(heading) / 111132.0;
            c.longitude_deg += d * std::sin(heading) / (111320.0 * std::cos(fm::deg2rad(c.latitude_deg)));
            c.alt_m = ankara::terrain(c.latitude_deg, c.longitude_deg);
            convoy.set<resim::core::Transform>(c);
            convoy.set<fm::ecs::TrackedGeoPose>({ { c.latitude_deg, c.longitude_deg, c.alt_m }, c.yaw_deg });
            if (t > 240.0)
            { // joined up
                worst[0] = std::max(worst[0], slot_error(behind, convoy, behind_slot));
                worst[1] = std::max(worst[1], slot_error(beside, convoy, beside_slot));
            }
            if (t >= drive_time)
            {
                RESIM_LOG(agent_demo, info, "[escort] t={:.0f}s the convoy stops after {:.1f} km; worst slot error {:.0f} m behind, {:.0f} m beside", t, ankara::metres(c.latitude_deg, c.longitude_deg, lat0, lon0) / 1000.0, worst[0], worst[1]);
                return true; // the escorts stay over the stopped convoy
            }
            return false;
        });
    }

    bool run_scenario(const char* name)
    {
        for (const Scenario& s : demo_scenarios)
        {
            if (std::strcmp(s.name, name) == 0)
            {
                stop_scenario(s.name);
                s.run();
                return true;
            }
        }
        return false;
    }

} // namespace demo
