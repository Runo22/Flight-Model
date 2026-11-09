#include "helpers.hpp"
///////////////////////////////////////////////////////////////////////////////
// Standart Headers
#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <utility>
///////////////////////////////////////////////////////////////////////////////
// Resim Headers
#include <resim/core/components.hpp>
#include <resim/core/core_functions.hpp>
#include <resim/core/platform_data.hpp>
#include <resim/core/resim_world.hpp>
///////////////////////////////////////////////////////////////////////////////

namespace ankara
{
    double metres(double lat1, double lon1, double lat2, double lon2)
    {
        const double kDeg = 3.14159265358979 / 180.0;
        const double dn = (lat2 - lat1) * 111132.0;
        const double de = (lon2 - lon1) * 111320.0 * std::cos(0.5 * (lat1 + lat2) * kDeg);
        return std::sqrt(dn * dn + de * de);
    }

    double terrain(double lat, double lon)
    {
        const auto hill = [&](double hlat, double hlon, double height, double radius) {
            const double d = metres(lat, lon, hlat, hlon) / radius;
            return height * std::exp(-d * d);
        };
        double z = 900.0 + 40.0 * std::sin((lat - 39.9) * 25.0) * std::cos((lon - 32.8) * 20.0);
        z += hill(kElmadag.lat, kElmadag.lon, 950.0, 7000.0);
        z += hill(39.9600, 32.9700, 420.0, 3000.0);  // Huseyingazi
        z += hill(40.3200, 32.7500, 500.0, 14000.0); // hills towards Kizilcahamam
        z += hill(39.7600, 32.8000, 90.0, 12000.0);  // Golbasi plateau
        for (const Airfield& f : kAirfields)
        {
            const double d = metres(lat, lon, f.lat, f.lon);
            const double w = std::clamp((d - 2500.0) / 2000.0, 0.0, 1.0);
            z = f.elevation + (z - f.elevation) * w * w * (3.0 - 2.0 * w);
        }
        return z;
    }

} // namespace ankara

namespace
{
    // --- Binding ---------------------------------------------------------------------------

    struct Binding
    {
        ecs_world_t* world = nullptr; // not owned
        fm::GeoProjection projection;
        DemoSetup setup;
    };
    Binding bound;

    void ensure_bound()
    {
        if (!bound.world)
        {
            flecs::world world = resim::core::get_world();
            bind_world(world);
        }
    }

    // --- Scenario logic --------------------------------------------------------------------

    struct Script
    {
        std::string name;
        std::vector<flecs::entity> actors;
        ScenarioStep step;
        std::function<void()> on_stop;
        double t = 0.0;
        bool done = false;
    };
    std::vector<std::unique_ptr<Script>> scripts;
    bool running_scripts = false; // stop_scenarios() / every_frame() called from a step

    void finish(Script& s)
    {
        if (s.done)
        {
            return;
        }
        s.done = true;
        if (s.on_stop)
        {
            s.on_stop();
        }
    }

    void run_scripts(flecs::world& world, double dt)
    {
        if (fm::ecs::is_paused(world))
        {
            return;
        }
        if (const auto* settings = world.try_get<fm::ecs::FlightSettings>())
        {
            dt = std::min(dt, settings->max_step); // same clock as the flight model
        }
        running_scripts = true;
        // By index: a step may start another scenario (push_back).
        for (std::size_t i = 0; i < scripts.size(); ++i)
        {
            Script& s = *scripts[i];
            if (s.done)
            {
                continue;
            }
            const bool deleted = std::any_of(s.actors.begin(), s.actors.end(), [](flecs::entity e) { return !e.is_alive(); });
            if (deleted)
            {
                finish(s);
                continue;
            }
            s.t += dt;
            if (s.step(s.t))
            {
                finish(s);
            }
        }
        running_scripts = false;
        std::erase_if(scripts, [](const std::unique_ptr<Script>& s) { return s->done; });
    }

    // --- Route display ---------------------------------------------------------------------

    // What the route display last wrote to the entity's WaypointList.
    struct RouteShown
    {
        std::size_t signature = 0;
        std::size_t index = 0;
        bool manual = false; // show_waypoints(): left alone until the agent flies a new route
    };

    const fm::RouteTracker* route_of(const fm::FlightAgent& a)
    {
        if (const auto* m = a.fixed_wing())
        {
            return &m->route();
        }
        if (const auto* m = a.rotorcraft())
        {
            return &m->route();
        }
        if (const auto* m = a.ship())
        {
            return &m->route();
        }
        return nullptr;
    }

    // Changes when the agent starts another route.
    std::size_t signature(const fm::Route& route)
    {
        const std::vector<fm::Waypoint>& waypoints = route.waypoints;
        std::size_t h = 2 * waypoints.size() + (route.loop ? 1 : 0) + 1;
        // clang-format off
        const auto mix = [&h](double v) { h ^= std::hash<double> {}(v) + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2); };
        // clang-format on
        for (const fm::Waypoint& w : waypoints)
        {
            mix(w.position.x);
            mix(w.position.y);
            mix(w.position.z);
        }
        return h;
    }

    resim::core::Waypoint game_waypoint(const fm::GeoPoint& p)
    {
        resim::core::Waypoint w;
        w.latitude_deg = p.latitude;
        w.longitude_deg = p.longitude;
        w.altitude_m = p.altitude;
        return w;
    }

    void show_route(flecs::entity e, const fm::RouteTracker& tracker, const RouteShown* shown)
    {
        const fm::Route& route = tracker.route();
        const std::vector<fm::Waypoint>& waypoints = route.waypoints;
        const std::size_t sig = signature(route);
        const std::size_t index = tracker.index();
        if (!shown || shown->signature != sig)
        {
            // A new route: converted once, then only the index follows.
            resim::core::WaypointList list;
            list.waypoints.reserve(waypoints.size());
            for (const fm::Waypoint& w : waypoints)
            {
                list.waypoints.push_back(game_waypoint(bound.projection.to_geo(w.position)));
            }
            list.current_index = index;
            list.loop = route.loop;
            e.set<resim::core::WaypointList>(std::move(list));
            e.set<RouteShown>({ sig, index, false });
            return;
        }
        if (shown->manual || shown->index == index)
        {
            return;
        }
        if (auto* list = e.try_get_mut<resim::core::WaypointList>())
        {
            list->current_index = index;
        }
        e.set<RouteShown>({ sig, index, false });
    }

    // --- Vehicles --------------------------------------------------------------------------

    void set_custom_starting_location(flecs::entity e, const resim::core::Transform& transform)
    {
        if (e.is_valid() && e.has<resim::core::StartingLocation>() && e.has<resim::core::Transform>())
        {
            auto& location = e.get_mut<resim::core::StartingLocation>();
            location.name = "Custom Location";
            location.calculated_latitude = transform.latitude_deg;
            location.calculated_longitude = transform.longitude_deg;
            location.origin_hot = transform.alt_m;
            location.runway_psi = transform.yaw_deg;
        }
    }

    fm::VehicleParams get_vehicle_params(resim::core::PlatformType type)
    {
        using namespace resim::core;

        switch (type)
        {
        case PlatformType::Hurjet:
        case PlatformType::HurjetAmmunition:
            return fm::presets::jet_trainer();
        case PlatformType::Kaan:
        case PlatformType::F15:
        case PlatformType::F18:
        case PlatformType::F22:
            return fm::presets::twin_engine_fighter();
        case PlatformType::F16:
            return fm::presets::single_engine_fighter();
        case PlatformType::Hurkus:
            return fm::presets::turboprop_trainer();
        case PlatformType::Anka:
        case PlatformType::TB2:
        case PlatformType::TB3:
            return fm::presets::propeller_uav();
        case PlatformType::Aksungur:
        case PlatformType::Akinci:
            return fm::presets::twin_propeller_uav();
        case PlatformType::Anka3:
        case PlatformType::KizilElma:
            return fm::presets::jet_uav();
        case PlatformType::T129:
            return fm::presets::attack_helicopter();
        case PlatformType::Hsoj: // Global6000:
        case PlatformType::E7T:
        case PlatformType::A400M:
        case PlatformType::C130:
        case PlatformType::KC135R:
            return fm::presets::airliner_jet();
        case PlatformType::TCGAnadolu:
        case PlatformType::Mugem:
            return fm::presets::carrier_ship();
        default:
            break; // by category
        };

        switch (get_platform_category(type))
        {
        case PlatformCategory::Fighter:
            return fm::presets::fast_jet();
        case PlatformCategory::Propeller:
            return fm::presets::turboprop_trainer();
        case PlatformCategory::UAV:
            return fm::presets::jet_uav();
        case PlatformCategory::Helicopter:
            return fm::presets::attack_helicopter();
        case PlatformCategory::Support:
            return fm::presets::airliner_jet();
        case PlatformCategory::Ship:
            return fm::presets::carrier_ship();
        case PlatformCategory::Munnition:
            return fm::presets::fast_jet(); // TODO: NEEDS TPYE IMPLEMENTATION
        case PlatformCategory::Unknown:
        default:
            return fm::presets::multirotor(); // Drone as fallback. Too slow.
        }
    }

    namespace game
    {
        double query_hot(double /*lat*/, double /*lon*/)
        {
            // TODO: implemet read data from binary data file
            return 835.0;
        }
    } // namespace game

} // namespace

///////////////////////////////////////////////////////////////////////////////
// World

void bind_world(flecs::world& world, const DemoSetup& setup)
{
    if (bound.world && bound.world != world.c_ptr())
    {
        unbind_world();
    }
    flecs::entity module = fm::ecs::module_entity(world);
    if (!module)
    {
        module = fm::ecs::load(world);
    }
    if (!fm::ecs::geo(world))
    {
        fm::ecs::set_geo_origin(world, setup.origin.latitude, setup.origin.longitude);
    }
    bound.world = world.c_ptr();
    bound.projection = *fm::ecs::geo(world);
    bound.setup = setup;

    // In the module scope: fm::ecs::unload() deletes them with the module.
    const flecs::entity previous_scope = world.set_scope(module);
    world.component<RouteShown>();
    if (!module.lookup("DemoScenarios"))
    {
        // After PostUpdate: the flight model has stepped and the transforms are written.
        world.system("DemoScenarios")
            .kind(flecs::PreStore)
            .immediate()
            .run([](flecs::iter& it) {
                flecs::world w = it.world();
                run_scripts(w, it.delta_time());
            });
        world.system<const fm::ecs::FlightController, const RouteShown*>("DemoRouteDisplay")
            .kind(flecs::PostUpdate)
            .each([](flecs::entity e, const fm::ecs::FlightController& fc, const RouteShown* shown) {
                const fm::RouteTracker* tracker = fc.agent ? route_of(*fc.agent) : nullptr;
                if (tracker && tracker->active() && !tracker->route().waypoints.empty())
                {
                    show_route(e, *tracker, shown);
                }
            });
        world.system<const resim::core::Transform, fm::ecs::TerrainHeight>("DemoTerrain")
            .kind(flecs::PreUpdate)
            .with<fm::ecs::FlightController>()
            .each([](const resim::core::Transform& t, fm::ecs::TerrainHeight& height) {
                height.value = ankara::terrain(t.latitude_deg, t.longitude_deg);
            });
    }
    // enable() / disable() without an argument: enable(x) would toggle component x.
    const auto enable = [](flecs::entity system, bool on) { on ? system.enable() : system.disable(); };
    enable(module.lookup("DemoRouteDisplay"), setup.show_routes);
    enable(module.lookup("DemoTerrain"), setup.synthetic_terrain);
    world.set_scope(previous_scope);
}

void unbind_world()
{
    stop_scenarios();
    if (!running_scripts)
    {
        scripts.clear(); // their code lives in this binary
    }
    bound = {};
    // This binary may stay loaded and bind a new world: forget the cached component id.
    flecs::_::type<RouteShown>::reset();
}

flecs::world sim_world()
{
    ensure_bound();
    return flecs::world(bound.world);
}

const fm::GeoProjection& projection()
{
    ensure_bound();
    return bound.projection;
}

///////////////////////////////////////////////////////////////////////////////
// Scenario logic

void every_frame(const char* name, std::vector<flecs::entity> actors, ScenarioStep step, std::function<void()> on_stop)
{
    ensure_bound();
    stop_scenario(name); // started again: the old run ends
    auto s = std::make_unique<Script>();
    s->name = name;
    s->actors = std::move(actors);
    s->step = std::move(step);
    s->on_stop = std::move(on_stop);
    scripts.push_back(std::move(s));
}

bool scenario_running(const char* name)
{
    return std::any_of(scripts.begin(), scripts.end(), [name](const std::unique_ptr<Script>& s) { return !s->done && s->name == name; });
}

void stop_scenario(const char* name)
{
    for (const std::unique_ptr<Script>& s : scripts)
    {
        if (s->name == name)
        {
            finish(*s);
        }
    }
    if (!running_scripts)
    {
        std::erase_if(scripts, [](const std::unique_ptr<Script>& s) { return s->done; });
    }
}

void stop_scenarios()
{
    for (const std::unique_ptr<Script>& s : scripts)
    {
        finish(*s);
    }
    if (!running_scripts)
    {
        scripts.clear();
    }
}

///////////////////////////////////////////////////////////////////////////////
// Entities

flecs::entity spawn(const char* name, resim::core::PlatformType platform_type, resim::core::Transform t, double speed, bool on_ground, const fm::ecs::AgentSetup& setup)
{
    flecs::world world = sim_world();
    // TODO: use real hot data from the HOT plugin in the future
    const double ground = ankara::terrain(t.latitude_deg, t.longitude_deg);
    if (on_ground)
    {
        t.alt_m = ground;
    }

    flecs::entity e = world.entity(name);
    // Reused (the scenario runs again): no formation, flight sequence or route display left
    // from the last run, or they would keep commanding the new agent.
    e.remove<fm::ecs::FollowLeader>();
    e.remove<fm::ecs::FlightSequence>();
    e.remove<resim::core::WaypointList>();
    e.remove<RouteShown>();
    e.is_a(resim::core::get_core_prefab(platform_type));
    e.set<resim::core::Transform>(t);
    e.set<resim::core::HotRequest>({ game::query_hot(t.latitude_deg, t.longitude_deg) });
    e.set<fm::ecs::TerrainHeight>({ ground });

    fm::ecs::attach_geo(e, get_vehicle_params(platform_type), { t.latitude_deg, t.longitude_deg, t.alt_m }, t.yaw_deg, speed, on_ground, setup);

    set_custom_starting_location(e, t); // to prevent move to akinci on start
    return e;
}

const resim::core::Transform& transform(flecs::entity e)
{
    return *e.try_get<resim::core::Transform>();
}

const fm::ecs::FlightPose& pose(flecs::entity e)
{
    return *e.try_get<fm::ecs::FlightPose>();
}

void show_waypoints(flecs::entity e, std::initializer_list<fm::GeoPoint> points)
{
    ensure_bound(); // RouteShown registered in the module scope
    resim::core::WaypointList list;
    list.waypoints.reserve(points.size());
    for (const fm::GeoPoint& p : points)
    {
        list.waypoints.push_back(game_waypoint(p));
    }
    list.current_index = 0;
    list.loop = false;
    e.set<resim::core::WaypointList>(std::move(list));
    // Kept until the agent flies a route other than the current one.
    RouteShown shown;
    shown.manual = true;
    if (const fm::FlightAgent* a = fm::ecs::agent(e))
    {
        if (const fm::RouteTracker* tracker = route_of(*a); tracker && tracker->active())
        {
            shown.signature = signature(tracker->route());
        }
    }
    e.set<RouteShown>(shown);
}

///////////////////////////////////////////////////////////////////////////////
// Geodetic <-> local frame

fm::Vec3 local(double lat, double lon, double alt)
{
    return projection().to_local({ lat, lon, alt });
}

fm::Vec3 local(const fm::GeoPoint& p)
{
    return projection().to_local(p);
}

fm::GeoPoint geo(const fm::Vec3& p)
{
    return projection().to_geo(p);
}

double local_heading(double true_heading_deg, const fm::GeoPoint& near)
{
    const fm::GeoProjection& proj = projection();
    return proj.grid_heading(fm::deg2rad(true_heading_deg), proj.to_local(near));
}

fm::Waypoint wp(double lat, double lon, double alt, fm::AltitudeRef ref)
{
    fm::Waypoint w;
    w.position = local(lat, lon, alt);
    w.altitude_ref = ref;
    return w;
}

fm::Waypoint wp(const ankara::Place& p, double alt, fm::AltitudeRef ref)
{
    return wp(p.lat, p.lon, alt, ref);
}

fm::Runway runway(const ankara::Airfield& f)
{
    const fm::GeoPoint arp { f.lat, f.lon, f.elevation };
    const double heading = local_heading(f.runway_heading, arp);
    const fm::Vec3 mid = local(arp);
    const fm::Vec2 start = mid.xy() - fm::heading_vector(heading) * (0.5 * f.runway_length);
    return { { start.x, start.y, f.elevation }, heading };
}

fm::Runway runway(const fm::GeoPoint& threshold, double true_heading_deg)
{
    return { local(threshold), local_heading(true_heading_deg, threshold) };
}

fm::Vec3 field(const ankara::Airfield& f, double along, double right)
{
    const fm::Runway r = runway(f);
    const fm::Vec2 dir = fm::heading_vector(r.heading);
    const fm::Vec2 p = r.start.xy() + dir * along + fm::right_of(dir) * right;
    return { p.x, p.y, f.elevation };
}

///////////////////////////////////////////////////////////////////////////////
// Measurements

fm::Vec3 position(flecs::entity e)
{
    if (const fm::FlightAgent* a = fm::ecs::agent(e))
    {
        return a->state().position;
    }
    const resim::core::Transform& t = transform(e);
    return local(t.latitude_deg, t.longitude_deg, t.alt_m);
}

double distance(flecs::entity e, double lat, double lon)
{
    return (position(e) - local(lat, lon, 0.0)).xy().length();
}

double distance(flecs::entity e, const ankara::Place& p)
{
    return distance(e, p.lat, p.lon);
}

double distance(flecs::entity a, flecs::entity b)
{
    return (position(a) - position(b)).length();
}

double height_above_terrain(flecs::entity e)
{
    const resim::core::Transform& t = transform(e);
    return t.alt_m - ankara::terrain(t.latitude_deg, t.longitude_deg);
}

fm::Vec2 on_runway(flecs::entity e, const fm::Runway& r)
{
    const fm::Vec2 d = position(e).xy() - r.start.xy();
    const fm::Vec2 dir = fm::heading_vector(r.heading);
    return { dot(d, dir), dot(d, fm::right_of(dir)) };
}

bool landed_on(flecs::entity e, const fm::Runway& r, const ankara::Airfield& f)
{
    const fm::Vec2 at = on_runway(e, r);
    return pose(e).phase == fm::FlightPhase::Landed && at.x > 0.0 && at.x < f.runway_length && std::abs(at.y) < 20.0;
}

double slot_error(flecs::entity follower, flecs::entity leader, const fm::FormationSlot& slot)
{
    const fm::Vec3 l = position(leader);
    double heading = 0.0;
    if (const fm::FlightAgent* a = fm::ecs::agent(leader))
    {
        heading = a->state().attitude.heading;
    }
    else
    {
        const resim::core::Transform& t = transform(leader);
        heading = local_heading(t.yaw_deg, { t.latitude_deg, t.longitude_deg, t.alt_m });
    }
    const fm::Vec2 fwd = fm::heading_vector(heading);
    const fm::Vec2 at = l.xy() + fm::right_of(fwd) * slot.offset.x + fwd * slot.offset.y;
    return (fm::Vec3 { at.x, at.y, l.z + slot.offset.z } - position(follower)).length();
}
