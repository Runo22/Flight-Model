// Flight demos around Ankara in a geodetic Flecs world:
//   rejoin     scattered Hurjet + two Kizilelma form up, fly to Mogan lake as a formation,
//              then the leader and one UAV land at Akinci in sequence, the other orbits
//   circuit    Hurkus: apron, taxi, takeoff, a short tour, overhead break, taxi back
//   waypoints  Anka tour: fly-by and flown-over corners, a speed in knots, terrain
//              following over Elmadag, landing back
//   flypast    four F-16s: sequenced takeoff, diamond over Anitkabir, overhead recovery
//   traffic    four aircraft converging over Kizilay at the same altitude
//   lowlevel   T129 from Guvercinlik, terrain following to Elmadag, hover, landing on a ridge
//   recovery   landings ordered from high above the field (airliner at Esenboga, Aksungur
//              at Akinci): descending orbit, straight-in
//   relief     station relief: an Aksungur takes over an Anka's orbit, the Anka lands
//   escort     a T129 and a drone escort a convoy driven by game logic
//
// The terrain is a synthetic stand-in (the Ankara basin at ~900 m, hills around it,
// Elmadag), not real elevation data; airfield and landmark positions, elevations and runway
// headings are approximate.
//
// Usage: fm_ankara_demos [demo...] [--csv trace.csv] [--kml trace.kml] [--list]
// Without a demo name all demos run. Every demo checks its own results; each is a test
// (fm_ankara_<demo>). The traces can be replayed in a game or opened in Google Earth.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "fm/ecs/flight_module.hpp"
#include "fm/presets.hpp"

namespace game {

struct Transform {
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    double alt_m = 0.0;
    double roll_deg = 0.0;
    double pitch_deg = 0.0;
    double yaw_deg = 0.0;  // true heading
};

struct Hot {
    double height = 0.0;  // m, terrain height under the entity
};

}  // namespace game

namespace ankara {

struct Place {
    const char* name;
    double lat;  // deg
    double lon;  // deg
};

struct Airfield {
    const char* name;
    double lat, lon;        // deg, reference point
    double elevation;       // m
    double runway_heading;  // deg true, the runway modelled (the reference point mid-runway)
    double runway_length;   // m
};

const Airfield kAkinci{"Akinci (Murted) AB", 40.0789, 32.5656, 857.0, 34.0, 3000.0};
const Airfield kEsenboga{"Esenboga", 40.1281, 32.9951, 953.0, 34.0, 3750.0};
const Airfield kGuvercinlik{"Guvercinlik", 39.9351, 32.7408, 820.0, 110.0, 1500.0};
const Airfield kAirfields[] = {kAkinci, kEsenboga, kGuvercinlik};

const Place kAnitkabir{"Anitkabir", 39.9250, 32.8369};
const Place kKizilay{"Kizilay", 39.9208, 32.8541};
const Place kMogan{"Mogan lake", 39.7800, 32.8000};
const Place kEymir{"Eymir lake", 39.8250, 32.8300};
const Place kElmadag{"Elmadag", 39.8600, 33.2300};
const Place kKahramankazan{"Kahramankazan", 40.2050, 32.6800};
const Place kEtimesgut{"Etimesgut", 39.9500, 32.6900};
const Place kPlaces[] = {kAnitkabir, kKizilay, kMogan, kEymir, kElmadag, kKahramankazan, kEtimesgut};

// Horizontal distance in meters (equirectangular, fine over tens of kilometers).
double metres(double lat1, double lon1, double lat2, double lon2) {
    const double kDeg = 3.14159265358979 / 180.0;
    const double dn = (lat2 - lat1) * 111132.0;
    const double de = (lon2 - lon1) * 111320.0 * std::cos(0.5 * (lat1 + lat2) * kDeg);
    return std::sqrt(dn * dn + de * de);
}

// Synthetic terrain: a gently rolling basin at ~900 m, hills around the city, Elmadag
// (~1,850 m) to the east-south-east, flat airfields.
double terrain(double lat, double lon) {
    const auto hill = [&](double hlat, double hlon, double height, double radius) {
        const double d = metres(lat, lon, hlat, hlon) / radius;
        return height * std::exp(-d * d);
    };
    double z = 900.0 + 40.0 * std::sin((lat - 39.9) * 25.0) * std::cos((lon - 32.8) * 20.0);
    z += hill(kElmadag.lat, kElmadag.lon, 950.0, 7000.0);
    z += hill(39.9600, 32.9700, 420.0, 3000.0);     // Huseyingazi
    z += hill(40.3200, 32.7500, 500.0, 14000.0);    // hills towards Kizilcahamam
    z += hill(39.7600, 32.8000, 90.0, 12000.0);     // Golbasi plateau
    for (const Airfield& f : kAirfields) {
        const double d = metres(lat, lon, f.lat, f.lon);
        const double w = std::clamp((d - 2500.0) / 2000.0, 0.0, 1.0);
        z = f.elevation + (z - f.elevation) * w * w * (3.0 - 2.0 * w);
    }
    return z;
}

}  // namespace ankara

namespace {

// --- Results -------------------------------------------------------------------------------

int failures = 0;

void check(bool ok, const char* what) {
    std::printf("  [%s] %s\n", ok ? " OK " : "FAIL", what);
    if (!ok) ++failures;
}

// --- Trace: CSV rows and a Google Earth track per aircraft, sampled every second ----------

class Trace {
public:
    bool open_csv(const char* path) {
        csv_ = std::fopen(path, "w");
        if (csv_)
            std::fprintf(csv_, "demo,t,name,lat,lon,alt,roll,pitch,yaw,kcas,mode,phase,gear\n");
        return csv_ != nullptr;
    }
    void enable_kml() { kml_ = true; }
    bool enabled() const { return csv_ || kml_; }

    // In the KML the demos play one after the other: each starts a minute after the last
    // sample of the one before.
    void begin(const char* demo) {
        demo_ = demo;
        if (!demos_.empty()) offset_ = end_ + 60.0;
        demos_.push_back({demo, {}});
    }

    void sample(flecs::world& world, double t) {
        if (!enabled()) return;
        world.each([&](flecs::entity e, const game::Transform& tr, const fm::ecs::FlightPose& p) {
            const std::string name = e.name().c_str();
            if (csv_) {
                std::fprintf(csv_, "%s,%.0f,%s,%.6f,%.6f,%.1f,%.1f,%.1f,%.1f,%.0f,%s,%s,%.2f\n",
                             demo_.c_str(), t, name.c_str(), tr.latitude_deg, tr.longitude_deg, tr.alt_m, tr.roll_deg,
                             tr.pitch_deg, tr.yaw_deg, p.calibrated_airspeed / 0.514444,
                             fm::to_string(p.mode), fm::to_string(p.phase), p.gear_position);
            }
            if (kml_) demos_.back().tracks[name].push_back({offset_ + t, tr.latitude_deg, tr.longitude_deg, tr.alt_m});
        });
        end_ = std::max(end_, offset_ + t);
    }

    bool write_kml(const char* path) const {
        std::FILE* f = std::fopen(path, "w");
        if (!f) return false;
        std::fprintf(f, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
                        "<kml xmlns=\"http://www.opengis.net/kml/2.2\" "
                        "xmlns:gx=\"http://www.google.com/kml/ext/2.2\">\n"
                        "<Document><name>Flight model demos around Ankara</name>\n");
        std::fprintf(f, "<Folder><name>Places (approximate)</name>\n");
        for (const ankara::Airfield& a : ankara::kAirfields)
            place(f, a.name, a.lat, a.lon);
        for (const ankara::Place& p : ankara::kPlaces) place(f, p.name, p.lat, p.lon);
        std::fprintf(f, "</Folder>\n");
        const char* colors[] = {"ff0000ff", "ff00ffff", "ff00ff00", "ffff0000", "ffff00ff",
                                "ff00a5ff", "ffffffff"};
        for (const DemoTracks& d : demos_) {
            std::fprintf(f, "<Folder><name>%s</name>\n", d.name.c_str());
            int k = 0;
            for (const auto& [name, points] : d.tracks) {
                std::fprintf(f, "<Placemark><name>%s</name><Style><LineStyle><color>%s</color>"
                                "<width>2</width></LineStyle></Style>\n"
                                "<gx:Track><altitudeMode>absolute</altitudeMode>\n",
                             name.c_str(), colors[k++ % 7]);
                for (const Point& p : points) {  // 29 October from 08:00 UTC
                    const long s = 8 * 3600 + static_cast<long>(p.t);
                    std::fprintf(f, "<when>2026-10-29T%02ld:%02ld:%02ldZ</when>\n", s / 3600,
                                 s / 60 % 60, s % 60);
                }
                for (const Point& p : points)
                    std::fprintf(f, "<gx:coord>%.6f %.6f %.1f</gx:coord>\n", p.lon, p.lat, p.alt);
                std::fprintf(f, "</gx:Track></Placemark>\n");
            }
            std::fprintf(f, "</Folder>\n");
        }
        std::fprintf(f, "</Document></kml>\n");
        std::fclose(f);
        return true;
    }

    ~Trace() {
        if (csv_) std::fclose(csv_);
    }

private:
    struct Point {
        double t, lat, lon, alt;
    };
    struct DemoTracks {
        std::string name;
        std::map<std::string, std::vector<Point>> tracks;
    };
    static void place(std::FILE* f, const char* name, double lat, double lon) {
        std::fprintf(f, "<Placemark><name>%s</name><Point><coordinates>%.6f,%.6f,0</coordinates>"
                        "</Point></Placemark>\n",
                     name, lon, lat);
    }
    std::FILE* csv_ = nullptr;
    bool kml_ = false;
    std::string demo_;
    double offset_ = 0.0;  // s, KML time of this demo's start
    double end_ = 0.0;     // s, KML time of the last sample
    std::vector<DemoTracks> demos_;
};

// --- World and helpers ---------------------------------------------------------------------

flecs::world make_world() {
    flecs::world world;
    fm::ecs::load(world);
    fm::ecs::set_geo_origin(world, 39.93, 32.80);  // Ankara
    world.system<const game::Transform, game::Hot>("game::HotSystem")
        .kind(flecs::PreUpdate)
        .each([](const game::Transform& t, game::Hot& hot) { hot.height = ankara::terrain(t.latitude_deg, t.longitude_deg); });
    world.system<const game::Hot, fm::ecs::TerrainHeight>("TerrainIn")
        .kind(flecs::PreUpdate)
        .each([](const game::Hot& hot, fm::ecs::TerrainHeight& t) { t.value = hot.height; });
    fm::ecs::bind_geo_transform<game::Transform>(world);
    return world;
}

// An aircraft at a latitude/longitude; on the ground `alt` is ignored.
flecs::entity spawn(flecs::world& world, const char* name, const fm::VehicleParams& params,
                    game::Transform t, double speed, bool on_ground) {
    if (on_ground) t.alt_m = ankara::terrain(t.latitude_deg, t.longitude_deg);
    const double hot = ankara::terrain(t.latitude_deg, t.longitude_deg);
    flecs::entity e = world.entity(name)
                          .set<game::Transform>(t)
                          .set<game::Hot>({hot})
                          .set<fm::ecs::TerrainHeight>({hot});
    fm::ecs::attach_geo(e, params, {t.latitude_deg, t.longitude_deg, t.alt_m}, t.yaw_deg, speed, on_ground);
    return e;
}

fm::Vec3 local(flecs::world& world, double lat, double lon, double alt) {
    return fm::ecs::to_local(world, {lat, lon, alt});
}

fm::Waypoint wp(flecs::world& world, const ankara::Place& p, double alt,
                fm::AltitudeRef ref = fm::AltitudeRef::Absolute) {
    fm::Waypoint w;
    w.position = local(world, p.lat, p.lon, alt);
    w.altitude_ref = ref;
    return w;
}

// The modelled runway: threshold half a runway length before the reference point.
fm::Runway runway(flecs::world& world, const ankara::Airfield& f) {
    const fm::GeoPoint arp{f.lat, f.lon, f.elevation};
    const double heading = fm::ecs::local_heading(world, f.runway_heading, arp);
    const fm::Vec3 mid = fm::ecs::to_local(world, arp);
    const fm::Vec2 start = mid.xy() - fm::heading_vector(heading) * (0.5 * f.runway_length);
    return {{start.x, start.y, f.elevation}, heading};
}

// A point `along` the centerline from the threshold and `right` of it.
fm::Vec3 field(flecs::world& world, const ankara::Airfield& f, double along, double right) {
    const fm::Runway r = runway(world, f);
    const fm::Vec2 dir = fm::heading_vector(r.heading);
    const fm::Vec2 p = r.start.xy() + dir * along + fm::right_of(dir) * right;
    return {p.x, p.y, f.elevation};
}

fm::GeoPoint geo(flecs::world& world, const fm::Vec3& p) { return fm::ecs::geo(world)->to_geo(p); }

const game::Transform& transform(flecs::entity e) { return *e.try_get<game::Transform>(); }
const fm::ecs::FlightPose& pose(flecs::entity e) { return *e.try_get<fm::ecs::FlightPose>(); }

fm::Vec3 position(flecs::world& world, flecs::entity e) {
    const game::Transform& t = transform(e);
    return local(world, t.latitude_deg, t.longitude_deg, t.alt_m);
}

double distance(flecs::world& world, flecs::entity e, double lat, double lon) {
    return (position(world, e) - local(world, lat, lon, 0.0)).xy().length();
}
double distance(flecs::world& world, flecs::entity e, const ankara::Place& p) {
    return distance(world, e, p.lat, p.lon);
}

double height_above_terrain(flecs::entity e) {
    const game::Transform& t = transform(e);
    return t.alt_m - ankara::terrain(t.latitude_deg, t.longitude_deg);
}

// Where `e` is on runway `r`: meters along it from the threshold, meters right of it.
fm::Vec2 on_runway(flecs::world& world, flecs::entity e, const fm::Runway& r) {
    const fm::Vec2 d = position(world, e).xy() - r.start.xy();
    const fm::Vec2 dir = fm::heading_vector(r.heading);
    return {dot(d, dir), dot(d, fm::right_of(dir))};
}

bool landed_on(flecs::world& world, flecs::entity e, const fm::Runway& r, const ankara::Airfield& f) {
    const fm::Vec2 at = on_runway(world, e, r);
    return pose(e).phase == fm::FlightPhase::Landed && at.x > 0.0 && at.x < f.runway_length &&
           std::abs(at.y) < 20.0;
}

// Distance of a follower from its formation slot.
double slot_error(flecs::world& world, flecs::entity follower, flecs::entity leader,
                  const fm::FormationSlot& slot) {
    const fm::Vec3 l = position(world, leader);
    const double heading = fm::ecs::agent(leader)->state().attitude.heading;
    const fm::Vec2 fwd = fm::heading_vector(heading);
    const fm::Vec2 at = l.xy() + fm::right_of(fwd) * slot.offset.x + fwd * slot.offset.y;
    const fm::Vec3 f = position(world, follower);
    return (fm::Vec3{at.x, at.y, l.z + slot.offset.z} - f).length();
}

void run(flecs::world& world, double seconds) {
    for (int i = 0; i < static_cast<int>(seconds * 60.0); ++i) world.progress(1.0f / 60.0f);
}

// --- Demos ---------------------------------------------------------------------------------

// Scattered west of Ankara, a Hurjet and two Kizilelma form up (the leader orbits until the
// UAVs have joined), fly to Mogan lake as a formation and orbit there. Then they split:
// Kizilelma-2 stays in an orbit over the lake, the Hurjet and Kizilelma-1 recover to Akinci
// with an overhead break and land one after the other.
void demo_rejoin(Trace& trace) {
    flecs::world world = make_world();
    flecs::entity lead = spawn(world, "Hurjet", fm::presets::jet_trainer(),
                               {39.72, 32.38, 3000.0, 0, 0, 70.0}, 160.0, false);
    flecs::entity uav1 = spawn(world, "Kizilelma-1", fm::presets::jet_uav(),
                               {39.66, 32.28, 2500.0, 0, 0, 40.0}, 140.0, false);
    flecs::entity uav2 = spawn(world, "Kizilelma-2", fm::presets::jet_uav(),
                               {39.80, 32.30, 3400.0, 0, 0, 130.0}, 140.0, false);
    const fm::FormationSlot slot1 = fm::formation::echelon_right(0, 60.0);
    const fm::FormationSlot slot2 = fm::formation::echelon_right(1, 60.0);
    fm::ecs::follow(uav1, lead, slot1);
    fm::ecs::follow(uav2, lead, slot2);
    {
        fm::Route r;  // as a formation to Mogan lake, orbiting there
        r.waypoints = {wp(world, ankara::kMogan, 3000.0)};
        r.on_end = fm::EndBehavior::Loiter;
        r.loiter_radius = 3500.0;
        fm::ecs::agent(lead)->follow_route(r);
    }
    const fm::Runway akinci = runway(world, ankara::kAkinci);

    bool leader_waited = false;  // orbited while the UAVs were joining
    double joined_since = -1.0;
    bool split = false;
    double split_time = 0.0;
    double worst_slot_at_split = 1e9;
    std::vector<double> touchdown(2, -1.0);
    for (int t = 0; t < 2000; ++t) {
        run(world, 1.0);
        trace.sample(world, t);
        leader_waited = leader_waited || fm::ecs::agent(lead)->fixed_wing()->waiting_for_formation();
        const double worst = std::max(slot_error(world, uav1, lead, slot1),
                                      slot_error(world, uav2, lead, slot2));
        if (!split) {
            const bool over_lake = pose(lead).mode == fm::FlightMode::Loiter;
            joined_since = worst < 100.0 ? (joined_since < 0.0 ? t : joined_since) : -1.0;
            if (over_lake && joined_since >= 0.0 && t - joined_since > 60.0) {
                // In position: Kizilelma-2 stays over the lake, the others recover.
                worst_slot_at_split = worst;
                fm::ecs::stop_following(uav2);
                fm::ecs::agent(uav2)->loiter(local(world, ankara::kMogan.lat, ankara::kMogan.lon, 3600.0),
                                            4000.0);
                fm::ecs::FlightLanding landing;
                landing.interval = 15.0;
                fm::ecs::land_flight(lead, {uav1}, akinci, landing);
                split = true;
                split_time = t;
                std::printf("  t=%4d s  formed up over Mogan lake: split\n", t);
            }
        }
        const flecs::entity flight[] = {lead, uav1};
        for (int k = 0; k < 2; ++k)
            if (split && touchdown[k] < 0.0 && pose(flight[k]).on_ground) touchdown[k] = t;
        if (split && t > split_time + 120.0 && landed_on(world, lead, akinci, ankara::kAkinci) &&
            landed_on(world, uav1, akinci, ankara::kAkinci))
            break;
    }
    run(world, 120.0);  // Kizilelma-2 keeps orbiting
    check(leader_waited, "the Hurjet orbited until the Kizilelma had joined (form up first)");
    check(split && worst_slot_at_split < 100.0, "formed up in echelon, in position over Mogan lake");
    check(landed_on(world, lead, akinci, ankara::kAkinci) &&
              landed_on(world, uav1, akinci, ankara::kAkinci),
          "Hurjet and Kizilelma-1 landed at Akinci");
    check(touchdown[0] > 0.0 && touchdown[1] - touchdown[0] > 8.0,
          "Kizilelma-1 touched down after the Hurjet");
    check(pose(uav2).mode == fm::FlightMode::Loiter &&
              std::abs(distance(world, uav2, ankara::kMogan) - 4000.0) < 400.0 &&
              std::abs(transform(uav2).alt_m - 3600.0) < 30.0,
          "Kizilelma-2 orbits Mogan lake at 3,600 m");
}

// A Hurkus on the apron west of Akinci's runway: taxi out, takeoff, a short tour north to
// Kahramankazan and back over Etimesgut, an overhead break, the roll to a stop point and the
// taxi back to its parking spot.
void demo_circuit(Trace& trace) {
    flecs::world world = make_world();
    const ankara::Airfield& f = ankara::kAkinci;
    const fm::Runway rwy = runway(world, f);
    const fm::GeoPoint apron = geo(world, field(world, f, 400.0, -250.0));
    // Parked on the apron, nose towards the runway.
    flecs::entity hurkus = spawn(world, "Hurkus", fm::presets::turboprop_trainer(),
                                 {apron.latitude, apron.longitude, 0.0, 0, 0, f.runway_heading + 90.0},
                                 0.0, true);
    {
        fm::TakeoffPlan plan;
        plan.taxi = {field(world, f, 400.0, -120.0), field(world, f, -40.0, -120.0)};
        plan.runway = rwy;
        fm::Route tour;
        tour.waypoints = {wp(world, ankara::kKahramankazan, 1700.0),
                          wp(world, ankara::kEtimesgut, 1500.0)};
        plan.then = tour;
        fm::ecs::agent(hurkus)->takeoff(plan);
    }
    bool lined_up = false, airborne = false, overhead = false, broke = false, taxi_in = false;
    double closest[2] = {1e9, 1e9};
    for (int t = 0; t < 1800; ++t) {
        run(world, 1.0);
        trace.sample(world, t);
        const fm::ecs::FlightPose& p = pose(hurkus);
        lined_up = lined_up || p.phase == fm::FlightPhase::LineUp;
        airborne = airborne || !p.on_ground;
        closest[0] = std::min(closest[0], distance(world, hurkus, ankara::kKahramankazan));
        closest[1] = std::min(closest[1], distance(world, hurkus, ankara::kEtimesgut));
        // Tour done: back for an overhead break; the roll ends 1,500 m down the runway.
        if (airborne && !overhead && p.mode == fm::FlightMode::Cruise) {
            fm::LandingOptions o;
            o.pattern = fm::LandingPattern::Overhead;
            o.break_distance = 500.0;
            o.stop_distance = 1500.0;
            overhead = fm::ecs::agent(hurkus)->land(rwy, o);
            std::printf("  t=%4d s  tour flown: overhead recovery\n", t);
        }
        broke = broke || (p.mode == fm::FlightMode::Land &&
                          fm::ecs::agent(hurkus)->fixed_wing()->landing_stage() ==
                              fm::FixedWingAgent::LandingStage::Break);
        if (overhead && !taxi_in && p.phase == fm::FlightPhase::Landed) {
            const std::vector<fm::Vec3> to_apron{field(world, f, 1550.0, -120.0),
                                                 field(world, f, 400.0, -120.0),
                                                 field(world, f, 400.0, -250.0)};
            taxi_in = fm::ecs::agent(hurkus)->taxi(to_apron, rwy.heading - fm::kPi / 2.0);  // nose in
            std::printf("  t=%4d s  landed: taxi back to the apron\n", t);
        }
        if (taxi_in && p.phase == fm::FlightPhase::Parked) break;
    }
    check(lined_up && airborne, "taxied out, lined up on runway 03 and took off");
    check(closest[0] < 100.0 && closest[1] < 100.0, "tour over Kahramankazan and Etimesgut");
    check(broke, "overhead break over the runway");
    check(taxi_in && pose(hurkus).phase == fm::FlightPhase::Parked &&
              distance(world, hurkus, apron.latitude, apron.longitude) < 3.0,
          "landed and parked again on the apron");
}

// An Anka tour from Akinci: Kahramankazan (corner cut), on at 110 KCAS, terrain following
// 400 m above Elmadag, Mogan lake flown over, the sharp turn back flown over too, landing at
// Akinci. Route turns at its 25 deg navigation bank.
void demo_waypoints(Trace& trace) {
    flecs::world world = make_world();
    const ankara::Airfield& f = ankara::kAkinci;
    const fm::Runway rwy = runway(world, f);
    const fm::GeoPoint start = geo(world, rwy.start);
    flecs::entity anka = spawn(world, "Anka", fm::presets::propeller_uav(),
                               {start.latitude, start.longitude, 0.0, 0, 0, f.runway_heading},
                               0.0, true);
    const ankara::Place east{"east of Esenboga", 40.0900, 33.0700};
    fm::Route r;
    r.waypoints = {wp(world, ankara::kKahramankazan, 1800.0), wp(world, east, 1900.0),
                   wp(world, ankara::kElmadag, 400.0, fm::AltitudeRef::AboveTerrain),
                   wp(world, ankara::kMogan, 1700.0)};
    r.waypoints[1].speed = 110.0;  // knots CAS from here on
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
    const ankara::Place* points[] = {&ankara::kKahramankazan, &east, &ankara::kElmadag,
                                     &ankara::kMogan};
    double closest[4] = {1e9, 1e9, 1e9, 1e9};
    double agl_at_elmadag = 0.0, kcas_on_leg = 0.0, max_bank = 0.0;
    for (int t = 0; t < 5000; ++t) {
        run(world, 1.0);
        trace.sample(world, t);
        const fm::ecs::FlightPose& p = pose(anka);
        if (p.mode == fm::FlightMode::Route) {
            const std::size_t leg = fm::ecs::agent(anka)->fixed_wing()->route().index();
            for (int k = 0; k < 4; ++k)
                closest[k] = std::min(closest[k], distance(world, anka, *points[k]));
            if (distance(world, anka, ankara::kElmadag) < 1500.0)
                agl_at_elmadag = height_above_terrain(anka);
            // Level on the leg to Elmadag (the climb over the mountain is flown slower).
            if (leg == 2 && distance(world, anka, east) > 5000.0 &&
                distance(world, anka, ankara::kElmadag) > 15000.0)
                kcas_on_leg = p.calibrated_airspeed / 0.514444;
            max_bank = std::max(max_bank, std::abs(transform(anka).roll_deg));
        }
        if (p.phase == fm::FlightPhase::Landed) break;
    }
    check(closest[0] < 0.3 * fm::ecs::agent(anka)->fixed_wing()->turn_radius(50.0) + 100.0 &&
              closest[1] < 600.0,
          "corners cut at Kahramankazan and east of Esenboga");
    check(std::abs(kcas_on_leg - 110.0) < 3.0, "110 knots CAS after the second waypoint");
    check(std::abs(agl_at_elmadag - 400.0) < 80.0 && closest[2] < 300.0,
          "400 m above the terrain over Elmadag");
    check(closest[3] < 50.0, "Mogan lake flown over");
    check(max_bank < 26.0, "route turns at the 25 deg navigation bank");
    check(landed_on(world, anka, rwy, f), "landed back at Akinci");
}

// Republic Day: four F-16s taxi out of their shelters, take off 10 s apart and rejoin in a
// diamond, pass over Anitkabir at 300 knots, then recover to Akinci with an overhead break.
void demo_flypast(Trace& trace) {
    flecs::world world = make_world();
    const ankara::Airfield& f = ankara::kAkinci;
    const fm::Runway rwy = runway(world, f);
    std::vector<flecs::entity> vipers;
    for (int k = 0; k < 4; ++k) {  // shelters east of the runway, noses towards it
        const fm::GeoPoint shelter = geo(world, field(world, f, 300.0 + 70.0 * k, 260.0));
        const std::string name = "Viper-" + std::to_string(k + 1);
        vipers.push_back(spawn(world, name.c_str(), fm::presets::single_engine_fighter(),
                               {shelter.latitude, shelter.longitude, 0.0, 0, 0, f.runway_heading - 90.0},
                               0.0, true));
    }
    // Diamond: two behind on the sides, the fourth in the slot behind the leader, lower.
    const std::vector<fm::FormationSlot> diamond = {
        {{-50.0, -50.0, 0.0}}, {{50.0, -50.0, 0.0}}, {{0.0, -100.0, -10.0}}};
    const ankara::Place in{"flypast start", 39.9400, 32.6600};
    const ankara::Place out{"flypast end", 39.9100, 33.0000};
    {
        fm::TakeoffPlan plan;
        plan.runway = rwy;
        plan.taxi = {field(world, f, 300.0, 180.0), field(world, f, -20.0, 180.0),
                     field(world, f, -20.0, 70.0)};
        fm::Route r;
        r.waypoints = {wp(world, in, 1550.0), wp(world, ankara::kAnitkabir, 1500.0),
                       wp(world, out, 1600.0)};
        r.waypoints[0].speed = 300.0;
        r.waypoints[0].speed_ref = fm::SpeedRef::CalibratedKnots;
        r.waypoints[1].fly_over = true;
        plan.then = r;
        fm::ecs::FlightTakeoff options;
        options.interval = 10.0;
        options.slots = diamond;
        fm::ecs::takeoff_flight(vipers[0], {vipers[1], vipers[2], vipers[3]}, plan, options);
    }
    double over_anitkabir = 1e9;  // leader's closest pass
    double worst_slot_there = 1e9;
    bool recovering = false;
    std::vector<double> touchdown(4, -1.0);
    for (int t = 0; t < 2400; ++t) {
        run(world, 1.0);
        trace.sample(world, t);
        const double d = distance(world, vipers[0], ankara::kAnitkabir);
        if (d < over_anitkabir) {
            over_anitkabir = d;
            worst_slot_there = 0.0;
            for (int k = 1; k < 4; ++k)
                worst_slot_there = std::max(
                    worst_slot_there, slot_error(world, vipers[k], vipers[0], diamond[k - 1]));
        }
        if (!recovering && !fm::ecs::flight_sequence_active(vipers[0]) &&
            pose(vipers[0]).mode == fm::FlightMode::Cruise) {
            fm::ecs::FlightLanding landing;
            landing.interval = 12.0;
            recovering = fm::ecs::land_flight(vipers[0], {vipers[1], vipers[2], vipers[3]}, rwy, landing);
            std::printf("  t=%4d s  flypast done: overhead recovery\n", t);
        }
        for (int k = 0; k < 4; ++k)
            if (recovering && touchdown[k] < 0.0 && pose(vipers[k]).on_ground) touchdown[k] = t;
        if (recovering && std::all_of(vipers.begin(), vipers.end(), [&](flecs::entity v) {
                return landed_on(world, v, rwy, f);
            }))
            break;
    }
    check(over_anitkabir < 100.0 && worst_slot_there < 60.0,
          "diamond over Anitkabir, every aircraft within 60 m of its slot");
    std::vector<double> sorted = touchdown;
    std::sort(sorted.begin(), sorted.end());
    bool spaced = sorted.front() > 0.0;
    for (int k = 1; k < 4; ++k) spaced = spaced && sorted[k] - sorted[k - 1] >= 8.0;
    check(spaced && std::all_of(vipers.begin(), vipers.end(),
                                [&](flecs::entity v) { return landed_on(world, v, rwy, f); }),
          "all four landed one after the other after the break");
}

// Four aircraft converge on Kizilay at 2,500 m from the four directions, all arriving at the
// same time. Collision avoidance separates them vertically (the slow UAVs too, which climbed
// or descended for the others first); then each flies on. The same encounter without
// collision avoidance, for comparison.
void demo_traffic(Trace& trace) {
    struct Inbound {
        const char* name;
        fm::FixedWingParams params;
        double bearing;  // deg true, from Kizilay to the start
        double speed;    // m/s
    };
    const Inbound inbound[] = {{"Hurkus", fm::presets::turboprop_trainer(), 0.0, 110.0},
                               {"Kizilelma", fm::presets::jet_uav(), 90.0, 150.0},
                               {"Anka", fm::presets::propeller_uav(), 180.0, 50.0},
                               {"Aksungur", fm::presets::twin_propeller_uav(), 270.0, 52.0}};
    struct Outcome {
        double closest = 1e9;  // m, between any two
        bool avoided = false;
        bool past = true;
    };
    const auto encounter = [&](bool avoidance, Trace* tr) {
        flecs::world world = make_world();
        fm::ecs::set_collision_avoidance(world, avoidance);
        const double arrival = 150.0;  // s
        const fm::Vec3 centre = local(world, ankara::kKizilay.lat, ankara::kKizilay.lon, 2500.0);
        std::vector<flecs::entity> aircraft;
        for (const Inbound& in : inbound) {
            const fm::Vec2 dir = fm::heading_vector(fm::deg2rad(in.bearing));
            const fm::Vec3 at = centre + fm::Vec3{dir.x, dir.y, 0.0} * (in.speed * arrival);
            const fm::GeoPoint g = geo(world, at);
            flecs::entity e = spawn(world, in.name, in.params,
                                    {g.latitude, g.longitude, 2500.0, 0, 0, in.bearing + 180.0},
                                    in.speed, false);
            fm::GoToOptions o;
            o.speed = in.speed;
            fm::ecs::agent(e)->go_to(centre, o);  // and straight on
            aircraft.push_back(e);
        }
        Outcome out;
        for (int t = 0; t < 400; ++t) {
            for (int i = 0; i < 4; ++i) {  // the closest approach is quick: check 4 times a second
                run(world, 0.25);
                for (std::size_t a = 0; a < aircraft.size(); ++a)
                    for (std::size_t b = a + 1; b < aircraft.size(); ++b)
                        out.closest = std::min(out.closest, (position(world, aircraft[a]) -
                                                             position(world, aircraft[b])).length());
            }
            if (tr) tr->sample(world, t);
            for (flecs::entity e : aircraft)
                out.avoided = out.avoided || fm::ecs::agent(e)->state().avoiding_traffic;
        }
        for (flecs::entity e : aircraft) out.past = out.past && distance(world, e, ankara::kKizilay) > 2000.0;
        return out;
    };
    const Outcome with = encounter(true, &trace);
    const Outcome without = encounter(false, nullptr);
    std::printf("  closest approach: %.0f m with collision avoidance, %.0f m without\n",
                with.closest, without.closest);
    check(with.avoided, "conflicts predicted and resolved by climbing / descending");
    check(with.closest > 100.0 && without.closest < 30.0,
          "kept 100 m apart (near collisions without collision avoidance)");
    check(with.past, "everyone flew on past Kizilay");
}

// A T129 lifts off at Guvercinlik and flies 40 m above the terrain past Eymir lake to
// Elmadag, hovers a minute over the summit and lands on a ridge east of it.
void demo_lowlevel(Trace& trace) {
    flecs::world world = make_world();
    const ankara::Airfield& f = ankara::kGuvercinlik;
    flecs::entity atak = spawn(world, "T129", fm::presets::attack_helicopter(),
                               {f.lat, f.lon, 0.0, 0, 0, 110.0}, 0.0, true);
    const ankara::Place foothills{"Elmadag foothills", 39.8700, 33.1000};
    const ankara::Place ridge{"ridge east of Elmadag", 39.8550, 33.2700};
    fm::Route r;
    r.waypoints = {wp(world, ankara::kEymir, 40.0, fm::AltitudeRef::AboveTerrain),
                   wp(world, foothills, 40.0, fm::AltitudeRef::AboveTerrain),
                   wp(world, ankara::kElmadag, 60.0, fm::AltitudeRef::AboveTerrain),
                   wp(world, ridge, 40.0, fm::AltitudeRef::AboveTerrain)};
    r.waypoints[2].hold_time = 60.0;  // hover over the summit
    r.on_end = fm::EndBehavior::Land;
    fm::ecs::agent(atak)->follow_route(r);
    double min_agl = 1e9, max_agl = 0.0;
    bool hovered = false;
    for (int t = 0; t < 1500; ++t) {
        run(world, 1.0);
        trace.sample(world, t);
        const fm::ecs::FlightPose& p = pose(atak);
        const std::size_t leg = fm::ecs::agent(atak)->rotorcraft()->route().index();
        if (p.mode == fm::FlightMode::Route && leg >= 1 && leg <= 2 &&
            distance(world, atak, ankara::kEymir) > 1000.0) {  // en route, past the climb
            min_agl = std::min(min_agl, height_above_terrain(atak));
            max_agl = std::max(max_agl, height_above_terrain(atak));
        }
        hovered = hovered || (distance(world, atak, ankara::kElmadag) < 30.0 && p.ground_speed < 1.0);
        if (p.phase == fm::FlightPhase::Landed) break;
    }
    std::printf("  height above the terrain en route: %.0f to %.0f m\n", min_agl, max_agl);
    check(min_agl > 15.0 && max_agl < 150.0, "terrain following, 40 m above the hills");
    check(hovered, "hovered over the summit");
    check(pose(atak).phase == fm::FlightPhase::Landed && distance(world, atak, ridge) < 5.0,
          "landed on the ridge");
}

// Landings ordered from high above the field: an airliner 2,500 m over Esenboga and an
// Aksungur 2,000 m over Akinci. Each first descends in an orbit outside the final approach
// fix, then flies a stabilised straight-in approach (no go-around).
void demo_recovery(Trace& trace) {
    flecs::world world = make_world();
    const fm::Runway esb = runway(world, ankara::kEsenboga);
    const fm::Runway akn = runway(world, ankara::kAkinci);
    flecs::entity airliner =
        spawn(world, "Airliner", fm::presets::airliner_jet(),
              {ankara::kEsenboga.lat, ankara::kEsenboga.lon, ankara::kEsenboga.elevation + 2500.0, 0, 0, 200.0},
              150.0, false);
    flecs::entity aksungur =
        spawn(world, "Aksungur", fm::presets::twin_propeller_uav(),
              {ankara::kAkinci.lat, ankara::kAkinci.lon, ankara::kAkinci.elevation + 2000.0, 0, 0, 300.0},
              50.0, false);
    fm::ecs::agent(airliner)->land(esb);
    fm::ecs::agent(aksungur)->land(akn);
    bool went_around = false;
    for (int t = 0; t < 2400; ++t) {
        run(world, 1.0);
        trace.sample(world, t);
        went_around = went_around || pose(airliner).phase == fm::FlightPhase::GoAround ||
                      pose(aksungur).phase == fm::FlightPhase::GoAround;
        if (landed_on(world, airliner, esb, ankara::kEsenboga) &&
            landed_on(world, aksungur, akn, ankara::kAkinci))
            break;
    }
    check(!went_around, "stabilised approaches, no go-around");
    check(landed_on(world, airliner, esb, ankara::kEsenboga), "airliner landed at Esenboga");
    check(landed_on(world, aksungur, akn, ankara::kAkinci), "Aksungur landed at Akinci");
}

// Station relief: an Anka orbits a station west of Ankara. An Aksungur takes off at Akinci,
// flies there and takes over the orbit 300 m higher; once it orbits the station, the Anka
// leaves and lands back at Akinci.
void demo_relief(Trace& trace) {
    flecs::world world = make_world();
    const ankara::Airfield& f = ankara::kAkinci;
    const fm::Runway rwy = runway(world, f);
    const ankara::Place station{"station west of Ankara", 39.8800, 32.4000};
    flecs::entity anka = spawn(world, "Anka", fm::presets::propeller_uav(),
                               {station.lat, station.lon + 0.03, 2400.0, 0, 0, 0.0}, 46.0, false);
    fm::ecs::agent(anka)->loiter(local(world, station.lat, station.lon, 2400.0), 2500.0);
    const fm::GeoPoint start = geo(world, rwy.start);
    flecs::entity aksungur = spawn(world, "Aksungur", fm::presets::twin_propeller_uav(),
                                   {start.latitude, start.longitude, 0.0, 0, 0, f.runway_heading},
                                   0.0, true);
    {
        fm::TakeoffPlan plan;
        plan.runway = rwy;
        fm::Route r;  // to the station, orbiting there above the Anka
        r.waypoints = {wp(world, station, 2700.0)};
        r.on_end = fm::EndBehavior::Loiter;
        r.loiter_radius = 2500.0;
        plan.then = r;
        fm::ecs::agent(aksungur)->takeoff(plan);
    }
    bool relieved = false;
    double both_on_station = 0.0;  // s with both orbiting the station
    double closest = 1e9;
    for (int t = 0; t < 4000; ++t) {
        run(world, 1.0);
        trace.sample(world, t);
        const bool anka_on = pose(anka).mode == fm::FlightMode::Loiter;
        const bool aksungur_on = pose(aksungur).mode == fm::FlightMode::Loiter &&
                                 distance(world, aksungur, station) < 3000.0;
        if (anka_on && aksungur_on) both_on_station += 1.0;
        if (!pose(aksungur).on_ground)
            closest = std::min(closest, (position(world, anka) - position(world, aksungur)).length());
        // Relieved after a full lap together: the Anka goes home.
        if (!relieved && both_on_station > 240.0) {
            relieved = fm::ecs::agent(anka)->land(rwy);
            std::printf("  t=%4d s  Aksungur on station: Anka returns to Akinci\n", t);
        }
        if (relieved && landed_on(world, anka, rwy, f)) break;
    }
    check(both_on_station > 240.0, "Aksungur reached the station and orbited with the Anka");
    check(closest > 250.0, "kept apart on station (300 m higher)");
    check(landed_on(world, anka, rwy, f), "Anka landed back at Akinci");
    check(pose(aksungur).mode == fm::FlightMode::Loiter && distance(world, aksungur, station) < 3000.0,
          "Aksungur stays on station");
}

// A convoy drives west along the Eskisehir road (game logic, not the flight model). A T129
// keeps 300 m behind it and 150 m above, a drone 60 m above and beside it.
void demo_escort(Trace& trace) {
    flecs::world world = make_world();
    struct Vehicle {
        double speed = 0.0;  // m/s
    };
    // Game logic: the convoy drives on along its yaw, on the ground.
    world.system<game::Transform, const Vehicle>("game::Drive")
        .kind(flecs::OnUpdate)
        .each([](flecs::iter& it, size_t, game::Transform& t, const Vehicle& v) {
            const double d = v.speed * it.delta_time();
            const double yaw = fm::deg2rad(t.yaw_deg);
            t.latitude_deg += d * std::cos(yaw) / 111132.0;
            t.longitude_deg += d * std::sin(yaw) / (111320.0 * std::cos(fm::deg2rad(t.latitude_deg)));
            t.alt_m = ankara::terrain(t.latitude_deg, t.longitude_deg);
        });
    // The game publishes the convoy's position for its escorts.
    world.system<const game::Transform, fm::ecs::TrackedGeoPose>("bridge::TrackedIn")
        .kind(flecs::PreUpdate)
        .each([](const game::Transform& t, fm::ecs::TrackedGeoPose& tracked) {
            tracked.position = {t.latitude_deg, t.longitude_deg, t.alt_m};
            tracked.yaw = t.yaw_deg;
        });
    const double lat0 = 39.9000, lon0 = 32.7600;
    flecs::entity convoy = world.entity("Convoy")
                               .set<game::Transform>({lat0, lon0, ankara::terrain(lat0, lon0), 0, 0, 245.0})
                               .set<Vehicle>({15.0})
                               .set<fm::ecs::TrackedGeoPose>({});
    flecs::entity atak = spawn(world, "T129", fm::presets::attack_helicopter(),
                               {39.94, 32.74, 1300.0, 0, 0, 200.0}, 40.0, false);
    flecs::entity drone = spawn(world, "Drone", fm::presets::multirotor(),
                                {lat0 + 0.002, lon0, ankara::terrain(lat0, lon0) + 50.0, 0, 0, 0.0},
                                0.0, false);
    const fm::FormationSlot behind{{0.0, -300.0, 150.0}};
    const fm::FormationSlot beside{{60.0, -20.0, 60.0}};
    fm::ecs::follow(atak, convoy, behind);
    fm::ecs::follow(drone, convoy, beside);
    const auto slot_error_of = [&](flecs::entity e, const fm::FormationSlot& slot) {
        const game::Transform& c = transform(convoy);
        const fm::Vec3 l = local(world, c.latitude_deg, c.longitude_deg, c.alt_m);
        const fm::Vec2 fwd = fm::heading_vector(fm::ecs::local_heading(world, c.yaw_deg, {c.latitude_deg, c.longitude_deg, c.alt_m}));
        const fm::Vec2 at = l.xy() + fm::right_of(fwd) * slot.offset.x + fwd * slot.offset.y;
        return (fm::Vec3{at.x, at.y, l.z + slot.offset.z} - position(world, e)).length();
    };
    double worst[2] = {0.0, 0.0};
    for (int t = 0; t < 900; ++t) {
        run(world, 1.0);
        trace.sample(world, t);
        if (t > 240) {  // joined up
            worst[0] = std::max(worst[0], slot_error_of(atak, behind));
            worst[1] = std::max(worst[1], slot_error_of(drone, beside));
        }
    }
    const double driven = ankara::metres(transform(convoy).latitude_deg, transform(convoy).longitude_deg, lat0, lon0);
    std::printf("  convoy drove %.1f km; worst slot error T129 %.0f m, drone %.0f m\n",
                driven / 1000.0, worst[0], worst[1]);
    check(driven > 13000.0, "the convoy drove west along the road");
    check(worst[0] < 60.0, "T129 keeps 300 m behind the convoy, 150 m above");
    check(worst[1] < 20.0, "drone keeps its place beside the convoy");
}

struct Demo {
    const char* name;
    const char* title;
    void (*run)(Trace&);
};

const Demo kDemos[] = {
    {"rejoin", "Formation rejoin, split: two land at Akinci, one orbits Mogan", demo_rejoin},
    {"circuit", "Hurkus: taxi, takeoff, short tour, overhead break, taxi back", demo_circuit},
    {"waypoints", "Anka waypoint tour: corners, knots, terrain following, landing", demo_waypoints},
    {"flypast", "Four F-16s: takeoff, diamond over Anitkabir, overhead recovery", demo_flypast},
    {"traffic", "Four aircraft converging over Kizilay, collision avoidance", demo_traffic},
    {"lowlevel", "T129 low level to Elmadag, hover, landing on a ridge", demo_lowlevel},
    {"recovery", "Landings ordered from high above Esenboga and Akinci", demo_recovery},
    {"relief", "Station relief: Aksungur takes over the orbit, Anka lands", demo_relief},
    {"escort", "T129 and a drone escort a convoy on the Eskisehir road", demo_escort},
};

}  // namespace

int main(int argc, char** argv) {
    std::vector<std::string> wanted;
    const char* csv = nullptr;
    const char* kml = nullptr;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--csv") == 0 && i + 1 < argc) {
            csv = argv[++i];
        } else if (std::strcmp(argv[i], "--kml") == 0 && i + 1 < argc) {
            kml = argv[++i];
        } else if (std::strcmp(argv[i], "--list") == 0) {
            for (const Demo& d : kDemos) std::printf("%-10s %s\n", d.name, d.title);
            return 0;
        } else {
            wanted.push_back(argv[i]);
        }
    }
    Trace trace;
    if (csv && !trace.open_csv(csv)) {
        std::fprintf(stderr, "cannot write %s\n", csv);
        return 2;
    }
    if (kml) trace.enable_kml();
    int ran = 0;
    for (const Demo& d : kDemos) {
        if (!wanted.empty() && std::find(wanted.begin(), wanted.end(), d.name) == wanted.end())
            continue;
        std::printf("== %s: %s\n", d.name, d.title);
        trace.begin(d.name);
        d.run(trace);
        ++ran;
    }
    if (ran == 0) {
        std::fprintf(stderr, "no such demo; --list shows them\n");
        return 2;
    }
    if (kml && !trace.write_kml(kml)) {
        std::fprintf(stderr, "cannot write %s\n", kml);
        return 2;
    }
    std::printf("\n%s\n", failures == 0 ? "all demos OK" : "SOME DEMOS FAILED");
    return failures == 0 ? 0 : 1;
}
