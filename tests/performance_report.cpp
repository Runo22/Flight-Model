// Prints the performance of every preset, measured by flying it through the agents, next to
// published reference values (markdown tables, used in docs/PRESETS.md).
//
// Usage: fm_performance_report [preset name]
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "fm/presets.hpp"
#include "performance.hpp"

using namespace fm;
using namespace fmperf;

namespace {

struct Row {
    std::string what;
    std::string measured;
    std::string reference;
};

// printf-style formatting of one number, with thousands separators for whole numbers >= 10,000.
std::string format(const char* fmt, double value) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), fmt, value);
    std::string out = buffer;
    const std::size_t start = out.find_first_of("0123456789");
    if (start == std::string::npos) return out;
    std::size_t end = start;
    while (end < out.size() && out[end] >= '0' && out[end] <= '9') ++end;
    if (end - start > 4 && (end == out.size() || out[end] != '.')) {
        for (std::size_t i = end; i > start + 3; i -= 3) out.insert(i - 3, ",");
    }
    return out;
}

void print(const std::string& title, const std::vector<Row>& rows) {
    std::printf("### %s\n\n| | Measured | Reference |\n|---|---|---|\n", title.c_str());
    for (const Row& r : rows)
        std::printf("| %s | %s | %s |\n", r.what.c_str(), r.measured.c_str(), r.reference.c_str());
    std::printf("\n");
}

struct FixedWingReference {
    double high_altitude = 0.0;  // m, where the top speed is given
    std::string top_speed, sea_level_speed, climb, ceiling, stall, takeoff, landing;
    double light_mass = 0.0;     // kg, also measure the ceiling at this mass (0 = no)
    std::string light_ceiling{};
    // Endurance (presets with fuel): altitude (0 = high_altitude), speed (0 = the speed of
    // the lowest fuel flow), reference; and at light_mass.
    double endurance_altitude = 0.0;
    double endurance_speed = 0.0;
    std::string endurance{};
    std::string light_endurance{};
};

std::string hours(double h) {
    const int minutes = static_cast<int>(std::lround(h * 60.0));
    return std::to_string(minutes / 60) + " h " + (minutes % 60 < 10 ? "0" : "") +
           std::to_string(minutes % 60) + " min";
}

void report(const FixedWingParams& p, const std::string& title, const FixedWingReference& ref) {
    const double h = ref.high_altitude;
    const double v_high = max_level_speed(p, h, false);
    const double v_sl = max_level_speed(p, 0.0, true);
    const Climb climb = best_climb(p, 0.0);
    const Takeoff to = takeoff(p);
    const Landing ld = landing(p);
    const Turn turn = course_reversal(p, 3000.0, p.cruise_speed);
    const double a = isa(h).speed_of_sound;
    std::vector<Row> rows = {
        {"Top speed at " + format("%.0f ft", ft(h)),
         format("%.0f kt TAS", kt(v_high)) + format(" (M%.2f)", v_high / a), ref.top_speed},
        {"Top speed at sea level (with limits)", format("%.0f kt", kt(v_sl)), ref.sea_level_speed},
        {"Best climb at sea level", format("%.0f ft/min", fpm(climb.rate)) +
                                        format(" at %.0f kt", kt(climb.speed)), ref.climb},
        {"Service ceiling (100 ft/min)", format("%.0f ft", ft(service_ceiling(p))), ref.ceiling},
        {"Stall speed, clean / landing flaps",
         format("%.0f", kt(stall_speed(p, 0.0, false))) +
             format(" / %.0f kt", kt(stall_speed(p, 0.0, true))), ref.stall},
        {"Takeoff: ground roll, lift-off, over 50 ft",
         format("%.0f m", to.ground_roll) + format(", %.0f kt", kt(to.liftoff_speed)) +
             format(", %.0f m", to.to_50ft), ref.takeoff},
        {"Landing: approach, touchdown, roll, from 50 ft",
         format("%.0f kt", kt(ld.approach_speed)) + format(", %.0f kt", kt(ld.touchdown_speed)) +
             format(", %.0f m", ld.ground_roll) + format(", %.0f m", ld.from_50ft), ref.landing},
        {"AI turn at cruise speed, 10,000 ft",
         format("%.1f deg/s", rad2deg(turn.rate)) + format(" at %.0f deg bank", rad2deg(turn.bank)),
         ""},
    };
    if (ref.light_mass > 0.0) {
        FixedWingParams light = p;
        light.mass = ref.light_mass;
        rows.insert(rows.begin() + 4, {format("Service ceiling at %.0f kg", ref.light_mass),
                                       format("%.0f ft", ft(service_ceiling(light))),
                                       ref.light_ceiling});
    }
    if (p.fuel_capacity > 0.0) {
        const double alt = ref.endurance_altitude > 0.0 ? ref.endurance_altitude : h;
        auto row = [&](const FixedWingParams& q, const std::string& what, const std::string& r) {
            const Endurance e = endurance(q, alt, ref.endurance_speed);
            rows.push_back({what + format(", %.0f ft", ft(alt)),
                            hours(e.hours) + format(" at %.0f kt", kt(e.speed)) +
                                format(", %.0f kg/h full", e.flow_full),
                            r});
        };
        row(p, format("Endurance on %.0f kg of fuel", p.fuel_capacity), ref.endurance);
        if (ref.light_mass > 0.0) {
            FixedWingParams light = p;
            light.mass = ref.light_mass;
            row(light, format("Endurance at %.0f kg", ref.light_mass), ref.light_endurance);
        }
    }
    print(title, rows);
}

struct RotorReference {
    std::string top_speed, climb, vertical_climb, hover_ceiling, ceiling;
};

void report(const RotorcraftParams& p, const std::string& title, const RotorReference& ref) {
    const RotorClimb climb = best_climb(p, 0.0);
    const Acceleration acc = accelerate(p, knots(100.0));
    const Stop stop = stop_at_point(p, p.cruise_speed);
    const RotorTurn turn = course_reversal(p, p.cruise_speed);
    std::vector<Row> rows = {
        {"Top speed, sea level / 3,000 m",
         format("%.0f", kt(max_level_speed(p, 0.0))) +
             format(" / %.0f kt", kt(max_level_speed(p, 3000.0))), ref.top_speed},
        {"Best climb", format("%.0f ft/min", fpm(climb.rate)) + format(" at %.0f kt", kt(climb.speed)),
         ref.climb},
        {"Vertical climb (out of ground effect)", format("%.0f ft/min", fpm(vertical_climb(p, 0.0))),
         ref.vertical_climb},
        {"Hover ceiling out of ground effect", format("%.0f m", hover_ceiling(p)), ref.hover_ceiling},
        {"Service ceiling", format("%.0f m", service_ceiling(p)), ref.ceiling},
        {"Pitch at cruise speed / top speed",
         format("%.1f", rad2deg(cruise_attitude(p, p.cruise_speed).pitch)) +
             format(" / %.1f deg", rad2deg(cruise_attitude(p, max_level_speed(p, 0.0)).pitch)), ""},
        {"0 to 100 kt", format("%.0f s", acc.time) + format(", %.0f deg nose down", rad2deg(acc.pitch_down)),
         ""},
        {"Stop from cruise speed", format("%.0f s", stop.time) + format(", %.0f m", stop.distance), ""},
        {"Turn at cruise speed",
         format("%.1f deg/s", rad2deg(turn.rate)) + format(", %.0f deg bank", rad2deg(turn.bank)) +
             format(", radius %.0f m", turn.radius), ""},
    };
    print(title, rows);
}

void report(const ShipParams& p, const std::string& title) {
    const ShipAcceleration acc = ship_acceleration(p);
    const ShipStop stop = ship_crash_stop(p);
    const TurningCircle turn = turning_circle(p);
    const double l = p.length;
    std::vector<Row> rows = {
        {"Top speed", format("%.1f kt", kt(acc.max_speed)), "21 kt"},
        {"0 to cruise speed / to 90% of top speed",
         format("%.0f", acc.to_cruise) + format(" / %.0f s", acc.to_90_percent), ""},
        {"Crash stop from top speed (full astern)",
         format("%.0f s", stop.time) + format(", %.0f m", stop.distance) +
             format(" (%.1f L)", stop.distance / l), "IMO: track reach <= 15 L"},
        {"Advance (full rudder at top speed)",
         format("%.0f m", turn.advance) + format(" (%.1f L)", turn.advance / l), "IMO: <= 4.5 L"},
        {"Tactical diameter", format("%.0f m", turn.tactical_diameter) +
                                  format(" (%.1f L)", turn.tactical_diameter / l),
         "IMO: <= 5 L; full-bodied ships 2.4-4 L"},
        {"Steady turning circle", format("%.0f m", turn.steady_diameter), ""},
        {"Speed in the steady turn",
         format("%.1f kt", kt(turn.speed)) +
             format(" (%.0f%% lost)", 100.0 * (1.0 - turn.speed / acc.max_speed)), ""},
        {"Heel / drift angle in the turn",
         format("%.1f", -rad2deg(turn.heel)) + format(" deg outward / %.0f deg", rad2deg(turn.drift)),
         ""},
    };
    print(title, rows);
}

bool wanted(const char* filter, const std::string& name) {
    return filter == nullptr || name == filter;
}

}  // namespace

int main(int argc, char** argv) {
    const char* filter = argc > 1 ? argv[1] : nullptr;

    if (wanted(filter, "single_engine_fighter")) {
        FixedWingReference ref{12192.0, "M2.05", "800 KCAS (limit)", "50,000 ft/min",
                               "50,000+ ft", "", "", ""};
        ref.endurance_altitude = feet(30000.0);
        ref.endurance_speed = 240.0;  // Mach 0.8
        report(presets::single_engine_fighter(),
               "single_engine_fighter: F-16C Block 50 class, 12,000 kg", ref);
    }
    if (wanted(filter, "twin_engine_fighter")) {
        FixedWingReference ref{12192.0, "M2.5+", "M1.2 (800 KCAS)", "> 50,000 ft/min",
                               "60,000-65,000 ft", "", "", ""};
        ref.endurance_altitude = feet(30000.0);
        ref.endurance_speed = 250.0;
        report(presets::twin_engine_fighter(), "twin_engine_fighter: F-15C class, 20,200 kg", ref);
    }
    if (wanted(filter, "turboprop_trainer")) {
        FixedWingReference ref{feet(30000.0), "288 KTAS max cruise", "310 KCAS", "4,370 ft/min",
                               "34,700-39,270 ft", "77 KCAS landing config", "489 m over 50 ft",
                               "593 m from 50 ft"};
        ref.endurance_altitude = feet(15000.0);
        ref.endurance_speed = knots(188.0);  // 798 nm in 4.25 h
        ref.endurance = "4 h 15 min at 15,000 ft (798 nm)";
        report(presets::turboprop_trainer(), "turboprop_trainer: TAI Hurkus class, 3,300 kg", ref);
    }
    if (wanted(filter, "jet_uav")) {
        FixedWingReference ref{feet(30000.0), "M0.9", "", "", "12,000-14,000 m", "", "", ""};
        ref.endurance_speed = 182.0;  // Mach 0.6
        ref.endurance = "~5 h";
        report(presets::jet_uav(), "jet_uav: Bayraktar Kizilelma class, 6,000 kg", ref);
    }
    if (wanted(filter, "propeller_uav")) {
        FixedWingReference ref{feet(20000.0), "", "117 kt (217 km/h)", "", "30,000 ft", "", "", ""};
        ref.endurance = "30 h";
        report(presets::propeller_uav(), "propeller_uav: TAI Anka-S class, 1,700 kg", ref);
    }
    if (wanted(filter, "twin_propeller_uav")) {
        FixedWingReference ref{feet(20000.0), "", "135 kt (250 km/h)", "",
                               "25,000 ft operating altitude with 750 kg payload", "", "", ""};
        ref.light_mass = 2700.0;
        ref.light_ceiling = "35,000 ft with 150 kg payload; 40,000 ft maximum";
        ref.endurance = "12 h with 750 kg payload (TAI, a lighter fuel load)";
        ref.light_endurance = "40 h (EO payload), 49 h demonstrated";
        report(presets::twin_propeller_uav(), "twin_propeller_uav: TAI Aksungur class, 3,300 kg",
               ref);
    }
    for (const FixedWingParams& p : {presets::fast_jet(), presets::airliner_jet(),
                                     presets::regional_turboprop(), presets::light_propeller()}) {
        FixedWingReference none;
        none.high_altitude = 6000.0;
        none.endurance_speed = p.cruise_speed;
        none.endurance_altitude = p.name == "airliner_jet"      ? 10668.0  // FL350
                                  : p.name == "fast_jet"        ? 9144.0
                                  : p.name == "light_propeller" ? 2000.0
                                                                : 6000.0;
        if (wanted(filter, p.name)) report(p, p.name + " (generic class preset)", none);
    }

    if (wanted(filter, "attack_helicopter")) {
        report(presets::attack_helicopter(), "attack_helicopter: TAI T129 ATAK class, 5,000 kg",
               {"150 kt", "2,750 ft/min (14 m/s)", "", "~3,050 m", "6,096 m"});
    }
    if (wanted(filter, "heavy_attack_helicopter")) {
        report(presets::heavy_attack_helicopter(),
               "heavy_attack_helicopter: Boeing AH-64D Apache class, 8,000 kg",
               {"158 kt", "2,500 ft/min", "1,775 ft/min", "~3,100 m (estimate)", "6,400 m"});
    }
    if (wanted(filter, "transport_helicopter")) {
        report(presets::transport_helicopter(),
               "transport_helicopter: Boeing CH-47F Chinook class, 22,680 kg (max gross)",
               {"155-160 kt cruise, 170 kt Vne", "1,522 ft/min", "", "", "6,100 m (lighter)"});
    }
    if (wanted(filter, "multirotor")) {
        const RotorcraftParams p = presets::multirotor();
        std::printf("### multirotor (generic class preset)\n\nNo power model: limits only "
                    "(max speed %.0f m/s, climb %.0f m/s).\n\n", p.max_speed, p.climb_rate_max);
    }

    if (wanted(filter, "carrier_ship")) {
        report(presets::carrier_ship(),
               "carrier_ship: TCG Anadolu (L-400) class LHD, 27,400 t, 231 m");
    }
    return 0;
}
