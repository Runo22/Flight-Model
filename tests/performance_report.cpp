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
};

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

bool wanted(const char* filter, const std::string& name) {
    return filter == nullptr || name == filter;
}

}  // namespace

int main(int argc, char** argv) {
    const char* filter = argc > 1 ? argv[1] : nullptr;

    if (wanted(filter, "single_engine_fighter")) {
        report(presets::single_engine_fighter(), "single_engine_fighter: F-16C Block 50 class, 12,000 kg",
               {12192.0, "M2.05", "800 KCAS (limit)", "50,000 ft/min", "50,000+ ft", "", "", ""});
    }
    if (wanted(filter, "twin_engine_fighter")) {
        report(presets::twin_engine_fighter(), "twin_engine_fighter: F-15C class, 20,200 kg",
               {12192.0, "M2.5+", "M1.2 (800 KCAS)", "> 50,000 ft/min", "60,000-65,000 ft", "", "", ""});
    }
    if (wanted(filter, "turboprop_trainer")) {
        report(presets::turboprop_trainer(), "turboprop_trainer: TAI Hurkus class, 3,300 kg",
               {feet(30000.0), "288 KTAS max cruise", "310 KCAS", "4,370 ft/min",
                "34,700-39,270 ft", "77 KCAS landing config", "489 m over 50 ft",
                "593 m from 50 ft"});
    }
    if (wanted(filter, "jet_uav")) {
        report(presets::jet_uav(), "jet_uav: Bayraktar Kizilelma class, 6,000 kg",
               {feet(30000.0), "M0.9", "", "", "12,000-14,000 m", "", "", ""});
    }
    for (const FixedWingParams& p : {presets::fast_jet(), presets::airliner_jet(),
                                     presets::regional_turboprop(), presets::light_propeller(),
                                     presets::fixed_wing_uav()}) {
        FixedWingReference none;
        none.high_altitude = 6000.0;
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
    return 0;
}
