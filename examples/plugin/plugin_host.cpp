// Minimal game host: owns the world and the game components, loads the flight plugin,
// unloads it while the game keeps running, loads it again and shuts down.
//
// Usage: fm_plugin_host <plugin path> [--expect-reject]
//   --expect-reject: the plugin is expected to refuse to load (wrongly linked Flecs).
#include <cmath>
#include <cstdio>
#include <cstring>

#include "game_sdk.hpp"

#if defined(_WIN32)
#include <windows.h>
using LibraryHandle = HMODULE;
static LibraryHandle open_library(const char* path) { return LoadLibraryA(path); }
static void* find_symbol(LibraryHandle h, const char* name) {
    return reinterpret_cast<void*>(GetProcAddress(h, name));
}
static void close_library(LibraryHandle h) { FreeLibrary(h); }
static bool library_still_loaded(const char* path) { return GetModuleHandleA(path) != nullptr; }
#else
#include <dlfcn.h>
using LibraryHandle = void*;
static LibraryHandle open_library(const char* path) { return dlopen(path, RTLD_NOW | RTLD_LOCAL); }
static void* find_symbol(LibraryHandle h, const char* name) { return dlsym(h, name); }
static void close_library(LibraryHandle h) { dlclose(h); }
static bool library_still_loaded(const char* path) {
    void* h = dlopen(path, RTLD_NOW | RTLD_NOLOAD);
    if (h) dlclose(h);
    return h != nullptr;
}
#endif

namespace {

double query_hot(double lat, double lon) {
    if (lat < 40.98) return 0.0;  // sea: the height of the sea surface
    return 120.0 + 60.0 * std::sin((lat - 41.0) * 40.0) * std::cos((lon - 29.0) * 30.0);
}

struct Plugin {
    LibraryHandle handle = nullptr;
    flight_plugin_load_fn load = nullptr;
    flight_plugin_unload_fn unload = nullptr;
};

bool open_plugin(const char* path, Plugin& p) {
    p.handle = open_library(path);
    if (!p.handle) {
        std::fprintf(stderr, "cannot open %s\n", path);
        return false;
    }
    p.load = reinterpret_cast<flight_plugin_load_fn>(find_symbol(p.handle, "flight_plugin_load"));
    p.unload =
        reinterpret_cast<flight_plugin_unload_fn>(find_symbol(p.handle, "flight_plugin_unload"));
    return p.load && p.unload;
}

// Reads a component through the C API (works with every Flecs 4.x version).
template <typename T>
T read(flecs::world& world, const char* name) {
    const flecs::entity e = world.lookup(name);
    return *static_cast<const T*>(ecs_get_id(world, e, world.component<T>()));
}

void run(flecs::world& world, double seconds) {
    for (int i = 0; i < static_cast<int>(seconds * 60.0); ++i) world.progress(1.0f / 60.0f);
}

void print(flecs::world& world, const char* title) {
    std::printf("--- %s\n", title);
    world.each([](flecs::entity e, const game::Transform& t) {
        std::printf("  %-6s lat %9.5f lon %9.5f alt %7.1f | roll %6.1f pitch %5.1f yaw %6.1f\n",
                    e.name().c_str(), t.lat, t.lon, t.alt, t.roll, t.pitch, t.yaw);
    });
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <plugin> [--expect-reject]\n", argv[0]);
        return 2;
    }
    const char* path = argv[1];
    const bool expect_reject = argc > 2 && std::strcmp(argv[2], "--expect-reject") == 0;

    flecs::world world;
    world.component<game::Transform>();
    world.component<game::Hot>();
    world.component<game::FlightRequest>();
    world.system<const game::Transform, game::Hot>("HotSystem")
        .kind(flecs::PreUpdate)
        .each([](const game::Transform& t, game::Hot& hot) { hot.height = query_hot(t.lat, t.lon); });

    const double ground = query_hot(41.02, 28.95);
    world.entity("TK1")
        .set<game::Transform>({41.02, 28.95, ground, 0, 0, 90})
        .set<game::Hot>({ground})
        .set<game::FlightRequest>({1, 0.0, 41.10, 29.10, 1500.0});  // parked turboprop
    world.entity("UAV1")
        .set<game::Transform>({41.05, 29.05, 600.0, 0, 0, 180})
        .set<game::Hot>({query_hot(41.05, 29.05)})
        .set<game::FlightRequest>({3, 46.0});                       // flies straight south
    world.entity("Drone1")
        .set<game::Transform>({41.06, 29.06, 300.0, 0, 0, 0})
        .set<game::Hot>({query_hot(41.06, 29.06)})
        .set<game::FlightRequest>({5, 10.0, 41.065, 29.065, 300.0});  // multirotor go-to
    world.entity("ATAK1")
        .set<game::Transform>({41.08, 29.02, query_hot(41.08, 29.02), 0, 0, 45})
        .set<game::Hot>({query_hot(41.08, 29.02)})
        .set<game::FlightRequest>({10, 0.0, 41.12, 29.08, 600.0});  // parked T129, go-to
    world.entity("TCG1")
        .set<game::Transform>({40.95, 29.0, 0.0, 0, 0, 90})
        .set<game::Hot>({query_hot(40.95, 29.0)})
        .set<game::FlightRequest>({13, 8.0});                       // ship sailing east

    int failures = 0;
    for (int cycle = 0; cycle < 2; ++cycle) {
        Plugin plugin;
        if (!open_plugin(path, plugin)) return 1;
        const bool loaded = plugin.load(world);
        if (expect_reject) {
            close_library(plugin.handle);
            run(world, 1.0);  // the world must still be usable
            std::printf("plugin %s\n", loaded ? "LOADED (unexpected)" : "rejected as expected");
            return loaded ? 1 : 0;
        }
        if (!loaded) return 1;

        const game::Transform before = read<game::Transform>(world, "UAV1");
        const game::Transform ship_before = read<game::Transform>(world, "TCG1");
        run(world, 60.0);
        print(world, cycle == 0 ? "plugin loaded, 60 s" : "plugin reloaded, 60 s");
        const game::Transform after = read<game::Transform>(world, "UAV1");
        if (!(after.lat < before.lat - 0.005)) {
            std::printf("FAIL: UAV1 did not fly south\n");
            ++failures;
        }
        const game::Transform ship_after = read<game::Transform>(world, "TCG1");
        if (!(ship_after.lon > ship_before.lon + 0.004 && ship_after.alt == 0.0)) {
            std::printf("FAIL: TCG1 did not sail east on the sea surface\n");
            ++failures;
        }

        // The game teleports UAV1 by writing its Transform; the plugin continues from there.
        const game::Transform teleport{41.20, 29.20, 900.0, 0.0, 0.0, 180.0};
        world.entity("UAV1").set<game::Transform>(teleport);
        run(world, 20.0);
        const game::Transform moved = read<game::Transform>(world, "UAV1");
        if (!(moved.lat < teleport.lat - 0.002 && std::abs(moved.lon - teleport.lon) < 1e-3 &&
              std::abs(moved.alt - teleport.alt) < 20.0)) {
            std::printf("FAIL: UAV1 did not continue from where the game put it\n");
            ++failures;
        }
        print(world, "20 s after the game teleported UAV1");

        plugin.unload(world);
        close_library(plugin.handle);
        std::printf("plugin unloaded (%s)\n",
                    library_still_loaded(path) ? "library still mapped" : "library unmapped");
        if (world.lookup("fm::ecs::FlightModule")) {
            std::printf("FAIL: module entity left behind\n");
            ++failures;
        }

        // The game keeps running without the plugin; aircraft keep their last Transform.
        const game::Transform frozen = read<game::Transform>(world, "UAV1");
        run(world, 5.0);
        const game::Transform still = read<game::Transform>(world, "UAV1");
        if (still.lat != frozen.lat || still.lon != frozen.lon) {
            std::printf("FAIL: something still moves the aircraft after unload\n");
            ++failures;
        }
        print(world, "5 s after unload");
    }
    std::printf("%s\n", failures == 0 ? "OK" : "FAILED");
    return failures == 0 ? 0 : 1;
}
