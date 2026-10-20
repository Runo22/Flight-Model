// Helicopters and drones landing on and taking off from a moving ship deck.
#include <cmath>

#include "fm/flight_agent.hpp"
#include "fm/presets.hpp"
#include "harness.hpp"

using namespace fm;

namespace {
constexpr double kDt = 1.0 / 60.0;

InitialConditions airborne(const Vec3& p, double heading_deg, double speed) {
    InitialConditions ic;
    ic.position = p;
    ic.heading = deg2rad(heading_deg);
    ic.speed = speed;
    return ic;
}

// The spot's world position now.
Vec3 spot_position(const FlightAgent& ship, const DeckSpot& spot) {
    const FlightState& s = ship.state();
    const Vec2 fwd = heading_vector(s.attitude.heading);
    const Vec2 r = right_of(fwd) * spot.offset.x + fwd * spot.offset.y;
    return {s.position.x + r.x, s.position.y + r.y, s.position.z + spot.offset.z};
}

// One frame like the Flecs module: the carrier's state from the start of the frame, then
// both step. The game's height of terrain at sea is the sea surface.
void step(FlightAgent& ship, FlightAgent& aircraft) {
    aircraft.set_leader_state(ship.as_leader());
    ship.step(kDt, {});
    Environment sea;
    sea.terrain_elevation = 0.0;
    aircraft.step(kDt, sea);
}

template <typename F>
void run(FlightAgent& ship, FlightAgent& aircraft, double seconds, F&& on_step) {
    for (int i = 0; i < static_cast<int>(seconds / kDt); ++i) {
        step(ship, aircraft);
        on_step(i * kDt);
    }
}
void run(FlightAgent& ship, FlightAgent& aircraft, double seconds) {
    run(ship, aircraft, seconds, [](double) {});
}

FlightAgent sailing_carrier(double speed) {
    FlightAgent ship(presets::carrier_ship(), airborne({0, 0, 0}, 0.0, speed));
    ship.hold(0.0, 0.0, speed);
    return ship;
}
}  // namespace

TEST_CASE("deck: helicopter lands on a sailing carrier, rides through a turn, takes off") {
    FlightAgent ship = sailing_carrier(8.0);
    const DeckSpot spot = presets::carrier_ship().deck_spots[2];
    const RotorcraftParams p = presets::attack_helicopter();
    FlightAgent heli(p, airborne({-2000, -3000, 300}, 0.0, 40.0));
    CHECK(heli.land_on(spot));
    CHECK(heli.mode() == FlightMode::Land);
    CHECK(heli.tracks_leader());
    step(ship, heli);
    CHECK(heli.state().phase == FlightPhase::Approach);

    double touchdown_sink = 0.0;
    bool landed = false;
    run(ship, heli, 300.0, [&](double) {
        if (!landed && heli.state().on_ground) {
            landed = true;
            touchdown_sink = -heli.state().vertical_speed;
        }
    });
    CHECK(landed);
    CHECK(heli.state().phase == FlightPhase::Landed);
    CHECK(touchdown_sink < 1.5);
    CHECK(heli.rotorcraft()->on_deck());
    const Vec3 on_spot = spot_position(ship, spot);
    CHECK((heli.state().position.xy() - on_spot.xy()).length() < 0.05);
    CHECK_NEAR(heli.state().position.z, on_spot.z + p.gear_height, 0.01);  // on the deck
    CHECK((heli.state().velocity - ship.state().velocity).length() < 0.01);

    // The carrier turns east: the helicopter stays on its spot and turns with it.
    ship.hold(deg2rad(90.0), 0.0, 8.0);
    double max_error = 0.0;
    double max_heading_error = 0.0;
    run(ship, heli, 300.0, [&](double) {
        max_error = std::max(max_error,
                             (heli.state().position.xy() - spot_position(ship, spot).xy()).length());
        max_heading_error = std::max(max_heading_error,
                                     std::abs(wrap_pi(heli.state().attitude.heading -
                                                      ship.state().attitude.heading)));
    });
    CHECK(max_error < 0.05);
    CHECK(max_heading_error < deg2rad(0.5));
    CHECK(heli.state().phase == FlightPhase::Landed);

    // Takeoff with the deck's velocity: straight up from the spot.
    CHECK(heli.takeoff({}));
    double max_drift = 0.0;
    run(ship, heli, 20.0, [&](double) {
        if (heli.mode() == FlightMode::Takeoff)
            max_drift = std::max(max_drift, (heli.state().position.xy() -
                                             spot_position(ship, spot).xy()).length());
    });
    CHECK(max_drift < 2.0);
    CHECK(!heli.state().on_ground);
    CHECK(heli.mode() == FlightMode::Hover);  // clear of the deck, hovering
    CHECK(!heli.rotorcraft()->deck_operation());
    CHECK(!heli.tracks_leader());
    CHECK(heli.state().position.z > on_spot.z + p.takeoff_height);
}

TEST_CASE("deck: heavy helicopter and drone land on different spots") {
    const ShipParams carrier = presets::carrier_ship();
    FlightAgent ship = sailing_carrier(6.0);
    FlightAgent chinook(presets::transport_helicopter(), airborne({1500, 2500, 400}, 180.0, 50.0));
    FlightAgent drone(presets::multirotor(), airborne({-200, -300, 60}, 0.0, 0.0));
    CHECK(chinook.land_on(carrier.deck_spots[4]));
    CHECK(drone.land_on(carrier.deck_spots[0]));
    for (int i = 0; i < static_cast<int>(400.0 / kDt); ++i) {
        chinook.set_leader_state(ship.as_leader());
        drone.set_leader_state(ship.as_leader());
        ship.step(kDt, {});
        Environment sea;
        sea.terrain_elevation = 0.0;
        chinook.step(kDt, sea);
        drone.step(kDt, sea);
    }
    for (const auto& [aircraft, spot] :
         {std::pair{&chinook, carrier.deck_spots[4]}, std::pair{&drone, carrier.deck_spots[0]}}) {
        CHECK(aircraft->state().phase == FlightPhase::Landed);
        CHECK((aircraft->state().position.xy() - spot_position(ship, spot).xy()).length() < 0.05);
    }
}

TEST_CASE("deck: a command given on the deck takes off from the moving deck first") {
    FlightAgent ship = sailing_carrier(8.0);
    const DeckSpot spot = presets::carrier_ship().deck_spots[1];
    FlightAgent heli(presets::attack_helicopter(), airborne({0, -400, 60}, 0.0, 0.0));
    CHECK(heli.land_on(spot));
    run(ship, heli, 200.0);
    CHECK(heli.rotorcraft()->on_deck());
    CHECK(!heli.land_on(spot));  // already on the deck
    heli.go_to({-3000.0, 5000.0, 300.0}, {.on_arrival = EndBehavior::Loiter});  // hover there
    CHECK(heli.mode() == FlightMode::Takeoff);
    double climb_drift = 0.0;
    run(ship, heli, 240.0, [&](double) {
        if (heli.mode() == FlightMode::Takeoff)
            climb_drift = std::max(climb_drift, (heli.state().position.xy() -
                                                 spot_position(ship, spot).xy()).length());
    });
    CHECK(climb_drift < 2.0);
    CHECK((heli.state().position.xy() - Vec2{-3000.0, 5000.0}).length() < 5.0);
}

TEST_CASE("deck: losing the carrier, being moved and vehicles that cannot land on a deck") {
    const DeckSpot spot = presets::carrier_ship().deck_spots[2];
    // No carrier updates during the approach: hovers where it is.
    {
        FlightAgent heli(presets::attack_helicopter(), airborne({0, 0, 200}, 0.0, 30.0));
        CHECK(heli.land_on(spot));
        heli.set_leader_state(sailing_carrier(8.0).as_leader());
        for (int i = 0; i < static_cast<int>(10.0 / kDt); ++i) heli.step(kDt, {});
        CHECK(heli.mode() == FlightMode::Hover);
        CHECK(!heli.tracks_leader());
    }
    // On the deck, moved by the game to land: off the carrier, parked there.
    {
        FlightAgent ship = sailing_carrier(8.0);
        FlightAgent heli(presets::multirotor(), airborne({-100, -150, 40}, 0.0, 0.0));
        CHECK(heli.land_on(spot));
        run(ship, heli, 200.0);
        CHECK(heli.rotorcraft()->on_deck());
        CHECK(heli.relocate({5000.0, 0.0, 120.0}));
        for (int i = 0; i < 120; ++i) {
            Environment land;
            land.terrain_elevation = 120.0;
            heli.step(kDt, land);
        }
        CHECK(!heli.rotorcraft()->deck_operation());
        CHECK(heli.state().on_ground);
        CHECK_NEAR(heli.state().position.x, 5000.0, 0.1);
        CHECK(heli.state().ground_speed < 0.01);
    }
    // Only rotorcraft land on decks; the spot must be usable.
    FlightAgent plane(presets::light_propeller(), airborne({0, 0, 500}, 0.0, 50.0));
    CHECK(!plane.land_on(spot));
    FlightAgent ship = sailing_carrier(8.0);
    CHECK(!ship.land_on(spot));
    FlightAgent heli(presets::multirotor(), airborne({0, 0, 50}, 0.0, 0.0));
    CHECK(!heli.land_on(DeckSpot{{std::nan(""), 0.0, 20.0}, 0.0}));
    CHECK(heli.mode() == FlightMode::Hover);
}
