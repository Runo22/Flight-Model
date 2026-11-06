// Live flight demos around Ankara for the resim plugin. A scenario spawns its aircraft, gives
// the first commands and registers its logic (helpers.hpp: every_frame), which follows the
// flight at normal speed in the game's own frames: later commands (split, landing, taxi
// back), milestones in the log. Running a scenario again restarts it.
#pragma once

namespace demo
{

    struct Scenario
    {
        const char* name;
        const char* title;
        void (*run)();
    };

#pragma region Functions

    // An Anka-3 parked on Akinci's runway threshold flies a route that ends with a landing back
    // on the same runway: takeoff, the route, the extended centerline, glide path, flare,
    // touchdown and braking.
    void anka3_tour();

    // A Kaan with two Anka-3 in vic flies to Akinci at 320 knots and orbits it; a Hurkus orbits
    // over Golbasi.
    void formation_goto();

    // A Hurjet orbits over northern Syria (far from the Ankara origin: the projection is still
    // good to ~0.5% there).
    void syria_orbit();

    // Scattered west of Ankara, a Hurjet, an Anka-3 and a Kizilelma form up (the leader orbits
    // until the UAVs have joined), fly to Mogan lake as a formation and orbit there. Then they
    // split: Kizilelma stays in an orbit over the lake, the Hurjet and the Anka-3 recover to
    // Akinci with an overhead break and land one after the other.
    void rejoin();

    // A Hurkus on the apron west of Akinci's runway: taxi out, takeoff, a short tour north to
    // Kahramankazan and back over Etimesgut, an overhead break, the roll to a stop point and
    // the taxi back to its parking spot.
    void circuit();

    // An Anka tour from Akinci: Kahramankazan (corner cut), on at 110 KCAS, terrain following
    // 400 m above Elmadag, Mogan lake flown over, landing at Akinci. Route turns at its 25 deg
    // navigation bank.
    void waypoints();

    // Republic Day: four F-16s taxi out of their shelters, take off 10 s apart and rejoin in a
    // diamond, pass over Anitkabir at 300 knots, then recover to Akinci with an overhead break.
    void flypast();

    // Four aircraft converge on Kizilay at 2,500 m from the four directions, all arriving at
    // the same time: first without collision avoidance (near misses), then the same encounter
    // with it (vertical separation). Collision avoidance is a world setting: other scenarios
    // running meanwhile fly without it during the first pass.
    void traffic();

    // A T129 lifts off at Guvercinlik and flies 40 m above the terrain past Eymir lake to
    // Elmadag, hovers a minute over the summit and lands on a ridge east of it.
    void lowlevel();

    // Landings ordered from high above the field: an A400M 2,500 m over Esenboga and an
    // Aksungur 2,000 m over Akinci. Each first descends in an orbit outside the final approach
    // fix, then flies a stabilised straight-in approach.
    void recovery();

    // Station relief: an Anka orbits a station west of Ankara. An Aksungur takes off at Akinci,
    // flies there and takes over the orbit 300 m higher; once both have orbited together for
    // four minutes, the Anka leaves and lands back at Akinci.
    void relief();

#pragma endregion

    inline constexpr Scenario demo_scenarios[] = {
        { "anka3_tour", "Anka-3: takeoff at Akinci, route, landing back", anka3_tour },
        { "formation_goto", "Kaan and two Anka-3 in vic to Akinci; Hurkus orbits Golbasi", formation_goto },
        { "syria_orbit", "Hurjet orbit over northern Syria", syria_orbit },
        { "rejoin", "Rejoin over Mogan: Kizilelma stays, Hurjet and Anka-3 land at Akinci", rejoin },
        { "circuit", "Hurkus: taxi out, takeoff, tour, overhead break, taxi back", circuit },
        { "waypoints", "Anka route: corner cuts, 110 KCAS, terrain following, landing", waypoints },
        { "flypast", "Four F-16s: formation takeoff, diamond over Anitkabir, overhead recovery", flypast },
        { "traffic", "Four aircraft meet over Kizilay: without, then with collision avoidance", traffic },
        { "lowlevel", "T129 low level to Elmadag, hover over the summit, landing on a ridge", lowlevel },
        { "recovery", "Landings from high overhead: A400M at Esenboga, Aksungur at Akinci", recovery },
        { "relief", "Station relief: Aksungur takes over the orbit, Anka lands at Akinci", relief },
    };

    // Runs the scenario called `name`; false when there is none.
    bool run_scenario(const char* name);

} // namespace demo
