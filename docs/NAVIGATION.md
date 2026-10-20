# Navigasyon komutları: noktaya git, loiter, rota

Bu belgedeki her kod parçası [`examples/navigation.cpp`](../examples/navigation.cpp)
programından alındı. Program hepsini aynı enlem/boylamlı Flecs dünyasında uçurur, sonuçları
kendisi kontrol eder ve test olarak çalışır:

```sh
./build/examples/fm_navigation      # ya da: ctest -R fm_navigation_example
```

Örnekler, oyunun kendi `game::Transform { lat, lon, alt, roll, pitch, yaw }` component'ini
`fm::ecs::bind_geo_transform<game::Transform>(world)` ile bağladığı kurulumu varsayar.

## 0. Temel kurallar

* **Komutlar yerel koordinat alır.** Enlem/boylam/irtifa `fm::ecs::to_local(world, {lat, lon, alt})`
  ile çevrilir (z = irtifa). Gerçek kuzeye göre bir yön (derece) yerel yöne
  `fm::ecs::local_heading(world, derece, nokta)` ile çevrilir.
* **Komutu kim, nerede verir:**
  * `world.progress()` dışından, ya da tek iş parçacıklı bir sistem/observer içinden
    `fm::ecs::agent(e)->...` ile.
  * Agent'ı aynı frame'de bir sistem içinde oluşturuyorsan komutu `attach_geo`'nun setup
    callback'inde ver (component o frame'in sonunda görünür):

    ```cpp
    fm::ecs::attach_geo(e, params, {lat, lon, alt}, yaw, 0.0, true,
                        [&](fm::FlightAgent& a) { a.go_to(target, opt); });
    ```
  * `agent(e)`'nin döndürdüğü pointer'ı saklama; komutu hemen ver.
* **Geçersiz komut** (NaN/sonsuz koordinat) yok sayılır, aracın durumu bozulmaz.

## 1. Bir noktaya git

```cpp
fm::GoToOptions opt;
opt.speed = 120.0;                         // m/s; leave it out to keep the current speed
opt.on_arrival = fm::EndBehavior::Loiter;  // aircraft orbit, drones hover over it
fm::ecs::agent(e)->go_to(fm::ecs::to_local(world, {41.20, 29.20, 2500.0}), opt);
```

| `GoToOptions` alanı | Değerler |
|---|---|
| `on_arrival` | `ContinueStraight` (varsayılan): üstünden geçip düz devam. `Loiter`: uçak noktanın etrafında tur atar, drone üstünde bekler. `Land`: drone oraya iner (uçak pist ister, bkz. [Bölüm 4](#4-kalkış-ve-iniş)). |
| `altitude_ref` | `Absolute` (varsayılan): deniz seviyesinden metre. `AboveTerrain`: z araziden yükseklik (arazi takibi). `Keep`: şu anki irtifa korunur, z yok sayılır. |
| `speed` | m/s gerçek hava hızı; NaN (varsayılan) = mevcut hız hedefi |
| `vertical` | `Linear` (varsayılan): irtifa yol boyunca doğrusal değişir. `Immediate`: hemen tırmanır/alçalır. |

Park halindeki bir uçağa `go_to` (ya da rota, loiter) verilirse **önce kendisi kalkar**:

```cpp
fm::ecs::agent(light)->go_to(fm::ecs::to_local(world, {41.06, 29.00, 800.0}),
                             {.on_arrival = fm::EndBehavior::Loiter});
```

## 2. Loiter ve bekleme

```cpp
fm::FlightAgent* a = fm::ecs::agent(e);
// Orbit the point with a 3 km radius, clockwise
a->loiter(fm::ecs::to_local(world, {41.15, 29.05, 1500.0}), 3000.0, true);
// Radius 0: aircraft derive it from their turn performance; a drone hovers over the point
a->loiter(fm::ecs::to_local(world, {41.0505, 29.0505, 60.0}), 0.0, true,
          fm::AltitudeRef::AboveTerrain);  // 60 m above the terrain
// Hold where it is now (aircraft orbit the current position)
a->hover();
```

* Uçakta yarıçap 0 ise dönüş performansından hesaplanır (normal yatış açısıyla rahat bir tur).
* Drone `loiter(merkez, r)` ile `r` yarıçaplı tur atar; dar turlarda (40 m) birkaç metre
  dışarıda kalabilir.
* `hover()` "şimdi bulunduğun yerde bekle" demektir; oyun aracı taşırsa bekleme noktası da
  taşınır. Belirli bir noktayla verilen loiter ise o noktada kalır.

## 3. Rota (waypoint'ler)

```cpp
fm::Waypoint wp(flecs::world& world, double lat, double lon, double alt,
                fm::AltitudeRef ref = fm::AltitudeRef::Absolute) {
    fm::Waypoint w;
    w.position = fm::ecs::to_local(world, {lat, lon, alt});  // z = alt
    w.altitude_ref = ref;
    return w;
}

fm::Route r;
r.waypoints = {wp(world, 41.10, 29.10, 3000.0), wp(world, 41.30, 29.30, 2000.0),
               wp(world, 41.30, 29.60, 500.0, fm::AltitudeRef::AboveTerrain)};
r.waypoints[1].fly_over = true;  // pass right over it instead of cutting the corner
r.waypoints[1].speed = 180.0;    // m/s from this waypoint on
r.on_end = fm::EndBehavior::Loiter;
r.loiter_radius = 4000.0;        // 0 = from turn performance
fm::ecs::agent(jet)->follow_route(r);
```

| `Waypoint` alanı | Anlamı |
|---|---|
| `position` | Yerel konum; z `altitude_ref`'e göre okunur |
| `altitude_ref` | `Absolute`, `AboveTerrain` (araziden yükseklik) ya da `Keep` (irtifayı değiştirme) |
| `speed` | Bu waypoint'ten itibaren hız (m/s); NaN = değiştirme |
| `fly_over` | `false` (varsayılan): köşeyi keserek dön (fly-by). `true`: tam üstünden geç |
| `hold_time` | Drone: burada bu kadar saniye bekle |

| `Route` alanı | Anlamı |
|---|---|
| `loop` | `true`: son waypoint'ten sonra başa dön (devriye) |
| `on_end` | `ContinueStraight` (son bacak yönünde düz devam), `Loiter`, `Land` |
| `loiter_radius` | `Loiter` bitişinde tur yarıçapı (m), 0 = performanstan |
| `vertical` | `Linear` (varsayılan) ya da `Immediate` |
| `runway` | Uçak için `Land` bitişinde inilecek pist |

Devriye ve drone ile tarama:

```cpp
fm::Route patrol;
patrol.waypoints = {wp(world, 41.01, 29.20, 600.0), wp(world, 41.01, 29.22, 600.0),
                    wp(world, 41.00, 29.22, 600.0)};
patrol.loop = true;  // after the last waypoint, back to the first

fm::Route survey;
survey.waypoints = {wp(world, 41.0710, 29.0700, 40.0, fm::AltitudeRef::AboveTerrain),
                    wp(world, 41.0710, 29.0715, 40.0, fm::AltitudeRef::AboveTerrain)};
survey.waypoints[0].hold_time = 10.0;   // hover 10 s here
survey.on_end = fm::EndBehavior::Land;  // land at the last waypoint
```

## 4. Kalkış ve iniş

Pistten kalkış, rota ve aynı piste dönüş:

```cpp
const fm::GeoPoint threshold{40.95, 29.30, hot};  // runway start, altitude = ground
const double runway_heading = fm::ecs::local_heading(world, 90.0, threshold);  // true 090

fm::Route r;
r.waypoints = {wp(world, 40.95, 29.36, 600.0), wp(world, 40.99, 29.36, 600.0)};
r.on_end = fm::EndBehavior::Land;
r.runway = fm::Runway{fm::ecs::to_local(world, threshold), runway_heading};

fm::TakeoffPlan plan;
plan.runway = fm::Runway{fm::ecs::to_local(world, threshold), runway_heading};
plan.climb_height = 300.0;  // m above the runway where the takeoff ends
plan.then = r;
fm::ecs::agent(e)->takeoff(plan);
```

* `takeoff({})`: bulunduğu yerden, burnunun baktığı yöne kalkar.
* Doğrudan iniş: `agent->land(fm::Runway{baslangic, yon})`. Uçak piste eksen uzantısından
  yaklaşır, 3° süzülüş yolunu izler, flare yapar, frenler; dengesiz yaklaşmada pas geçip
  yeniden dener. Drone `land()` ile bulunduğu yere iner.
* `Runway::start` iniş için pist başıdır (threshold); `heading` iniş yönüdür.

## 5. Düz uçuş: yön, irtifa, hız

```cpp
const double course = fm::ecs::local_heading(world, 45.0, {t.lat, t.lon, t.alt});
fm::ecs::agent(e)->hold(course, 7000.0, 230.0);  // true course 045, 7000 m, 230 m/s
fm::ecs::agent(e)->set_speed(200.0);             // only the speed
fm::ecs::agent(e)->cruise();                     // keep going straight as now
```

* `hold` komutun verildiği noktadan geçen çizgiyi izler: araç o çizgiye döner (uçak biraz
  taşıp oturabilir) ve rüzgârda sürüklenmez.
* Çizgi düzdür: 200 km sonra gerçek kuzeye göre rota ~1° kayar (büyük daire gibi).
* Hiç komut yoksa araç düz devam eder (`cruise`); drone durgunsa olduğu yerde bekler.

## 6. Komutu oyundan bir component ile vermek

```cpp
struct FlyTo {  // game component
    double lat = 0.0;
    double lon = 0.0;
    double alt = 0.0;
    bool loiter = true;
};

// Only entities that already have an agent match, so a command given before the agent
// exists waits for it.
world.system<const game::FlyTo, fm::ecs::FlightController>("FlyToCommand")
    .kind(flecs::PreUpdate)
    .each([](flecs::entity e, const game::FlyTo& c, fm::ecs::FlightController& fc) {
        fm::GoToOptions opt;
        opt.on_arrival =
            c.loiter ? fm::EndBehavior::Loiter : fm::EndBehavior::ContinueStraight;
        flecs::world w = e.world();
        fc.agent->go_to(fm::ecs::to_local(w, {c.lat, c.lon, c.alt}), opt);
        e.remove<game::FlyTo>();  // command done
    });

// Game side:
entity.set<game::FlyTo>({41.0910, 29.0910, 200.0, true});
```

* Plugin'de bu sistemi modül scope'unda oluştur (`register_systems` içinde); `unload()`
  onu da kaldırır.
* `event(flecs::OnSet)` ile bir observer da olur, ama komut agent oluşmadan gelirse kaybolur.
  Sistem hali komutu agent oluşana kadar bekletir.

## 7. Gemi

```cpp
flecs::entity carrier = spawn(world, "Carrier", fm::presets::carrier_ship(),
                              {40.80, 29.00, 0.0, 0, 0, 90.0}, 8.0, false);
fm::Route r;
r.waypoints = {wp(world, 40.80, 29.03, 0.0), wp(world, 40.785, 29.03, 0.0)};
r.on_end = fm::EndBehavior::Land;  // ships: stop and keep station there
fm::ecs::agent(carrier)->follow_route(r);
```

* Gemi hep su yüzeyinde kalır (`water_level`, varsayılan 0). Komutlardaki ve
  `game::Transform`'daki irtifa yok sayılır, `AltitudeRef` önemsizdir.
* Durmak için: `go_to` ya da rota sonunda `Land`/`Loiter` (yarıçapsız), ya da `hover()`.
  Gemi planlı yavaşlar, noktayı geçerse geri gelir, nokta yandaysa pod'larla yerinde döner.
  `set_speed(0)` yalnızca durur; negatif hız tornistandır.
* `loiter(merkez, r)` en az dönüş yarıçapıyla (preset'te 300 m) tur atar; `r = 0` orada
  istasyon tutar.
* Tam dümen ya da sabit bir dönüş hızı için `agent->ship()->steer(rad_s)`
  (`+inf` = sancağa tam dümen). Başka bir komut gelene kadar sürer.
* HOT `water_level + 0.5` m'yi geçerse gemi karaya oturur (`phase = TerrainContact`) ve oyun
  onu taşıyana kadar durur.
* Kol (eskort / konvoy) `join_formation` ile aynı şekilde kurulur; lider durursa gemi slotunda
  istasyon tutar.

## 8. İlerlemeyi takip etmek

| Ne | Nereden |
|---|---|
| Ne yapıyor | `agent->mode()` ya da `FlightPose::mode`: `GoTo`/`Route` varınca `Loiter`, `Hover`, `Cruise` ya da `Land` olur |
| Kalkış/iniş aşaması | `FlightPose::phase`: `TakeoffRoll`, `InitialClimb`, `Approach`, `Flare`, `Rollout`, `Landed` ... Gemide `Underway` (seyirde), `Parked` (durdu), `TerrainContact` (karaya oturdu) |
| Yerde mi | `FlightPose::on_ground` |
| Rotada kaçıncı waypoint | `agent->fixed_wing()->route().index()` / `agent->rotorcraft()->route().index()` / `agent->ship()->route().index()` |
| Araziden yükseklik | `agent->state().height_above_terrain` |

## 9. Hangi araç ne yapar

| Komut | Sabit kanat | Helikopter / drone | Gemi |
|---|---|---|---|
| `go_to` | Noktaya uçar; varınca düz devam ya da tur (`Land` için pist gerekir, yoksa tur atar) | Noktaya uçar; varınca düz devam, üstünde bekleme (yavaşlayıp durur) ya da iniş | Noktaya gider; varınca düz devam, ya da (`Loiter`/`Land`) yavaşlayıp durur ve orada istasyon tutar |
| `loiter(merkez, r)` | `r` yarıçaplı tur (0 = performanstan) | `r` yarıçaplı tur; `r = 0` üstünde bekleme | `r` yarıçaplı tur (en az dönüş yarıçapı); `r = 0` orada istasyon tutma |
| `hover()` | Bulunduğu yerin etrafında tur | Bulunduğu yerde bekleme | Durur, bulunduğu yerde istasyon tutar |
| `follow_route` | Fly-by / fly-over, waypoint hızları | Aynısı, ayrıca `hold_time`. Drone keskin köşede yavaşlar; helikopter köşeyi hızını koruyarak yatışlı bir yayla alır | Fly-by, waypoint hızları; dümen gecikmesi için dönüşe erken başlar (`hold_time` yok) |
| `takeoff` | Pistten: koşu, rotasyon, tırmanış | Dikey kalkış | Yok (`false`) |
| `land(runway)` | Pist yaklaşması, flare, frenleme | Pist başına uçup dikey iniş | Yok (`false`) |
| `land()` | Pist olmadan inemez (`false`) | Bulunduğu yere dikey iniş | Yok (`false`); durmak için `hover()` |