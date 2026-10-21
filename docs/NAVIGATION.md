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
| `speed` | Hız; NaN (varsayılan) = mevcut hız hedefi |
| `speed_ref` | `True` (varsayılan): gerçek hava hızı, m/s. `Calibrated`: kalibre hava hızı (gösterge, IAS/CAS), m/s. `Mach`: Mach sayısı. Sabit kanatlı CAS ve Mach hedeflerini irtifa değişirken korur |
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
| `speed`, `speed_ref` | Bu waypoint'ten itibaren hız; NaN = değiştirme. `speed_ref` ile TAS, CAS ya da Mach |
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
patrol.waypoints = {wp(world, 41.02, 29.20, 600.0), wp(world, 41.02, 29.24, 600.0),
                    wp(world, 41.00, 29.24, 600.0)};
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
* `Runway` verildiğinde kalkış pist ekseninde yapılır: uçak pistin üzerindeyse (eksene 30 m'den
  yakın) bulunduğu yerden eksene hizalanıp koşuya başlar; değilse önce `start` noktasına
  taksi yapar.

### Hangardan taksi ve yön ile kalkış

Uçak hangardan taksi yolu boyunca piste gelir, pist eksenine hizalanır (`LineUp`) ve kalkar.
Taksi noktalarının `z` değeri kullanılmaz; uçak HOT ile yerde kalır:

```cpp
// Points around the runway: `along` the centerline from the threshold, `right` of it.
const auto airfield = [&](double along, double right) { /* runway_start + ... */ };

fm::TakeoffPlan plan;
plan.taxi = {airfield(-80.0, 60.0), airfield(0.0, 60.0)};  // out, along the taxiway
plan.runway = fm::Runway{runway_start, runway_heading};     // then lined up from here
plan.climb_height = 600.0;
fm::ecs::agent(viper)->takeoff(plan);
```

* **Yalnızca yön:** `plan.heading = yon;` (pist yok). Uçak bulunduğu yerden o yöne döner,
  hizalanır ve kalkar. Yön de verilmezse son taksi bölümünün yönü, o da yoksa burnun yönü.
* **Hizalanma:** Burun yönü 3°'den, eksenden uzaklık 3 m'den fazlaysa uçak önce taksi hızında
  döner ve eksene oturur; tam güç ondan sonra verilir. Zaten hizalıysa koşu hemen başlar.
* **Taksi:** Düzde `taxi_speed`, keskin köşelerden önce frenleyip yavaşlar (yanal ivme
  ~1.5 m/s²), burun tekerleği `taxi_turn_radius`'tan dar dönemez; uçak yerinde dönemez.
  Geri itme (pushback) yok: hangardan burnu dışarı bakarak çıkar.
* Taksi sırasında `mode` = `Takeoff`, `phase` = `Taxi`, piste dönerken `LineUp`.

### İnişten sonra park yerine taksi

```cpp
// After the landing roll the shuttle taxis to its parking spot, nose south.
if (pose(shuttle).phase == fm::FlightPhase::Landed) {
    const std::vector<fm::Vec3> to_apron{airfield(150.0, 60.0), airfield(150.0, 100.0)};
    fm::ecs::agent(shuttle)->taxi(to_apron, runway_heading + fm::kPi / 2.0);
}
```

* `taxi(yol, son_yon)` yerdeki her uçakta çalışır (park hâlinde, iniş koşusunda ya da kalkış
  koşusunu iptal edip). Son noktada durur (`phase` = `Parked`, `mode` = `Cruise`).
* `son_yon` verilirse uçak son noktaya bu yönde gelir: son bölüm bu yöne bakmıyorsa önüne
  `6 x taxi_turn_radius` uzunluğunda düz bir giriş eklenir (hangara burun önde girmek için).
* Yolda oyun uçağı taşırsa (teleport), en yakın bölümden devam eder.
* **Henüz yok:** Yerde çarpışma önleme (uçaklar taksi yolunda birbirini beklemez).

### İniş

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
* Hız gösterge hızı ya da Mach olarak da verilebilir:
  `hold(course, 10000.0, 0.78, fm::AltitudeRef::Absolute, fm::SpeedRef::Mach)`,
  `set_speed(fm::knots(250.0), fm::SpeedRef::Calibrated)`. Sabit kanatlı bu hedefi
  tırmanırken ve alçalırken de tutar.
* **Tırmanış kipi (sabit kanat):** bazı tırmanışlarda komut edilen hız, istenen tırmanma
  oranını veremez. Tırmanış 150 m'den büyükse uçak bu durumda tam güçle en iyi tırmanma
  hızında tırmanır ve hedefe 30 m kala komut edilen hıza döner. `go_to`, rota, `hold` ve
  `loiter` için geçerlidir. `FixedWingParams::climb_speed` ile sabit bir tırmanma hızı (EAS)
  verilebilir; `< 0` kipi kapatır.

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

### Gemiye iniş ve gemiden kalkış (helikopter, drone)

```cpp
flecs::entity seahawk = spawn(world, "DeckHeli", fm::presets::attack_helicopter(),
                              {40.78, 28.98, 300.0, 0, 0, 0}, 0.0, false);
const fm::DeckSpot spot = fm::ecs::agent(carrier)->ship()->params().deck_spots[1];
fm::ecs::land_on(seahawk, carrier, spot);
```

* İniş sırası:
  1. Spotun 60 m arkasındaki, güvertenin ~15 m üstündeki bir kapıya gelir. Kapı gemiyle
     birlikte hareket eder; helikopter oraya seyir hızıyla gider.
  2. Gemiyle aynı hıza gelir, güverteye göre 3 m/s ile spotun üstüne geçer.
  3. Spotun üstünde (1 m içinde) alçalır ve ~1 m/s ile teker koyar. Gemi döner ya da hızlanır
     da spottan kayarsa, tekrar üstüne gelene kadar alçalmayı durdurur.
  4. Güvertede gemiyle birlikte hareket eder ve döner (`phase = Landed`,
     `rotorcraft()->on_deck()`).
* `takeoff()` güverteden geminin hızıyla dikey kalkar ve `takeoff_height` kadar yükselince
  gemiden ayrılır (sonra `hover` ya da `plan.then`). Güvertedeyken verilen `go_to` /
  `follow_route` / `loiter` / `hover` önce güverteden kalkar.
* `deck_spots` gemi frame'indedir: x sancak (sağ), y pruva (ileri), z su hattından güverte
  yüksekliği. Preset'te 6 spot vardır. Kendi `DeckSpot`'unu da verebilirsin (ör. bir
  fırkateynin helikopter pisti).
* Taşıyıcı yalnızca simüle edilen bir gemi olmak zorunda değil: `TrackedPose` ya da
  `TrackedGeoPose` ile oyunun kendisinin hareket ettirdiği bir gemi de olur. Modül gemi
  durumunu her frame helikoptere verir; Flecs dışında `set_leader_state(ship.as_leader())`
  her adımda çağrılır.
* Gemi silinirse ya da durumu gelmezse (2 s): yaklaşan helikopter olduğu yerde bekler,
  güvertedeki olduğu yerde kalır. Oyun güvertedeki helikopteri başka yere taşırsa gemiden
  ayrılır.
* Sabit kanatlı İHA'lar (TB3 sınıfı, ski-jump) henüz gemiye inip kalkamaz; bkz.
  [`ROADMAP.md`](ROADMAP.md). `land_on` onlar için `false` döner.

## 8. Kol uçuşu ve çarpışma önleme

* **Kol:** `fm::ecs::follow(takipci, lider, slot)` takipçiyi liderin frame'indeki bir slota
  bağlar: x sağ, y ileri, z yukarı. Hazır dizilimler `fm::formation::echelon_right`, `vic`,
  `line_abreast` ve `trail`'dir.
* **Liderin sınırı:** Sabit kanatlı lider, en geniş takipçisi izleyebilsin diye yatışını ve
  hızını sınırlar. Bunu modül her frame otomatik yapar. Geniş açıklıklı savaş uçağı kolları
  da sert dönüşlerde dağılmaz.
* **Çarpışma önleme:** Modül her frame havadaki ajanları birbirine trafik olarak verir.
  Sabit kanatlı, 30 s içinde 300 m yatay / 100 m dikeyden yakın bir geçiş öngörürse TCAS
  gibi tırmanır ya da alçalır. İki uçak her zaman zıt yön seçer; çatışma bitince göreve
  döner.
* **Devre dışı durumlar:** Yerde, kalkış/inişte ve yere 150 m'den yakınken çalışmaz.
  Alçalma arazi tabanını delecekse tırmanış seçilir. Aynı kolun üyeleri birbirinden kaçmaz.
* **Ayarlar:** `FixedWingParams::avoidance_radius`, `avoidance_height` ve
  `avoidance_lookahead` ile ayarlanır. `fm::ecs::set_collision_avoidance(world, false)`
  kapatır. Kaçınma sürerken `state().avoiding_traffic` `true` olur. Flecs dışında
  `agent.set_traffic(...)` her adımda çağrılır.
* **Henüz yok:** Oyuncunun uçağı gibi ajanı olmayan trafik ve helikopterlerin kaçınması.

## 9. İlerlemeyi takip etmek

| Ne | Nereden |
|---|---|
| Ne yapıyor | `agent->mode()` ya da `FlightPose::mode`: `GoTo`/`Route` varınca `Loiter`, `Hover`, `Cruise` ya da `Land` olur |
| Kalkış/iniş aşaması | `FlightPose::phase`: `Taxi`, `LineUp`, `TakeoffRoll`, `InitialClimb`, `Approach`, `Flare`, `Rollout`, `Landed`, `Parked` ... Gemide `Underway` (seyirde), `Parked` (durdu), `TerrainContact` (karaya oturdu) |
| Yerde mi | `FlightPose::on_ground` |
| Rotada kaçıncı waypoint | `agent->fixed_wing()->route().index()` / `agent->rotorcraft()->route().index()` / `agent->ship()->route().index()` |
| Araziden yükseklik | `agent->state().height_above_terrain` |
| Hızlar | `FlightPose`: `airspeed` (TAS), `calibrated_airspeed` (IAS/CAS), `mach`, `ground_speed`, `vertical_speed`. `state()` ek olarak `equivalent_airspeed` ve `dynamic_pressure` verir |
| Yakıt | `FlightPose::fuel` (kg; modellenmiyorsa NaN). `state()` ek olarak `fuel_flow` (kg/s) ve `mass` (kg) verir. İkmal: `agent->set_fuel(kg)` |
| G ve açılar | `FlightPose`: `g_force` (gövde eksenlerinde hissedilen g: x ileri, y sağ, z yukarı; düz uçuşta ~{0, 0, 1}), `angle_of_attack`, `sideslip_angle`, `turn_rate` (derece). `state()` ek olarak `load_factor`, `acceleration` (m/s², ENU), `pitch_rate` ve `roll_rate` verir |

## 10. Hangi araç ne yapar

| Komut | Sabit kanat | Helikopter / drone | Gemi |
|---|---|---|---|
| `go_to` | Noktaya uçar; varınca düz devam ya da tur (`Land` için pist gerekir, yoksa tur atar) | Noktaya uçar; varınca düz devam, üstünde bekleme (yavaşlayıp durur) ya da iniş | Noktaya gider; varınca düz devam, ya da (`Loiter`/`Land`) yavaşlayıp durur ve orada istasyon tutar |
| `loiter(merkez, r)` | `r` yarıçaplı tur (0 = performanstan) | `r` yarıçaplı tur; `r = 0` üstünde bekleme | `r` yarıçaplı tur (en az dönüş yarıçapı); `r = 0` orada istasyon tutma |
| `hover()` | Bulunduğu yerin etrafında tur | Bulunduğu yerde bekleme | Durur, bulunduğu yerde istasyon tutar |
| `follow_route` | Fly-by / fly-over, waypoint hızları | Aynısı, ayrıca `hold_time`. Drone keskin köşede yavaşlar; helikopter köşeyi hızını koruyarak yatışlı bir yayla alır | Fly-by, waypoint hızları; dümen gecikmesi için dönüşe erken başlar (`hold_time` yok) |
| `takeoff` | Gerekirse taksi ve hizalanma, sonra pistten koşu, rotasyon, tırmanış | Dikey kalkış | Yok (`false`) |
| `taxi(yol, yon)` | Yerde taksi, son noktada durur | Yok (`false`) | Yok (`false`) |
| `land(runway)` | Pist yaklaşması, flare, frenleme | Pist başına uçup dikey iniş | Yok (`false`) |
| `land()` | Pist olmadan inemez (`false`) | Bulunduğu yere dikey iniş | Yok (`false`); durmak için `hover()` |
| `land_on(spot)` | Henüz yok (`false`) | Hareketli güverteye iniş, güvertede gemiyle hareket; `takeoff()` güverteden kalkar | Yok (`false`); iniş noktalarını sağlar (`deck_spots`) |