# Yeni araç tipi ekleme rehberi

Bu belge kütüphanenin içine yeni bir **araç tipi** (farklı uçuş mantığına sahip bir model)
eklemeyi adım adım anlatır. Kod ve kod yorumları İngilizcedir; açıklamalar Türkçedir.

Örnek olarak bilerek mevcut tiplerden çok farklı bir araç seçildi: **sıcak hava balonu**.
Balon yalnızca irtifasını kontrol eder (brülör / valf), rüzgârla sürüklenir, yön veremez,
kol uçuşu yapamaz. Bu yüzden "araç bir komutu yerine getiremiyorsa ne yapmalı" sorusunu da
gösterir. Aşağıdaki kodun tamamı bu deponun bir kopyasında derlendi ve testleri geçti.

## 0. Yeni tip mi, yeni preset mi?

| İhtiyaç | Yapılacak |
|---|---|
| Aynı sınıftan başka bir uçak (ör. başka bir jet, daha büyük bir drone) | Sadece parametre seti: `presets.cpp`'ye bir fonksiyon ya da kendi `FixedWingParams` / `RotorcraftParams` değerleriniz. Kod değişmez. |
| Mevcut bir tipin davranışına küçük ekleme | O tipin `.cpp` dosyası (ör. `fixed_wing.cpp`). |
| Farklı fizik ve/veya farklı komut mantığı (balon, zeplin, planör, VTOL, kara aracı ...) | **Yeni tip**: bu rehber. |

## 1. Mimari özeti

```
XParams (veri)  ──►  XAgent (davranış)  ──►  FlightAgent (std::variant)  ──►  Flecs modülü
vehicle_params.hpp    x.hpp / x.cpp           flight_agent.hpp                tipten habersiz
```

* **Params**: yalnızca veri (kütle, hız sınırları, zaman sabitleri ...).
* **Agent**: dinamik + otopilot + navigasyon. `fm::FlightModel` concept'ini sağlar.
* **FlightAgent**: tüm tipler için tek giriş noktası. Komutları `std::visit` ile doğru
  agent'a iletir; geçersiz (NaN/sonsuz) komutları tipten bağımsız olarak burada eler.
* **Flecs modülü**: yalnızca `FlightAgent` ile konuşur. Yeni tip için **hiçbir değişiklik
  gerekmez**: `attach`, `attach_geo`, `FlightPose`/`GeoPose`, duraklatma, oyunun
  Transform'u değiştirmesiyle konum değişimi (`relocate`), `follow` hepsi otomatik çalışır.

Somut tiplere göre dallanan tek yer `flight_agent.hpp` içindeki `AgentVariant`'tır.

## 2. Kontrol listesi

| # | Dosya | Yapılacak |
|---|---|---|
| 1 | `include/fm/vehicle_params.hpp` | `BalloonParams` struct'ı; `VehicleParams` variant'ına ve `VehicleKind` enum'una ekleme |
| 2 | `include/fm/presets.hpp`, `src/presets.cpp` | (İsteğe bağlı) hazır parametre seti |
| 3 | `include/fm/balloon.hpp`, `src/balloon.cpp` | Agent sınıfı |
| 4 | `include/fm/flight_agent.hpp` | Header'ı include et, `AgentVariant`'a ekle |
| 5 | `CMakeLists.txt` | `src/balloon.cpp`'yi `flight_model` kaynaklarına ekle |
| 6 | `tests/` | Test dosyası ve `tests/CMakeLists.txt` |
| 7 | Oyun / plugin kodu | Yeni params'ı seçen yer (ör. plugin'deki `vehicle_params()`), `VehicleKind` üzerindeki `switch`'ler |

Bir adım unutulursa derleme açık bir mesajla durur (bkz. [Bölüm 7](#7-derleme-hataları-ve-anlamları)).

### Adım 1 — Parametreler

`include/fm/vehicle_params.hpp`, dosyanın sonundaki variant ve enum'un hemen üstüne:

```cpp
// Hot-air balloon: controls only its altitude (burner / vent) and drifts with the wind.
struct BalloonParams {
    std::string name = "hot_air_balloon";
    double climb_rate_max = 4.0;          // m/s
    double descent_rate_max = 4.0;        // m/s
    double altitude_gain = 0.1;           // 1/s, altitude error -> vertical speed
    double vertical_time_constant = 6.0;  // s, burner / vent response
    double drift_time_constant = 20.0;    // s, how fast it takes up the wind
    double landing_speed = 0.5;           // m/s, final descent
    double basket_height = 1.0;           // m, reference point above the ground when landed
    double takeoff_height = 150.0;        // m AGL
};

using VehicleParams = std::variant<FixedWingParams, RotorcraftParams, ShipParams, BalloonParams>;

enum class VehicleKind { FixedWing, Rotorcraft, Ship, Balloon };
```

Kurallar:

* Birimler SI (m, s, kg, rad). Açılar radyan; sadece Flecs çıktıları derece kullanır.
* Her alanın makul bir varsayılanı olsun; `BalloonParams{}` tek başına uçabilmeli.
* Bir params tipi yalnızca **bir** agent'a ait olabilir (eşleştirme `params_type` ile
  derleme zamanında yapılır).

### Adım 2 — Preset (isteğe bağlı)

```cpp
// include/fm/presets.hpp
BalloonParams hot_air_balloon();       // passenger hot-air balloon

// src/presets.cpp (namespace fm::presets içinde)
BalloonParams hot_air_balloon() { return {}; }  // the defaults describe one
```

### Adım 3 — Agent sınıfı

`include/fm/balloon.hpp`:

```cpp
// Hot-air balloon: climbs and descends on command, drifts with the wind, cannot steer.
#pragma once

#include "fm/flight_state.hpp"
#include "fm/formation.hpp"
#include "fm/relocation.hpp"
#include "fm/route.hpp"
#include "fm/terrain.hpp"
#include "fm/vehicle_params.hpp"

namespace fm {

class BalloonAgent {
public:
    using params_type = BalloonParams;
    static constexpr VehicleKind kind = VehicleKind::Balloon;

    BalloonAgent(BalloonParams params, const InitialConditions& ic);

    // --- Commands: only their altitude part is used, the balloon cannot steer --------
    void cruise();
    void hold(double course, double altitude, double speed, AltitudeRef ref);
    void set_speed(double) {}  // no speed of its own
    void go_to(const Vec3& point, const GoToOptions& options);
    void follow_route(Route route);
    void loiter(const Vec3& center, double radius, bool clockwise, AltitudeRef ref);
    bool takeoff(const TakeoffPlan& plan);
    void hover() { cruise(); }
    bool land();
    bool land(const Runway&) { return land(); }  // lands where it is
    void join_formation(const FormationSlot&) {}  // not possible: ignored
    void set_leader_state(const LeaderState&) {}
    bool relocate(const Vec3& position, double heading);

    // --- Simulation -----------------------------------------------------------------
    void step(double dt, const Environment& env);
    const FlightState& state() const { return out_; }
    FlightMode mode() const { return mode_; }
    double ground_offset() const { return p_.basket_height; }

private:
    void set_altitude(double altitude, AltitudeRef ref);
    double ground() const;  // NaN when unknown
    void update_output();

    BalloonParams p_;
    FlightState out_{};
    Vec3 position_{};
    Vec3 velocity_{};
    double heading_ = 0.0;
    bool on_ground_ = false;
    bool settle_ = false;  // relocated: check the ground in the next step
    FlightMode mode_ = FlightMode::Cruise;
    FlightPhase phase_ = FlightPhase::Airborne;
    double altitude_target_ = 0.0;
    AltitudeRef altitude_ref_ = AltitudeRef::Absolute;
    double ground_reference_ = kUnknown;  // ground known from the start, until terrain input
    TerrainMonitor terrain_{20.0};
};

}  // namespace fm
```

`src/balloon.cpp`:

```cpp
#include "fm/balloon.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace fm {

BalloonAgent::BalloonAgent(BalloonParams params, const InitialConditions& ic)
    : p_(std::move(params)) {
    position_ = ic.position;
    heading_ = wrap_2pi(ic.heading);
    on_ground_ = ic.on_ground;
    if (on_ground_) {  // on the ground position.z is the ground elevation
        ground_reference_ = ic.position.z;
        position_.z += p_.basket_height;
        phase_ = FlightPhase::Parked;
    }
    altitude_target_ = position_.z;
    update_output();
}

// --- Commands ---------------------------------------------------------------------------

void BalloonAgent::cruise() {
    mode_ = FlightMode::Cruise;
    set_altitude(position_.z, AltitudeRef::Absolute);
}

void BalloonAgent::hold(double, double altitude, double, AltitudeRef ref) {
    mode_ = FlightMode::Cruise;
    set_altitude(altitude, ref);
}

void BalloonAgent::go_to(const Vec3& point, const GoToOptions& options) {
    mode_ = FlightMode::Cruise;  // cannot fly there: takes the altitude only
    set_altitude(point.z, options.altitude_ref);
}

void BalloonAgent::follow_route(Route route) {
    if (route.waypoints.empty()) {
        cruise();
        return;
    }
    mode_ = FlightMode::Cruise;
    const Waypoint& first = route.waypoints.front();
    set_altitude(first.position.z, first.altitude_ref);
}

void BalloonAgent::loiter(const Vec3& center, double, bool, AltitudeRef ref) {
    mode_ = FlightMode::Cruise;
    set_altitude(center.z, ref);
}

bool BalloonAgent::takeoff(const TakeoffPlan& plan) {
    if (!on_ground_) return false;
    mode_ = FlightMode::Takeoff;
    phase_ = FlightPhase::InitialClimb;
    set_altitude(plan.climb_height > 0.0 ? plan.climb_height : p_.takeoff_height,
                 AltitudeRef::AboveTerrain);
    return true;
}

bool BalloonAgent::land() {
    if (on_ground_) return false;
    mode_ = FlightMode::Land;
    phase_ = FlightPhase::Landing;
    return true;
}

bool BalloonAgent::relocate(const Vec3& position, double heading) {
    const Relocation r = classify_relocation(position_, heading_, position, heading);
    position_ = position;
    heading_ = wrap_2pi(heading_ + r.turn);
    if (r.jump) terrain_.reset();  // old samples say nothing about the new place
    // Holding an absolute altitude: keep the one the game gave.
    if (r.vertical_jump && mode_ == FlightMode::Cruise && altitude_ref_ == AltitudeRef::Absolute)
        altitude_target_ = position_.z;
    settle_ = true;
    update_output();
    return true;
}

void BalloonAgent::set_altitude(double altitude, AltitudeRef ref) {
    if (ref == AltitudeRef::Keep) return;
    altitude_target_ = altitude;
    altitude_ref_ = ref;
}

// --- Simulation -------------------------------------------------------------------------

void BalloonAgent::step(double dt, const Environment& input) {
    if (!(dt > 0.0) || !std::isfinite(dt)) return;
    const Environment env = sanitized(input);
    terrain_.update(position_.xy(), env.terrain_elevation);
    const double g = ground();
    const double floor = std::isnan(g) ? -1e9 : g + p_.basket_height;
    if (settle_) {  // moved by the host: never below the ground
        settle_ = false;
        position_.z = std::max(position_.z, floor);
    }

    // Vertical speed from the altitude error; burner and vent respond slowly.
    double target = altitude_target_;
    if (altitude_ref_ == AltitudeRef::AboveTerrain) target += std::isnan(g) ? 0.0 : g;
    double vz = std::clamp(p_.altitude_gain * (target - position_.z), -p_.descent_rate_max,
                           p_.climb_rate_max);
    if (mode_ == FlightMode::Land)
        vz = -std::clamp(0.05 * (position_.z - floor), p_.landing_speed, p_.descent_rate_max);
    if (on_ground_ && vz <= 0.0) {  // stays on the ground
        velocity_ = {};
        update_output();
        return;
    }
    if (on_ground_) {
        on_ground_ = false;
        if (phase_ != FlightPhase::InitialClimb) phase_ = FlightPhase::Airborne;
    }
    velocity_.z = lag(velocity_.z, vz, p_.vertical_time_constant, dt);
    // Horizontally the balloon takes up the wind.
    velocity_.x = lag(velocity_.x, env.wind.x, p_.drift_time_constant, dt);
    velocity_.y = lag(velocity_.y, env.wind.y, p_.drift_time_constant, dt);
    position_ += velocity_ * dt;

    if (mode_ == FlightMode::Takeoff && position_.z >= target - 2.0) {
        mode_ = FlightMode::Cruise;
        phase_ = FlightPhase::Airborne;
        set_altitude(position_.z, AltitudeRef::Absolute);
    }
    if (position_.z <= floor) {  // touched the ground
        position_.z = floor;
        velocity_ = {};
        on_ground_ = true;
        phase_ = FlightPhase::Landed;
        mode_ = FlightMode::Cruise;
        set_altitude(position_.z, AltitudeRef::Absolute);
    }
    update_output();
}

double BalloonAgent::ground() const {
    return terrain_.valid() ? terrain_.current() : ground_reference_;
}

void BalloonAgent::update_output() {
    out_.position = position_;
    out_.velocity = velocity_;
    out_.attitude = {heading_, 0.0, 0.0};
    out_.airspeed = 0.0;  // moves with the air mass
    out_.ground_speed = velocity_.xy().length();
    out_.vertical_speed = velocity_.z;
    out_.track = out_.ground_speed > 0.1 ? heading_of(velocity_.xy()) : heading_;
    out_.height_above_terrain = position_.z - ground();
    out_.on_ground = on_ground_;
    out_.phase = phase_;
}

}  // namespace fm
```

Bu örnekte basit bir model olduğu için alt adım (sub-step) yok: `lag()` her `dt` için
kesin çözüm verir. Sertleşen (stiff) dinamiklerde mevcut agent'lar gibi adımı sabit alt
adımlara bölün (`fixed_wing.cpp`: 0.02 s, `rotorcraft.cpp`: 0.01 s).

### Adım 4 — FlightAgent'a kayıt

`include/fm/flight_agent.hpp`:

```cpp
#include "fm/balloon.hpp"
#include "fm/fixed_wing.hpp"
#include "fm/rotorcraft.hpp"
...
using AgentVariant = std::variant<FixedWingAgent, RotorcraftAgent, ShipAgent, BalloonAgent>;
```

Başka bir şey gerekmez. İsterseniz kısayol erişimci ekleyebilirsiniz (zorunlu değil,
`agent.as<BalloonAgent>()` her zaman çalışır):

```cpp
BalloonAgent* balloon() { return as<BalloonAgent>(); }
```

### Adım 5 — CMake

`CMakeLists.txt` içindeki `add_library(flight_model ...)` listesine `src/balloon.cpp`.
Flecs adaptörü ayrı bir kütüphanedir ve değişmez.

### Adım 6 — Test

`tests/test_balloon.cpp` (ve `tests/CMakeLists.txt` içindeki `fm_tests` listesine ekleyin):

```cpp
// Hot-air balloon: altitude control, wind drift, takeoff and landing.
#include "fm/flight_agent.hpp"
#include "fm/presets.hpp"
#include "harness.hpp"
#include "sim.hpp"

using namespace fm;
using fmtest::simulate;

TEST_CASE("balloon: takes off, drifts with the wind and lands") {
    InitialConditions ic;
    ic.position = {0, 0, 200};  // on the ground: z = ground elevation
    ic.on_ground = true;
    FlightAgent a(presets::hot_air_balloon(), ic);
    CHECK(a.kind() == VehicleKind::Balloon);
    const auto ground = [](double, double) { return 200.0; };
    CHECK(a.takeoff({}));
    simulate(a, 300.0, ground, {3.0, 0.0, 0.0});
    CHECK(a.mode() == FlightMode::Cruise);
    CHECK_NEAR(a.state().height_above_terrain, 150.0 + 1.0, 10.0);
    CHECK(a.state().position.x > 500.0);  // drifted east with the wind
    a.go_to({0, 0, 600});                 // cannot steer: only climbs to 600 m
    simulate(a, 300.0, ground, {3.0, 0.0, 0.0});
    CHECK_NEAR(a.state().position.z, 600.0, 10.0);
    CHECK(a.land());
    simulate(a, 600.0, ground, {3.0, 0.0, 0.0});
    CHECK(a.state().on_ground);
    CHECK(a.state().phase == FlightPhase::Landed);
    a.join_formation(formation::trail(0, 50.0));  // not possible: ignored
    CHECK(a.mode() == FlightMode::Cruise);
}
```

Flecs üzerinden de çalıştığını görmek için (`tests/test_flecs.cpp`):

```cpp
TEST_CASE("flecs: a new vehicle type works through the module unchanged") {
    flecs::world world;
    ecs::load(world);
    ecs::set_geo_origin(world, 41.0, 29.0);
    flecs::entity balloon = world.entity();
    ecs::attach_geo(balloon, presets::hot_air_balloon(), {41.0, 29.0, 100.0}, 0.0, 0.0, true,
                    [](FlightAgent& a) { a.takeoff({}); });
    balloon.set<ecs::TerrainHeight>({100.0}).set<ecs::Wind>({{0.0, 4.0, 0.0}});
    run(world, 200.0);
    CHECK(geo_pose_of(balloon).latitude > 41.003);  // drifted north
    CHECK(geo_pose_of(balloon).altitude > 200.0);
}
```

### Adım 7 — Oyun / plugin tarafı

* Params seçimi: ör. `examples/plugin/flight_plugin.cpp` içindeki `vehicle_params(int)`
  fonksiyonuna yeni bir `case` (`return fm::presets::hot_air_balloon();`).
* Oyun kodunda `VehicleKind` üzerinde `switch` varsa yeni değeri ekleyin (`-Wswitch`
  uyarısı eksik olanı gösterir).

## 3. `FlightModel` sözleşmesi

Her agent aşağıdakileri sağlar (`flight_agent.hpp` içindeki concept ile derleme zamanında
denetlenir). **Hiçbir komut hata vermez**: araç yapamıyorsa yok sayar ya da `false` döner.

| Üye | Anlamı | Yapamıyorsa |
|---|---|---|
| `using params_type = XParams;` | Bu agent'ı oluşturan params tipi | — |
| `static constexpr VehicleKind kind` | Oyun kodunun tip ayrımı için | — |
| `X(XParams, const InitialConditions&)` | Başlangıç durumu. `ic.on_ground` ise `ic.position.z` **zemin yüksekliğidir**; referans noktasını `ground_offset()` kadar yükseltin | — |
| `cruise()` | Hedef yok: düz devam (kendi tipine göre "düz") | — |
| `hold(course, altitude, speed, ref)` | Rota/irtifa/hız tut. `ref == Keep` → irtifayı değiştirme; `speed` NaN → hızı değiştirme | Yapabildiği kısmı uygular |
| `set_speed(speed)` | Hız hedefi (NaN = değiştirme) | Yok sayar |
| `go_to(point, options)` | Noktaya git; `options.on_arrival` sonrası davranış | Yapabildiği kısmı uygular |
| `follow_route(route)` | Rota takibi. Yerdeyse önce kalkış (mevcut tipler böyle yapar) | Yapabildiği kısmı uygular |
| `loiter(center, radius, cw, ref)` | Nokta etrafında tur / üzerinde bekleme | Yapabildiği kısmı uygular |
| `bool takeoff(plan)` | Kalkış; yerde değilse `false` | `false` |
| `hover()` | Bulunduğu yerde bekle (sabit kanat: üzerinde tur) | En yakın eşdeğeri |
| `bool land()` | Bulunduğu yere in | `false` |
| `bool land(runway)` | Piste in | Yapabildiği eşdeğer ya da `false` |
| `join_formation(slot)`, `set_leader_state(leader)` | Kol uçuşu; lider durumu her adım gelir | Yok sayar |
| `bool relocate(position, heading)` | Oyun aracı taşıdı: oradan devam (bkz. Bölüm 4). `heading` NaN = değiştirme | — |
| `step(dt, env)` | Simülasyon adımı | — |
| `const FlightState& state() const` | Çıktı | — |
| `FlightMode mode() const` | Şu anki mod | — |
| `ground_offset() const` | Yerdeyken referans noktasının zeminden yüksekliği (m) | — |

### `step(dt, env)` kuralları

* İlk satırlar: `if (!(dt > 0.0) || !std::isfinite(dt)) return;` ve
  `const Environment env = sanitized(input);` — oyundan gelen bozuk veri (NaN rüzgâr,
  sonsuz arazi) durumu bozmamalı. `FlightAgent` komutları zaten eler; `step` girdilerini
  agent kendisi temizler çünkü agent doğrudan da kullanılabilir.
* `dt` değişkendir (Flecs'te en fazla `max_step`, varsayılan 0.25 s). Kararlılık için
  gerekirse sabit alt adımlara bölün.
* Arazi yalnızca aracın **altında** bilinir (`env.terrain_elevation`, NaN = bilinmiyor).
  Eğim ve ileriyi tahmin için `TerrainMonitor` kullanın.
* `step` başka entity'lere erişmez; Flecs'te `FlightSystem` çok iş parçacıklı çalışır.
  Lider bilgisi yalnızca `set_leader_state` ile gelir.
* Sonunda `out_` (FlightState) güncellenir.

### `FlightState` alanları

| Alan | Doldurulması |
|---|---|
| `position` | ENU metre, referans noktası (yerdeyken zemin + `ground_offset()`) |
| `velocity` | Yere göre hız, ENU m/s |
| `attitude` | heading (Kuzey'den saat yönü), pitch (burun yukarı +), roll (sağ kanat aşağı +), radyan. Flecs'teki görsel rotasyon buradan üretilir |
| `airspeed`, `ground_speed`, `vertical_speed`, `track` | Hızlar ve yer izi |
| `on_ground`, `phase` | Yer durumu ve faz (`Parked`, `Airborne`, `Landing`, `Landed` ...) |
| `height_above_terrain` | Biliniyorsa zeminden yükseklik, yoksa NaN |
| `flight_path_angle`, `angle_of_attack`, `load_factor`, `throttle`, `mach`, `flaps`, `gear_down`, `speed_brake` | Anlamlıysa doldurun; değilse varsayılan kalabilir |

Koordinatlar her zaman çekirdek çerçevesindedir (ENU, metre, radyan). Oyun motorunun
eksenlerine ve enlem/boylama çeviri Flecs modülünde yapılır; agent'ın bunu bilmesi
gerekmez.

## 4. `relocate` nasıl yazılır

Flecs modülü, oyun `FlightPose`/`GeoPose`'u (ya da `bind_geo_transform` ile bağlı
Transform'u) değiştirdiğinde `relocate`'i kendisi çağırır. Agent'ın görevi aracı yeni
yerden **mantıklı şekilde** devam ettirmektir. `fm/relocation.hpp` ortak kuralları verir:

```cpp
const Relocation r = classify_relocation(old_position, old_heading, position, heading);
// r.jump          : düzeltmeden büyük (> 50 m, > 20 m dikey, > 10 deg) -> görevi buradan yeniden kur
// r.vertical_jump : irtifa düzeltmeden büyük değişti
// r.new_altitude  : irtifa hiç değişti mi (yerdeki araç havaya mı konuldu?)
// r.turn          : yön değişimi (rad); heading NaN ise 0
```

Önerilen davranış (mevcut tipler böyle yapar):

* Küçük düzeltmeler (`!r.jump`): yalnızca konumu/yönü değiştir; görev aynen sürer.
* Sıçramalar: rota bacağını yeni konumdan başlat (`RouteTracker::restart_leg`), düz uçuş
  hattını yeni konum ve yöne taşı, "burada bekle" türü hedefleri taşı, belirli bir noktaya
  bağlı hedefleri (loiter merkezi, waypoint) koru, yaklaşmayı baştan kur.
* Zemin kontrolünü **bir sonraki `step`'e** bırakın: yeni yerin arazisi ancak o zaman
  bilinir. Mevcut agent'lar bunun için `std::optional<Relocation>` tutar; aynı adımdan
  önce iki kez taşınırsa `combine()` ile birleştirir.
* Uzak sıçramada `TerrainMonitor::reset()` (eski örnekler yeni yerin eğimini söylemez).
* Saçma girdileri (NaN, |z| > 100 km) `FlightAgent::relocate` zaten reddeder.

## 5. Hazır yapı taşları

| Başlık | İçerik |
|---|---|
| `fm/math.hpp` | `Vec2/Vec3/Quat`, `wrap_pi`, `wrap_2pi`, `heading_of`, `heading_vector`, `lag`, `approach`, `body_axes`, `attitude_from_up`, `is_finite` |
| `fm/atmosphere.hpp` | `isa(altitude, temperature_offset)`: yoğunluk, ses hızı, sıcaklık |
| `fm/terrain.hpp` | `TerrainMonitor`: nokta örneklerinden eğim, `predicted_max` ile ileri arazi |
| `fm/route_tracker.hpp` | `RouteTracker`: bacak ilerlemesi, waypoint geçişi, doğrusal irtifa profili, AGL çözümü |
| `fm/guidance.hpp` | L1 yönlendirme: `l1_line`, `l1_leg`, `l1_point`, `l1_loiter` (yanal ivme komutu) |
| `fm/formation.hpp` | `LeaderTracker`: liderden slot konumu/hızı ve dönüş oranı |
| `fm/relocation.hpp` | `classify_relocation`, `combine`, eşik sabitleri |
| `fm/flight_state.hpp` | `sanitized(env)`, `kUnknown` (NaN) |

Mevcut üç agent (`fixed_wing.cpp`, `rotorcraft.cpp`, `ship.cpp`) bu parçaların nasıl
birleştiğini gösteren tam örneklerdir: komut → guidance → otopilot → dinamik →
`update_output`. `ship.cpp` en kısasıdır ve 3. boyutu olmayan bir aracın ortak arayüzü nasıl
karşıladığını gösterir: irtifayı yok sayar, `takeoff`/`land` için `false` döner, kendine özel
komutu (`steer`) `agent.ship()` ile açar.

## 6. Flecs tarafında yapılacak bir şey yok

Yeni tip eklendikten sonra:

```cpp
fm::ecs::attach_geo(e, fm::presets::hot_air_balloon(), {lat, lon, hot}, yaw, 0.0, true,
                    [](fm::FlightAgent& a) { a.takeoff({}); });
```

* Çıktılar (`FlightPose`, `GeoPose`), duraklatma, oyunun Transform'u değiştirmesiyle konum
  değişimi, `follow` (lider olarak `as_leader()` durumdan üretilir; takipçi olarak
  `join_formation` çağrılır) otomatik çalışır.
* Tipe özel bir şeye erişmek için: `fm::ecs::agent(e)->as<fm::BalloonAgent>()` ya da
  `agent->visit([](auto& m) { ... })`.
* Plugin DLL'lerinde de değişiklik yok; yeni kaynak dosya `fm::core` ile birlikte derlenir.

## 7. Derleme hataları ve anlamları

| Mesaj | Sebep |
|---|---|
| `every agent in AgentVariant must satisfy fm::FlightModel` | Agent'ta bir üye eksik ya da imzası farklı. Derleyicinin concept açıklaması hangi ifadenin sağlanmadığını gösterir (ör. `m.relocate(p, d)` bool dönmüyor). |
| `every params type in VehicleParams needs an agent in AgentVariant` | Params `VehicleParams`'a eklendi ama agent `AgentVariant`'a eklenmedi (Adım 4) ya da agent'ın `params_type`'ı yanlış. Ardından gelen `in_place_type_t<void>` hataları aynı sebepten. |
| `-Wswitch` uyarısı | Oyun kodunda `VehicleKind` üzerindeki bir `switch` yeni değeri bilmiyor. |

Not: iki agent aynı `params_type`'ı kullanırsa hata çıkmaz ama variant'ta önce gelen
seçilir, diğeri hiç oluşturulmaz. Her tipin kendi params struct'ı olsun.

## 8. Test önerileri

Yeni tip için en az şunlar (mevcut testler şablon olarak kullanılabilir):

1. Komut yokken davranış (`cruise`), rüzgârda ve rüzgârsız.
2. Desteklenen her komut ve desteklenmeyenlerin yok sayılması.
3. Yerden kalkış / iniş ve `ground_offset` (yerdeyken `position.z = zemin + offset`).
4. `relocate`: küçük düzeltme, büyük sıçrama, havadan yere / yerden havaya
   (`tests/test_relocation.cpp`).
5. Bozuk girdiler: NaN rüzgâr, sonsuz arazi, sonsuz `dt` (`tests/test_robustness.cpp`).
6. Flecs üzerinden `attach` / `attach_geo` ile birkaç saniyelik uçuş.

## 9. Sık yapılan hatalar

* **Yön birimi**: heading radyan, Kuzey'den saat yönünde (ENU: x Doğu, y Kuzey). Derece
  yalnızca Flecs çıktılarında.
* **Yerdeki başlangıç**: `InitialConditions::position.z` yerdeyken zemin yüksekliğidir;
  agent referans noktasını `ground_offset()` kadar yükseltmelidir, aksi halde
  `snapshot()` ile model değişiminde araç zıplar.
* **NaN sızması**: NaN bir kez duruma girerse kalıcıdır. Bölmelerde sıfıra karşı
  koruma, `std::clamp` ve `sanitized` kullanın.
* **Paylaşılan durum**: agent'lar birbirini görmez; global/statik değişken kullanmayın
  (çok iş parçacıklı `FlightSystem`).
* **Pointer saklama**: `fm::ecs::agent(e)` dönüşü component eklenip çıkarılınca geçersiz
  olur; komutu hemen verin, pointer'ı saklamayın.
