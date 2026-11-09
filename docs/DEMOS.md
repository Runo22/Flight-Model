# Ankara demoları (canlı, resim eklentisi)

`examples/resim/` altındaki senaryolar, resim eklentisinde normal hızda, oyunun kendi
karelerinde canlı gösterilir. Senaryolar dünyayı kendileri ilerletmez: oyun `world.progress()`
çağırır. Her senaryo uçaklarını koyar, ilk komutları verir ve kendi mantığını kaydeder
(`every_frame`). Bu mantık her karede çalışır, sonraki komutları verir (ayrılma, iniş, apron'a
taksi) ve önemli anları `RESIM_LOG` ile yazar.

Dosyalar resim başlıklarına bağlıdır ve eklentiye olduğu gibi eklenir. Repo içinde yalnızca
testler derler: `tests/resim/stub/` resim API'sinin küçük bir taklididir (kullanılan
bileşenler oyundaki tanımlarıyla aynı tutulur). `fm_resim_<senaryo>` testleri her senaryoyu
60 Hz'de uçurur ve sonucunu kontrol eder. `fm_resim_restart` yarıda yeniden başlatmayı,
`fm_resim_all_at_once` on iki senaryonun aynı dünyada birlikte uçuşunu sınar. Biçimlendirme resim projesinin stilindedir (`examples/resim/.clang-format` bu klasörü
biçimlendirmeye kapatır).

| Dosya | İçerik |
|---|---|
| `helpers.hpp/.cpp` | Ankara yerleri ve havaalanları, sentetik arazi, dünyaya bağlanma, `every_frame`, `spawn`, enlem/boylam ↔ yerel dönüşümler, pist, ölçümler |
| `scenarios.hpp/.cpp` | Senaryolar ve `demo_scenarios` listesi (ad, başlık, fonksiyon) |

> **Not:** Arazi gerçek yükseklik verisi değildir. ~900 m'lik Ankara havzası, çevre tepeleri
> ve Elmadağ (~1.850 m) olan, havaalanlarında düzleşen sentetik bir yüzeydir. Havaalanı ve yer
> koordinatları, rakımlar ve pist yönleri yaklaşıktır. HOT eklentisi gelince
> `DemoSetup::synthetic_terrain = false` yapılır.

## Eklentiye bağlama

```cpp
// Yüklerken
fm::ecs::load(world);
fm::ecs::set_geo_origin(world, 39.93, 32.80);               // isteğe bağlı: yoksa bind_world koyar
fm::ecs::bind_geo_transform<resim::core::Transform>(world);
bind_world(world);                                          // DemoSetup ile ayarlanabilir

const std::string title = demo::run_scenario("flypast");     // başlık ya da "[ERR] ..." metni

// Kaldırırken
unbind_world();
fm::ecs::unload(world);
```

`bind_world()` tek dünyayı bir kere bağlar; sonra hiçbir yardımcı `world` parametresi almaz:

* Varlık alan yardımcılar dünyayı `e.world()` ile bulur (ücretsizdir, her zaman doğrudur).
* Enlem/boylam dönüşümleri bağlanırken saklanan `GeoProjection` ile yapılır: ECS sorgusu ve
  kopya yoktur, yalnızca matematik vardır.
* Dünya ham işaretçi (`ecs_world_t*`) olarak saklanır. Saklanan bir `flecs::world` referans
  sayacını tutar ve oyunun dünyası kapanırken yok edilmesini engeller.
* `bind_world()` çağrılmazsa ilk kullanımda `resim::core::get_world()` ile kendiliğinden
  bağlanır.

`bind_world()` üç sistemi uçuş modülünün kapsamına ekler, böylece `fm::ecs::unload()` onları da
siler:

| Sistem | Faz | İş |
|---|---|---|
| `DemoScenarios` | `PreStore`, immediate | Senaryo mantıkları. Uçuş modeli adım attıktan ve `Transform` yazıldıktan sonra çalışır. Bir sistemin içinde çalıştığı için var olan ajanlara verilen komutlar (iniş, taksi, tur) hemen etkili olur. Eklenen ya da silinen bileşenler (yeni ajan, `follow()`, `stop_following()`) ise sistem bitince işlenir. Orada oluşturulan bir ajanın komutu `spawn()`'ın `setup` parametresiyle verilir |
| `DemoRouteDisplay` | `PostUpdate` | `resim::core::WaypointList`, her ajanın uçtuğu rotayı (rota, `go_to`, kalkıştan sonraki rota), `current_index` ile hedef noktayı ve `loop` ile rotanın döngü olup olmadığını gösterir. Rota yalnızca değiştiğinde bir kere dönüştürülür; sonra sadece indeks güncellenir. Rota olmayan şeyler (tur merkezi) için `show_waypoints()` |
| `DemoTerrain` | `PreUpdate` | `fm::ecs::TerrainHeight`, `ankara::terrain()` ile doldurulur (`DemoSetup::synthetic_terrain`) |

## Senaryo mantığı

```cpp
every_frame("ad", { aktörler }, [=, durum = ...](double t) mutable {
    // t: senaryonun başından beri geçen s (duraklatılan süre sayılmaz)
    return bitti_mi;
}, /* on_stop, isteğe bağlı */);
```

Senaryo mantığı şu durumlarda biter: `true` döndüğünde, aktörlerden biri silindiğinde (sahne
sıfırlama), aynı adla tekrar başlatıldığında ya da `stop_scenario(ad)` / `stop_scenarios()`
çağrıldığında. Bittiğinde uçaklar son komutlarını uçmaya devam eder. `spawn()` aynı adlı varlığı
yeniden kullanır ve temizler (kol, uçuş dizisi, rota gösterimi silinir), bu yüzden bir
senaryoyu tekrar çalıştırmak onu baştan başlatır; yarıda kalmış eski bir iniş dizisi yeni
uçaklara komut veremez. `scenario_running(ad)`, mantığın hâlâ çalışıp çalışmadığını söyler. Senaryolardaki
varlık adları birbirinden farklıdır; bu sayede iki senaryo birbirinin uçaklarını almaz.

## Senaryolar

| Ad | Ne olur | Gösterdiği |
|---|---|---|
| `anka3_tour` | Akıncı pist 03 başındaki Anka-3 kalkar, iki noktalı rotayı uçar ve aynı piste iner | Park hâlinde `follow_route` (önce kalkış), rota sonunda iniş, eşik + yönden pist |
| `formation_goto` | Kaan ve iki Anka-3 V düzeninde 320 knot ile Akıncı'ya gider ve orada tur atar; bir Hürkuş Gölbaşı üzerinde 6.000 m'de, 2 km yarıçapla tur atar | `formation::vic`, `go_to` (`TrueKnots`, `Keep`, `Loiter`), `loiter` |
| `syria_orbit` | Hürjet Kuzey Suriye üzerinde tur atar | Projeksiyon merkezinden ~650 km uzakta `loiter` |
| `rejoin` | Dağınık bir Hürjet, bir Anka-3 ve bir Kızılelma kola girer, Mogan Gölü üzerine gider ve orada tur atar. Kol konumdayken ayrılır: Kızılelma göl üzerinde kalır, Hürjet ve Anka-3 Akıncı'ya overhead break ile sırayla iner | `follow`, "önce kol, sonra git", `stop_following`, `land_flight` |
| `circuit` | Hürkuş apron'dan taksi yapar, kalkar, Kahramankazan ve Etimesgut üzerinden tur atar, overhead break ile iner, 1.500 m'de durur ve apron'a geri taksi yapar | `TakeoffPlan::taxi/then`, `LandingOptions`, `taxi` |
| `waypoints` | Anka Akıncı'dan kalkar, köşe keser, 110 KCAS uçar, Elmadağ'ı araziden 400 m yukarıda geçer, Mogan Gölü'nün üstünden geçer ve iner | Köşe kesme / üstünden geçme, `CalibratedKnots`, `AboveTerrain`, 25° seyrüsefer yatışı |
| `flypast` | 29 Ekim: dört F-16 sığınaklarından taksi yapar, 10 s arayla kalkar, elmas düzende 300 KCAS ile Anıtkabir üzerinden geçer, overhead break ile sırayla iner | `takeoff_flight`, `FlightTakeoff::slots`, `land_flight` |
| `traffic` | Dört uçak aynı anda, aynı irtifada Kızılay'a gelir: önce çarpışma kaçınma kapalı (~1 m), sonra aynı karşılaşma açık (~100 m, dikey ayrım) | `set_collision_avoidance`. Ayar dünya genelidir: ilk geçiş sırasında diğer senaryolar da kaçınmasız uçar. Bitince ya da durdurulunca açılır |
| `lowlevel` | T129 Güvercinlik'ten kalkar, araziden 40 m yukarıda Eymir üzerinden Elmadağ'a uçar, zirvede bir dakika havada asılı kalır ve doğudaki sırta iner | Helikopterde arazi takibi, `hold_time` |
| `recovery` | Esenboğa'nın 2.500 m üstündeki A400M ve Akıncı'nın 2.000 m üstündeki Aksungur iniş alır; önce alçalma turu atarlar, sonra stabil yaklaşmayla inerler | Yüksekten `land()` |
| `relief` | Batıdaki görev noktasında Anka tur atar. Aksungur Akıncı'dan kalkar ve 300 m yukarıda görevi devralır. Dört dakika birlikte tur attıktan sonra Anka dönüp iner | Görev devri, yükseklik ayrımı |

Testlerde (`ctest -R fm_resim`) günlükler (özet). On iki senaryo birlikte, 27 uçakla kare
başına ~0,2 ms sürer:

```
[rejoin]   t=457s formed up over Mogan lake (worst slot 0 m, the Hurjet waited for the UAVs): split
[rejoin]   t=1052s Hurjet and Anka-3 landed at Akinci, 58 s apart; Kizilelma orbits Mogan lake
[circuit]  t=1291s parked, 0.0 m from its spot
[flypast]  t=598s over Anitkabir: leader 12 m off, worst slot 3 m
[flypast]  t=1251s all four landed, touchdowns at least 11 s apart
[traffic]  closest approach without collision avoidance: 1 m
[traffic]  closest approach with collision avoidance: 104 m (climbed / descended away)
[lowlevel] t=901s landed on the ridge, 0.2 m from the point
```

## Konumlar (yaklaşık)

| Yer | Enlem, boylam | Not |
|---|---|---|
| Akıncı (Mürted) Hava Üssü | 40,0789, 32,5656 | 857 m, pist 03 (034°), 3.000 m |
| Esenboğa | 40,1281, 32,9951 | 953 m, pist 03R (034°), 3.750 m |
| Güvercinlik | 39,9351, 32,7408 | 820 m, helikopter üssü |
| Anıtkabir | 39,9250, 32,8369 | |
| Kızılay | 39,9208, 32,8541 | |
| Mogan Gölü | 39,7800, 32,8000 | |
| Eymir Gölü | 39,8250, 32,8300 | |
| Elmadağ | 39,8600, 33,2300 | Sentetik tepe ~1.850 m |
| Kahramankazan | 40,2050, 32,6800 | |

## Yeni senaryo eklemek

1. `scenarios.hpp`'de fonksiyonu bildir, `demo_scenarios` listesine ad ve başlıkla ekle.
2. Fonksiyonda `spawn()` ile uçakları koy (adlar diğer senaryolardan farklı olmalı), ilk
   komutları ver.
3. Sonraki adımlar varsa `every_frame()` ile mantığı kaydet; durumu `pose()`, `distance()`,
   `landed_on()`, `slot_error()` ile izle, önemli anları `RESIM_LOG` ile yaz.
