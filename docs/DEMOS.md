# Ankara demoları

`examples/ankara_demos.cpp`, Ankara çevresinde uçuş ağırlıklı dokuz senaryo uçurur. Her biri
kendi dünyasını kurar, sonucunu kendisi kontrol eder ve ayrı bir test olarak çalışır
(`fm_ankara_<demo>`). Dünya coğrafidir (enlem/boylam); oyun tarafı `navigation.cpp`'deki
gibi: `Transform` (lat, lon, alt, roll, pitch, yaw), HOT sorgusu, `bind_geo_transform`.

```sh
./build/examples/fm_ankara_demos                        # hepsi
./build/examples/fm_ankara_demos rejoin flypast         # seçilenler
./build/examples/fm_ankara_demos --list
./build/examples/fm_ankara_demos --csv iz.csv --kml iz.kml
```

> **Not:** Arazi gerçek yükseklik verisi değildir. ~900 m'lik Ankara havzası, çevre tepeleri
> ve Elmadağ (~1.850 m) olan, havaalanlarında düzleşen sentetik bir yüzeydir. Havaalanı ve yer
> koordinatları, rakımlar ve pist yönleri yaklaşıktır.

## Senaryolar

| Demo | Ne olur | Gösterdiği |
|---|---|---|
| `rejoin` | Polatlı tarafında dağınık bir Hürjet ve iki Kızılelma kola girer, Mogan Gölü üzerine gider ve orada tur atar. Kol konumdayken ayrılır: Kızılelma-2 göl üzerinde 3.600 m'de beklemede kalır, Hürjet ve Kızılelma-1 Akıncı'ya overhead break ile sırayla iner | `follow`, "önce kol, sonra git" (lider takipçiler gelene kadar tur atar), `stop_following`, `loiter`, `land_flight` |
| `circuit` | Hürkuş apron'dan taksiyle piste çıkar, kalkar, Kahramankazan ve Etimesgut üzerinden kısa bir tur atar, overhead break ile iner, pistte 1.500 m'de durur ve apron'a geri taksi yapar | `TakeoffPlan::taxi`, `TakeoffPlan::then`, `LandingOptions` (overhead, durma noktası), `taxi` |
| `waypoints` | Anka Akıncı'dan kalkar. Kahramankazan'da köşeyi keser, Esenboğa'nın doğusundan sonra 110 KCAS uçar (knot ile), Elmadağ'ı araziden 400 m yukarıda geçer, Mogan Gölü'nün tam üstünden geçer, keskin dönüşün üstünden geçip Akıncı'ya iner | Köşe kesme ve üstünden geçme, `SpeedRef::CalibratedKnots`, `AltitudeRef::AboveTerrain`, `nav_bank` (25°), rota sonunda iniş |
| `flypast` | 29 Ekim: dört F-16 sığınaklarından taksi yapar, 10 s arayla kalkar, elmas düzende toplanır, 300 KCAS ile Anıtkabir üzerinden geçer, overhead break ile sırayla iner | `takeoff_flight` (taksi, sıralı kalkış, `FlightTakeoff::slots` ile elmas), `land_flight` |
| `traffic` | Hürkuş, Kızılelma, Anka ve Aksungur dört yönden aynı anda, aynı irtifada Kızılay'a gelir. Çarpışma kaçınma dikey olarak ayırır; aynı karşılaşma kaçınma kapalıyken de uçulur | Çarpışma kaçınma (çoklu tehdit: kaçınma açıkken en yakın geçiş ~104 m, kapalıyken ~1 m) |
| `lowlevel` | T129 Güvercinlik'ten kalkar, Eymir Gölü üzerinden Elmadağ'a araziden 40 m yukarıda uçar, zirvede bir dakika havada asılı kalır, doğudaki sırta iner | Helikopterde arazi takibi, `hold_time`, rota sonunda iniş |
| `recovery` | Esenboğa'nın 2.500 m üstündeki bir yolcu jetine ve Akıncı'nın 2.000 m üstündeki bir Aksungur'a iniş verilir. İkisi de önce son yaklaşma noktasının dışında tur atarak alçalır, sonra stabil yaklaşmayla iner | Yüksekten `land()`: alçalma turu, pas geçmeden iniş |
| `relief` | Batıdaki görev noktasında Anka tur atar. Aksungur Akıncı'dan kalkıp gelir ve 300 m yukarıda görevi devralır, Anka dönüp iner | Görev devri: `TakeoffPlan::then` ile görev noktasına `Loiter`, yükseklik ayrımı |
| `escort` | Eskişehir yolunda batıya giden bir konvoyu (oyun mantığı) T129 300 m geriden ve 150 m yukarıdan, bir drone 60 m yukarıdan yanında izler | Oyunun hareket ettirdiği varlığı takip: `TrackedGeoPose`, `follow` |

## Çıktılar

**CSV** (`--csv`): her uçak için saniyede bir satır.

| Sütun | Anlamı |
|---|---|
| `demo`, `t`, `name` | Demo, demo içindeki zaman (s), varlık adı |
| `lat`, `lon`, `alt` | Derece, derece, m (deniz seviyesinden) |
| `roll`, `pitch`, `yaw` | Derece; `yaw` gerçek yön |
| `kcas` | Gösterge hızı (knot) |
| `mode`, `phase` | `FlightMode`, `FlightPhase` |
| `gear` | İniş takımı pozisyonu (0 yukarıda, 1 aşağıda) |

**KML** (`--kml`): Google Earth'te açılır (Dosya → Aç). Her demo ayrı bir klasördür,
uçaklar zamanlı izdir (`gx:Track`); zaman kaydırıcısıyla oynatılır. Demolar 29 Ekim 2026
08:00 UTC'den başlayarak art arda oynar (geçiş töreni 09:30 UTC, Ankara saatiyle 12:30).
"Places" klasöründe havaalanları ve yer işaretleri vardır. İrtifalar deniz seviyesinden
mutlaktır; arazi sentetik olduğu için alçak uçuşta iz yer yer Google Earth'ün gerçek
arazisinin altında ya da çok üstünde görünebilir.

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

1. `demo_...(Trace&)` adlı bir fonksiyon yaz: `make_world()` ile dünyayı kur, `spawn()` ile
   uçakları koy, komutları ver.
2. Döngüde `run(world, 1.0)` ve `trace.sample(world, t)` çağır; durumu `pose()`,
   `transform()`, `distance()`, `landed_on()`, `slot_error()` ile izle.
3. Sonunda `check(koşul, "açıklama")` ile sonucu doğrula.
4. `kDemos` listesine ve `examples/CMakeLists.txt`'teki test listesine ekle.
