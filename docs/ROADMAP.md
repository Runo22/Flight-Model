# Yol Haritası

Bu belge projenin mimari kararlarını, şu an tamamlanmış olanları ve sıradaki fazları
özetler. Kod ve kod yorumları İngilizcedir; bu belge ekip içi planlama içindir.

## 1. Temel kararlar

| Konu | Karar | Neden |
|---|---|---|
| Model | Nokta-kütle (3-DOF) dinamik + türetilmiş tutum açıları | JSBSim gibi 6-DOF modeller pilot girişi (stick/pedal) bekler; oyunda AI için gereksiz maliyet. 3-DOF; dönüş oranı, yük faktörü, stall, itki kaybı, motor gecikmesi, rüzgârda crab gibi oyuncunun gördüğü davranışları doğru üretir. |
| Otopilot | Enerji tabanlı (TECS benzeri), model tersinden gaz hesabı | Her uçak tipi için ayrı PID ayarı gerekmez; parametreler değişince otopilot kendiliğinden uyum sağlar. |
| Yol takibi | L1 non-lineer guidance (açık kaynak otopilotlarla aynı yöntem) | Rüzgârda sapmasız hat takibi; L1 mesafesi dönüş yarıçapına göre ölçeklenir, büyük uçaklar taşmaz. |
| Arazi | Sadece entity altındaki yükseklik (HOT) | Oyunun sağladığı veri bu. Geçmiş örneklerden eğim tahmin edilir; minimum irtifa ve AGL (arazi takibi) bunun üzerine kurulu. |
| ECS | Çekirdek Flecs'ten bağımsız, ince Flecs modülü | Flecs güncellemesi çekirdeği etkilemez; modül düz Flecs API'si kullanır (Flecs ≥ 4.1, `try_get`). |
| Dil | C++20 çekirdek (C++23 ile de derlenir) | GCC/Clang/MSVC'de sorunsuz; modules ve `std::print` gibi henüz oturmamış özelliklerden kaçınıldı. |

Her adımda aynı boru hattı çalışır:

```
Mod (cruise/route/takeoff/formation) → Guidance (L1, hedef irtifa/hız, arazi tabanı)
  → Otopilot (yatış, uçuş yolu açısı, gaz) → Dinamik (alt adımlarla) → FlightState
```

Katmanlar arasında yalnızca düz struct'lar geçer; bu yüzden ileride bir katman (ör.
dinamik → 6-DOF) diğerlerine dokunmadan değiştirilebilir.

## 2. Faz 0 — tamamlandı

- **Araç sınıfları:** sabit kanat (jet, yolcu jeti, turboprop, hafif pervaneli, sabit
  kanat İHA), multirotor, helikopter ve gemi, hazır parametre setleriyle.
- **Komutlar:** düz uçuş (hedef yoksa), rota/iz tutma, bir noktaya git, rota (fly-by /
  fly-over, döngü, waypoint bekleme süresi), loiter/orbit, pistten kalkış (koşu, rotasyon,
  havalanma, flap toplama, tırmanış), multirotor dikey kalkış/iniş/hover.
- **Kol uçuşu ve takip:** slot tabanlı formasyon (echelon, V, yan yana, arka arkaya),
  katılma (join-up), liderin dönüşünü ileri besleme, minimum ayrılma, lider kaybolunca düz
  uçuşa dönüş. Lider olarak her entity kullanılabilir (`TrackedPose`).
- **Arazi:** minimum yükseklik koruması, AGL irtifa (arazi takibi), eğimli pistten kalkış.
- **Koordinat sistemleri:** ENU, Unity, Unreal (cm), Godot, NED ve özel tanım.
- **Geodetik dünya:** WGS-84 enlem/boylam/irtifa; yatayda teğet düzlem projeksiyonu,
  dikeyde doğrudan irtifa; yaw için harita kuzeyi–gerçek kuzey düzeltmesi. Flecs'te
  `GeoPose` çıktısı, `TrackedGeoPose` girdisi, `set_geo_origin` / `attach_geo` / `to_local`.
- **Flecs modülü:** Flecs ≥ 4.1 (düz API, `try_get`/`try_get_mut`); 4.1.2 ve 4.1.6 ile derlenip
  test edildi. Çalışma anında sürüm kontrolü yapmaz (host'un sorumluluğu).
- **Plugin DLL desteği:** `fm::ecs::load()` (örnek plugin, yanlış bağlanmış Flecs kopyasını
  giriş noktasında yakalar),
  `fm::ecs::unload()` (sistemleri ve component'leri temizler, DLL güvenle boşaltılır),
  `FM_FLECS_TARGET` / `FM_FLECS_SHARED`, gerçek yükle–boşalt–tekrar yükle testi.
- **İniş:** pist yaklaşması, süzülüş yolu, pas geçme, flare, teker koyma, frenleme.
- **Duraklatma:** dünya çalışırken tüm ajanları (`set_paused`) veya tek ajanı (`pause`)
  durdurma; uzun karelerde `max_step` sınırı.
- **Konum değişimi (teleport):** oyun Transform'u değiştirince ajan yeni konumdan devam
  eder (`FlightAgent::relocate`, Flecs'te otomatik algılama, `bind_geo_transform`).
  Küçük düzeltmeler görevi bozmaz; büyük sıçramalarda rota bacağı, düz uçuş hattı,
  yaklaşma yeni konumdan yeniden kurulur; yer/hava geçişleri ve saçma değerler ele alınır.
- **Referans sınıfı presetler:** F-16C, F-15C, Hürkuş, Kızılelma, Anka-S, Aksungur,
  T129 ATAK, AH-64D, CH-47F, TCG Anadolu sınıflarında, yayımlanmış performansa uydurulup araç uçurularak ölçülen modeller
  (`docs/PRESETS.md`, `fm_performance_report`).
- **Helikopter kipi:** güce dayalı performans (momentum teorisi, yer etkisi, profil ve parazit
  güç, motor ve şanzıman sınırı), yatışlı dönüşler, köşelerde yay, yana/geri hız sınırı.
- **Motor ve atmosfer:** tropopoz üstünde itki/güç yoğunlukla orantılı, jet ram etkisi,
  turbo-şarjlı / flat-rated motorlarda kritik irtifaya kadar sabit güç, ISA 32 km'ye kadar.
- **Arazi:** arazi tabanının altında kalan uçak hızını en iyi tırmanma açısı hızına düşürüp
  enerjiyi tırmanmaya verir (zayıf tırmanan İHA'lar yükselen araziyi aşar).
- **Tırmanış kipi:** büyük tırmanışlar, komut edilen hız yetmiyorsa tam güçle en iyi
  tırmanma hızında (ya da `climb_speed` ile verilen hızda) uçulur.
- **Hava verisi ve türev çıktılar:**
  - Hızlar: CAS/IAS, EAS, Mach (ses üstü pitot formülü dahil), dinamik basınç.
  - Kuvvetler ve oranlar: ivme, gövde eksenlerinde g kuvveti, dönüş / yunuslama / yatış
    hızları.
  - Açılar: hücum açısı, yan kayma.
  - Hız komutları TAS, CAS ya da Mach olarak verilebilir.
- **Yakıt ve kütle (sabit kanat):** TSFC / BSFC ile tüketim (afterburner ayrı), hafifleyen
  kütle, yakıt bitince motor durması ve en iyi süzülme hızında süzülme, ikmal (`set_fuel`).
  Yatay uçuşta hızlanmada itki fazlasının tamamı kullanılır (savaş uçağı tam afterburner
  ile ivmelenir).
- **Kol lideri:** takipçisi olan sabit kanatlı lider yatışını ve hızını takipçilerin
  izleyebileceği kadar sınırlar: dıştaki takipçi için hız ve yatış sınırı, içteki için
  minimum hız. Flecs bunu `follow` ilişkilerinden otomatik yapar.
- **Çarpışma önleme (sabit kanat):**
  - TCAS benzeri dikey çözüm: en yakın geçiş tahmini, zıt yönlü tırmanış / alçalış.
  - Yere yakın ve kalkış/inişte devre dışı; arazi tabanı korunur.
  - Flecs'te ızgarayla komşu bulan trafik sistemi; kol üyeleri birbirinden kaçmaz.
- **Gemi:** TCG Anadolu sınıfı LHD (helikopter / İHA gemisi). Güç ve bollard pull sınırlı
  itki, Nomoto dümen tepkisi, dönüş çemberi, dönüşte hız kaybı / drift / yatma. LOS
  otopilot, istasyon tutma (ileri, tornistan, yerinde dönüş), karaya oturma. Manevra
  değerleri IMO standartlarıyla karşılaştırılarak ölçülür.
- **Gemiye iniş / gemiden kalkış (helikopter, drone):** seyirdeki geminin güverte spotuna
  arkadan yaklaşma, gemiyle aynı hıza gelip spotun üstünde alçalma, güvertede gemiyle
  birlikte hareket, hareketli güverteden kalkış. Taşıyıcı simüle edilen bir gemi ya da
  oyunun hareket ettirdiği herhangi bir entity olabilir (`fm::ecs::land_on`).
- **Yerde taksi (sabit kanat):** hangardan taksi yolu boyunca piste gelme, pist eksenine
  hizalanma (`LineUp`) ve kalkış (`TakeoffPlan::taxi`, `TakeoffPlan::heading`); inişten sonra
  park yerine taksi (`taxi(yol, son_yon)`). Pure pursuit yol takibi, köşe öncesi frenleme,
  burun tekeri dönüş yarıçapı, son noktada istenen yönde durma.
- **Kol halinde sıralı kalkış ve iniş (Flecs):** `takeoff_flight` (sırayla taksi, pistte
  kademeli dizilme, aralıklı ya da ikişerli kalkış, lidere katılma) ve `land_flight`
  (echelon, overhead break, aralıklı touchdown, pistin dönüşümlü yarıları, sıralı durma).
  Çekirdekte: overhead pattern (`LandingOptions`), kalkışta bekle/serbest bırak
  (`TakeoffPlan::hold`, `release_takeoff`), inişte durma noktası.
- **Testler:** 105 çekirdek + 28 Flecs + navigasyon örneği + 2 plugin testi;
  AddressSanitizer/UBSan temiz.

## 3. Sonraki fazlar

### Faz 1 — Sabit kanat iniş ve yer hareketleri
- [x] Yaklaşma: pist ekseni uzantısına konumlanma, final yaklaşma fiksi, 3° süzülüş yolu
  (alttan yakalama + dikey hız ileri besleme), flap/iniş takımı, yaklaşma hızı 1.3 Vs.
- [x] Karar yüksekliğinde stabilite kontrolü, pas geçme ve otomatik yeni deneme.
- [x] Gecikme telafili flare, yumuşak teker koyma (0.4–1.0 m/s), frenleme, park durumu.
- [x] Rota sonunda iniş (`Route::runway` + `EndBehavior::Land`), inişten sonra tekrar kalkış.
- [x] Yerde taksi: taksi yolu (waypoint listesi) üzerinde düşük hızlı hareket ve dönüş,
  pist eksenine hizalanma, yön ile kalkış, inişten sonra park yerine taksi.
- [x] Overhead pattern (initial, break, downwind, base, kısa final), kapalı pattern ile pas
  geçme, inişte durma noktası.
- [x] Kol halinde sıralı kalkış ve iniş (`fm::ecs::takeoff_flight` / `land_flight`).
- [ ] Yerde çarpışma önleme (taksi yolunda sıra bekleme).
- [ ] Bağımsız trafik için pist sıralaması (ATC benzeri: farklı kollardan gelenleri tek
  sıraya koyma, düz yaklaşmada aralık).
- [ ] Yerdeyken rüzgâr etkisi, yan rüzgârda teker koymadan önce crab'dan çıkma (decrab).
- [ ] Pist uzunluğu ve pist sonu kontrolü (kısa pistte pas geçme / kalkış iptali).

### Faz 2 — Veri odaklı parametreler
- Uçak tiplerini JSON'dan okuma (glaze veya nlohmann/json), hot-reload.
- Flecs prefab entegrasyonu: tip başına prefab, entity'ler `is_a` ile parametre paylaşır.

### Faz 3 — Gelişmiş trajektori
- Zaman etiketli (4D) trajektori: belirli bir noktaya belirli bir saatte varma.
- Waypoint'te hız/irtifa kısıtları (at / at-or-above / at-or-below).
- Bekleme paterni (holding), prosedür kalkış/geliş benzeri rota şablonları.
- Rota önizleme: tahmini varış süresi ve izlenecek eğrinin hesaplanması (çizim için).

### Faz 4 — Ölçek ve performans
- Detay seviyesi (LOD): uzak araçlar için ucuz kinematik model ve düşük güncelleme
  frekansı; `snapshot()` ile sıçramasız geçiş.
- 1.000–10.000 ajan için benchmark, Flecs çok iş parçacıklı çalıştırma ölçümleri.
- İsteğe bağlı deterministik mod (sabit adım, replay/multiplayer için).

### Faz 5 — Çoklu ajan davranışları
- Çarpışma önlemenin devamı: yatay çözüm (sağa dönüş), ajanı olmayan trafik (oyuncu uçağı,
  `TrackedPose`), helikopterler için kaçınma.
- Formasyon değişimi (dizilim geçişleri), lider değişimi, slot yeniden atama.
- Çok geniş haritalar (> 500 km) için ajan başına projeksiyon merkezini kaydırma.
- Kalkış/iniş sıralaması (aynı pistte birden fazla uçak).

### Faz 6 — Ek model tipleri
- Helikopterde motor arızası ve otorotasyon.
- VTOL / tiltrotor (multirotor ↔ sabit kanat geçişi).
- İsteğe bağlı 6-DOF sabit kanat modeli: otopilot çıkışını (yatış, uçuş yolu açısı, gaz)
  kumanda yüzeylerine çeviren bir iç döngü ile aynı arayüzü (`FlightModel` concept) uygular.

### Faz 7 — Araçlar
- Debug çizimi: rota, L1 hedef noktası, formasyon slotları, arazi tabanı.
- ImGui/ImPlot paneli: canlı grafikler (irtifa, hız, yatış, gaz).
- CSV kayıt + Python ile çizim betikleri.

### Faz 8 — Gemi operasyonları
Helikopter ve drone için iniş / kalkış hazır (Faz 0). Sıradakiler:
- **Sabit kanatlı İHA'nın gemiden kalkışı:** TB3 sınıfı bir preset; güverte boyunca kısa
  kalkış koşusu ve ski-jump. Güverte, gemiyle birlikte hareket eden bir pist olarak ele
  alınır.
- **Sabit kanatlı İHA'nın gemiye inişi:**
  - Hareketli piste yaklaşma: süzülüş yolu geminin rotasına ve hızına göre kayar.
  - Kısa iniş ve frenle duruş (kanca yok).
  - Uygun değilse pas geçip yeniden deneme (wave-off).
- **Güverte üstü rüzgâr:** gemi hızı ile rüzgârın bileşkesi. Helikopter iniş sınırları
  (yan / arka rüzgâr) ve gemiye "rüzgâra dön" komutu.
- **Gemi hareketi:** dalgada güvertenin yalpa, baş-kıç vurma ve dalıp çıkması. Güvertedeki
  araç bu tutumu izler; iniş için sakin an beklenir.
- **Birden çok araç:**
  - Geminin yanında tur atarak iniş sırası bekleme.
  - Boş spot bulan bir Flecs yardımcı fonksiyonu.
  - Kalkış sıralaması.
- **Güverte işleri:** güvertedeki aracı başka bir spota taşıma (`move_on_deck`). Hangar ve
  asansör oyun tarafında kalır.

## 4. Model değişimi

| Senaryo | Yöntem |
|---|---|
| Farklı uçak / gemi tipi | Farklı parametre seti (preset veya kendi `FixedWingParams`/`RotorcraftParams`/`ShipParams`); kod değişmez. |
| Çalışırken model/parametre değişimi (LOD) | `FlightAgent::snapshot()` → yeni ajan → komutu tekrar ver. |
| Yeni model tipi | `FlightModel` concept'ini sağlayan sınıf yazılır, `AgentVariant`'a eklenir; adım adım rehber: [`EXTENDING.md`](EXTENDING.md). |

## 5. Flecs güncelleme prosedürü

1. `cmake -DFM_FLECS_TAG=vX.Y.Z` ile derle.
2. Derleme hatası varsa yalnızca `adapters/flecs/src/flight_module.cpp` ve
   `adapters/flecs/include/fm/ecs/flight_module.hpp` güncellenir (çekirdek Flecs görmez).
3. `fm_flecs_tests` çalıştırılır.
4. Çalışma anındaki Flecs sürüm/kopya kontrolü host'ta yapılır; modül yapmaz.

## 6. Bilinen sınırlamalar

- Yerde çarpışma önleme yok; uçak yerinde dönemez (Faz 1).
- Kol sıralaması yalnızca kendi kolunu bilir: aynı pisti kullanan başka trafikle
  (ör. ayrı bir kol ya da yolcu uçağı) aralık ayarlamaz.
- Yan kayma (sideslip) yok; uçuş her zaman koordineli.
- Yerdeyken rüzgâr yok sayılır.
- Atmosfer modeli 32 km'ye kadar (ISA).
- Gemide rüzgâr, akıntı ve sığ su etkisi yok; dalga yalnızca görsel yalpa/baş-kıç
  vurmasıdır. Liman manevrası (yanaşma, römorkör) yok.
- Araçlar gemiyle çarpışmaz (helikopter adanın içinden geçebilir). Bu yüzden iniş yaklaşması
  geminin arkasından yapılır. Güvertedeki araç gemiyle yatıp kalkmaz.
- Arazi öngörüsü geçmiş örneklere dayanır; ani dik yamaçlarda (ör. uçurum) koruma sınırlıdır.
  Oyun ileride ileriye dönük arazi sorgusu sağlarsa `TerrainMonitor` buna genişletilebilir.
