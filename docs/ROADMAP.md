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
| ECS | Çekirdek Flecs'ten bağımsız, ince Flecs modülü | Flecs güncellemesi çekirdeği etkilemez; sürüm farkları tek dosyada (`compat.hpp`). |
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
  kanat İHA) ve multirotor, hazır parametre setleriyle.
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
- **Flecs modülü:** 4.0.5, 4.1.2 ve 4.1.6 ile derlenip test edildi.
- **Plugin DLL desteği:** `fm::ecs::load()` (yanlış bağlanmış Flecs kopyasını yakalar),
  `fm::ecs::unload()` (sistemleri ve component'leri temizler, DLL güvenle boşaltılır),
  `FM_FLECS_TARGET` / `FM_FLECS_SHARED`, gerçek yükle–boşalt–tekrar yükle testi.
- **İniş:** pist yaklaşması, süzülüş yolu, pas geçme, flare, teker koyma, frenleme.
- **Duraklatma:** dünya çalışırken tüm ajanları (`set_paused`) veya tek ajanı (`pause`)
  durdurma; uzun karelerde `max_step` sınırı.
- **Konum değişimi (teleport):** oyun Transform'u değiştirince ajan yeni konumdan devam
  eder (`FlightAgent::relocate`, Flecs'te otomatik algılama, `bind_geo_transform`).
  Küçük düzeltmeler görevi bozmaz; büyük sıçramalarda rota bacağı, düz uçuş hattı,
  yaklaşma yeni konumdan yeniden kurulur; yer/hava geçişleri ve saçma değerler ele alınır.
- **Testler:** 46 çekirdek + 16 Flecs + 2 plugin testi; AddressSanitizer/UBSan temiz.

## 3. Sonraki fazlar

### Faz 1 — Sabit kanat iniş ve yer hareketleri
- [x] Yaklaşma: pist ekseni uzantısına konumlanma, final yaklaşma fiksi, 3° süzülüş yolu
  (alttan yakalama + dikey hız ileri besleme), flap/iniş takımı, yaklaşma hızı 1.3 Vs.
- [x] Karar yüksekliğinde stabilite kontrolü, pas geçme ve otomatik yeni deneme.
- [x] Gecikme telafili flare, yumuşak teker koyma (0.4–1.0 m/s), frenleme, park durumu.
- [x] Rota sonunda iniş (`Route::runway` + `EndBehavior::Land`), inişten sonra tekrar kalkış.
- [ ] Yerde taksi: taksi yolu (waypoint listesi) üzerinde düşük hızlı hareket ve dönüş.
- [ ] Yerdeyken rüzgâr etkisi, yan rüzgârda teker koymadan önce crab'dan çıkma (decrab).
- [ ] Pist uzunluğu ve pist sonu kontrolü (kısa pistte pas geçme / kalkış iptali).

### Faz 2 — Veri odaklı parametreler
- Uçak tiplerini JSON'dan okuma (glaze veya nlohmann/json), hot-reload.
- Flecs prefab entegrasyonu: tip başına prefab, entity'ler `is_a` ile parametre paylaşır.
- CAS/TAS dönüşümü; hızları gösterge hızı (IAS) olarak verebilme.
- Yakıt tüketimi ve kütle değişimi.

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
- Ajanlar arası çarpışma önleme (ayrılma kuralları).
- Formasyon değişimi (dizilim geçişleri), lider değişimi, slot yeniden atama.
- Takipçisi olan liderin dönüşlerde yatışı sınırlaması (dış taraftaki uçaklar dağılmasın).
- Çok geniş haritalar (> 500 km) için ajan başına projeksiyon merkezini kaydırma.
- Kalkış/iniş sıralaması (aynı pistte birden fazla uçak).

### Faz 6 — Ek model tipleri
- Tek rotorlu helikopter (otorotasyon olmadan, oyun seviyesinde).
- VTOL / tiltrotor (multirotor ↔ sabit kanat geçişi).
- İsteğe bağlı 6-DOF sabit kanat modeli: otopilot çıkışını (yatış, uçuş yolu açısı, gaz)
  kumanda yüzeylerine çeviren bir iç döngü ile aynı arayüzü (`FlightModel` concept) uygular.

### Faz 7 — Araçlar
- Debug çizimi: rota, L1 hedef noktası, formasyon slotları, arazi tabanı.
- ImGui/ImPlot paneli: canlı grafikler (irtifa, hız, yatış, gaz).
- CSV kayıt + Python ile çizim betikleri.

## 4. Model değişimi

| Senaryo | Yöntem |
|---|---|
| Farklı uçak tipi | Farklı parametre seti (preset veya kendi `FixedWingParams`/`RotorcraftParams`); kod değişmez. |
| Çalışırken model/parametre değişimi (LOD) | `FlightAgent::snapshot()` → yeni ajan → komutu tekrar ver. |
| Yeni model tipi | `FlightModel` concept'ini sağlayan sınıf yazılır, `FlightAgent` içindeki `std::variant`'a eklenir. |

## 5. Flecs güncelleme prosedürü

1. `cmake -DFM_FLECS_TAG=vX.Y.Z` ile derle.
2. Derleme hatası varsa yalnızca `adapters/flecs/include/fm/ecs/compat.hpp` ve gerekirse
   `adapters/flecs/src/flight_module.cpp` güncellenir.
3. `fm_flecs_tests` çalıştırılır.
4. Major sürüm (5.x) için `compat.hpp` içindeki sürüm kontrolü genişletilir.

## 6. Bilinen sınırlamalar

- Taksi yok; pistten çıkış/park yeri yok (Faz 1).
- Yan kayma (sideslip) yok; uçuş her zaman koordineli.
- Yerdeyken rüzgâr yok sayılır.
- Atmosfer modeli 20 km'ye kadar (ISA).
- Arazi öngörüsü geçmiş örneklere dayanır; ani dik yamaçlarda (ör. uçurum) koruma sınırlıdır.
  Oyun ileride ileriye dönük arazi sorgusu sağlarsa `TerrainMonitor` buna genişletilebilir.
