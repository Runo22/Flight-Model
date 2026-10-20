# Hazır modeller (presets)

İki tür preset var:

* **Referans sınıfı presetler**: belirli bir uçağın yayımlanmış performansına göre ayarlandı
  ve ölçülerek doğrulandı. Oyunda "gerçekçiye yakın" davranması gereken araçlar için.
* **Genel sınıf presetler** (`fast_jet`, `airliner_jet`, `regional_turboprop`,
  `light_propeller`, `fixed_wing_uav`, `multirotor`): sınıfı temsil eden makul değerler,
  belirli bir uçağa göre ayarlanmadı.

| Preset | Referans | Sınıf |
|---|---|---|
| `single_engine_fighter()` | F-16C Block 50 | Tek motorlu, afterburner'lı savaş uçağı |
| `twin_engine_fighter()` | F-15C | Çift motorlu, afterburner'lı savaş uçağı |
| `turboprop_trainer()` | TUSAŞ Hürkuş | Yüksek performanslı turboprop eğitim / hafif taarruz |
| `jet_uav()` | Bayraktar Kızılelma | Jet motorlu insansız savaş uçağı (ses altı) |
| `attack_helicopter()` | TUSAŞ T129 ATAK | Hafif / orta taarruz helikopteri |
| `heavy_attack_helicopter()` | AH-64D Apache | Ağır taarruz helikopteri |
| `transport_helicopter()` | CH-47F Chinook | Ağır nakliye helikopteri (tandem rotor), azami ağırlıkta |

## Nasıl ayarlandı

1. **Veri:** kütle, kanat alanı / rotor çapı, motor itkisi / gücü ve yayımlanmış performans
   (azami hız, tırmanma, tavan, stall, kalkış/iniş mesafesi) toplandı. Kaynaklar aşağıda.
2. **Uydurma:** bilinmeyen fiziksel katsayılar (sürükleme poları, itki/güç kaybının irtifa
   ve Mach ile değişimi, helikopterde profil gücü, parazit sürükleme alanı, şanzıman sınırı)
   aynı formüllerle birden çok yayımlanmış değere aynı anda uyduruldu. Gerçekçi olmayan
   değerlere kaçmaması için katsayılar gerçek uçaklarda görülen aralıklarla sınırlandı
   (ör. F-16 için CD0 ~0.017, indüklenen sürükleme k 0.14-0.20).
3. **Ölçüm:** presetler gerçek agent'lar üzerinden uçurularak ölçüldü
   (`tests/performance.hpp`); aşağıdaki tablolar `fm_performance_report` çıktısıdır.
4. **Test:** `tests/test_presets.cpp` önemli değerleri tolerans içinde kontrol eder; ayrıca
   her preset kalkış, rota, iniş (helikopterlerde noktaya gidip iniş ve kol uçuşu) yapar.

Tabloyu yeniden üretmek için:

```sh
./build/tests/fm_performance_report                    # hepsi
./build/tests/fm_performance_report turboprop_trainer  # tek preset
```

### Uçuş zarfı ile yapay zekâ pilotu sınırları

Fizik (kütle, sürükleme, itki, güç) uçağın **yapabildiğini** belirler: azami hız, tırmanma,
tavan. Ayrı parametreler yapay zekâ pilotunun **normalde ne kadarını kullandığını**
belirler. Örneğin savaş uçakları 9 g'ye dayanır ama AI 7 g ve 70° yatışla sınırlıdır.
F-16 50.000 ft/dk tırmanabilir ama irtifa değiştirirken 25° ve 120 m/s ile tırmanır.
Bu sınırlar (`load_factor_max`, `bank_max`, `climb_angle_max`, `climb_rate_max`,
`descent_*`) oyunun ihtiyacına göre değiştirilebilir. Ölçülen performans değişmez,
yalnızca AI'nin davranışı değişir.

## Sabit kanat

### `single_engine_fighter()` — F-16C Block 50 sınıfı, 12.000 kg

Veri: kanat alanı 300 ft² (27,87 m²), hava-hava yüklü kütle 26.463 lb, F110-GE-129
(afterburner'la ~29.000 lbf), 40.000 ft'te Mach 2,05, servis tavanı 50.000+ ft, tırmanma
~50.000 ft/dk, hız sınırı 800 KCAS. CLmax 1,6 = F-16'nın 25° hücum açısı sınırı.

| | Measured | Reference |
|---|---|---|
| Top speed at 40,000 ft | 1124 kt TAS (M1.96) | M2.05 |
| Top speed at sea level (with limits) | 800 kt | 800 KCAS (limit) |
| Best climb at sea level | 48,015 ft/min at 600 kt | 50,000 ft/min |
| Service ceiling (100 ft/min) | 58,728 ft | 50,000+ ft |
| Stall speed, clean / landing flaps | 128 / 117 kt |  |
| Takeoff: ground roll, lift-off, over 50 ft | 352 m, 162 kt, 505 m |  |
| Landing: approach, touchdown, roll, from 50 ft | 149 kt, 131 kt, 733 m, 1143 m |  |
| AI turn at cruise speed, 10,000 ft | 6.5 deg/s at 70 deg bank |  |

Tepe hız, gerçekçi sürükleme değerleriyle %4 düşük kaldı. Sürüklemeyi gerçek dışı küçültmek
yerine bu bırakıldı. Kalkış afterburner'la yapılır.

### `twin_engine_fighter()` — F-15C sınıfı, 20.200 kg

Veri: kanat alanı 608 ft² (56,5 m²), yüklü kütle 44.500 lb, 2× F100-PW-220 (her biri
afterburner'la 23.770 lbf), yüksek irtifada Mach 2,5+, alçakta Mach 1,2, tavan
60.000-65.000 ft, tırmanma > 50.000 ft/dk. KAAN boyutunda çift motorlu bir uçak için de iyi
bir başlangıç.

| | Measured | Reference |
|---|---|---|
| Top speed at 40,000 ft | 1352 kt TAS (M2.36) | M2.5+ |
| Top speed at sea level (with limits) | 800 kt | M1.2 (800 KCAS) |
| Best climb at sea level | 53,262 ft/min at 611 kt | > 50,000 ft/min |
| Service ceiling (100 ft/min) | 66,802 ft | 60,000-65,000 ft |
| Stall speed, clean / landing flaps | 122 / 113 kt |  |
| Takeoff: ground roll, lift-off, over 50 ft | 316 m, 153 kt, 465 m |  |
| Landing: approach, touchdown, roll, from 50 ft | 147 kt, 131 kt, 724 m, 1139 m |  |
| AI turn at cruise speed, 10,000 ft | 6.2 deg/s at 70 deg bank |  |

Tepe hız %6 düşük; sebep F-16'daki ile aynı.

### `turboprop_trainer()` — TUSAŞ Hürkuş sınıfı, 3.300 kg

Veri: kanat alanı 16,32 m² (EASA tip sertifikası), azami kalkış ağırlığı 3.300 kg,
PT6A-68T 1.600 shp, azami hız 310 KCAS, azami seyir 288 KTAS, deniz seviyesinde tırmanma
4.370 ft/dk, tavan 34.700 ft (çoğu kaynak) / 39.270 ft (TUSAŞ broşürü), iniş düzeninde
stall 77 KCAS, 50 ft engeli aşarak kalkış 489 m, 50 ft'ten iniş 593 m, +7 / -3,5 g.

| | Measured | Reference |
|---|---|---|
| Top speed at 30,000 ft | 289 kt TAS (M0.49) | 288 KTAS max cruise |
| Top speed at sea level (with limits) | 310 kt | 310 KCAS |
| Best climb at sea level | 4414 ft/min at 127 kt | 4,370 ft/min |
| Service ceiling (100 ft/min) | 35,852 ft | 34,700-39,270 ft |
| Stall speed, clean / landing flaps | 92 / 78 kt | 77 KCAS landing config |
| Takeoff: ground roll, lift-off, over 50 ft | 319 m, 94 kt, 452 m | 489 m over 50 ft |
| Landing: approach, touchdown, roll, from 50 ft | 99 kt, 91 kt, 377 m, 659 m | 593 m from 50 ft |
| AI turn at cruise speed, 10,000 ft | 7.6 deg/s at 60 deg bank |  |

Azami seyir hızı (288 KTAS) uydurmada kullanılmadı. Model 30.000 ft'te 289 kt verdi;
bu, ayardan bağımsız bir doğrulama.

### `jet_uav()` — Bayraktar Kızılelma sınıfı, 6.000 kg

Veri: azami kalkış ağırlığı 6.000 kg (2022; daha yeni kaynaklar 8.500 kg), faydalı yük
1.500 kg, kanat açıklığı 10 m, azami hız Mach 0,9 (900 km/sa), seyir Mach 0,6, tavan
12.000 m (2022) / 14.000 m. **Varsayımlar** (kamuya açık veri yok): kanat alanı 30 m²,
motor TF6000 sınıfı 26,7 kN itki, afterburner yok. Bu preset diğerlerinden daha
belirsizdir.

| | Measured | Reference |
|---|---|---|
| Top speed at 30,000 ft | 527 kt TAS (M0.89) | M0.9 |
| Top speed at sea level (with limits) | 467 kt |  |
| Best climb at sea level | 9513 ft/min at 353 kt |  |
| Service ceiling (100 ft/min) | 41,523 ft | 12,000-14,000 m |
| Stall speed, clean / landing flaps | 100 / 91 kt |  |
| Takeoff: ground roll, lift-off, over 50 ft | 560 m, 128 kt, 685 m |  |
| Landing: approach, touchdown, roll, from 50 ft | 119 kt, 111 kt, 642 m, 1047 m |  |
| AI turn at cruise speed, 10,000 ft | 3.7 deg/s at 50 deg bank |  |

## Helikopterler

Helikopterler `RotorcraftParams`'ın helikopter kipiyle çalışır (`engine_power` ve
`rotor_disc_area` dolu). Bu kipte:

* Performans güçten hesaplanır: indüklenen güç (momentum teorisi, yer etkisi dahil), profil
  gücü ve parazit güç; motor ve şanzıman sınırına karşı.
* Azami hız, hıza göre tırmanma (en iyi tırmanma 60-75 kt civarında) ve askıda kalma ve
  servis tavanları buradan çıkar ve irtifayla değişir.
* Hızlı uçuşta burun, sürüklemeye göre aşağı eğilir.
* Seyirde yön değişimi hızı koruyan yatışlı dönüştür; rota köşeleri yay çizerek alınır.
* Yana ve geri uçuş hızı sınırlıdır.
* AI tırmanmak için gerekirse yavaşlar, tıpkı bir pilot gibi.

### `attack_helicopter()` — TUSAŞ T129 ATAK sınıfı, 5.000 kg

Veri: azami kalkış ağırlığı 5.000 kg, 2× LHTEC CTS800-4A (her biri 1.014 kW), rotor çapı
11,9 m, azami hız 150 kt (278-281 km/sa), seyir 145 kt, tırmanma 14 m/s, servis tavanı
6.096 m, OGE askı tavanı ~3.048 m (kaynağı zayıf).

| | Measured | Reference |
|---|---|---|
| Top speed, sea level / 3,000 m | 152 / 153 kt | 150 kt |
| Best climb | 2746 ft/min at 61 kt | 2,750 ft/min (14 m/s) |
| Vertical climb (out of ground effect) | 2116 ft/min |  |
| Hover ceiling out of ground effect | 2971 m | ~3,050 m |
| Service ceiling | 6012 m | 6,096 m |
| Pitch at cruise speed / top speed | -7.9 / -9.2 deg |  |
| 0 to 100 kt | 17 s, 20 deg nose down |  |
| Stop from cruise speed | 52 s, 1506 m |  |
| Turn at cruise speed | 5.5 deg/s, 34 deg bank, radius 753 m |  |

### `heavy_attack_helicopter()` — AH-64D Apache sınıfı, 8.000 kg

Veri: brüt kütle 17.650 lb (8.000 kg), 2× T700-GE-701C (her biri 1.890 shp), rotor çapı
14,63 m, azami hız 158 kt (Vne 197 kt), seyir 143 kt, tırmanma 2.500 ft/dk, dikey tırmanma
1.775 ft/dk, servis tavanı 21.000 ft. OGE askı tavanı (~3.100 m) yayımlanmış bir değer
değil, tahmindir.

| | Measured | Reference |
|---|---|---|
| Top speed, sea level / 3,000 m | 159 / 165 kt | 158 kt |
| Best climb | 2528 ft/min at 67 kt | 2,500 ft/min |
| Vertical climb (out of ground effect) | 1760 ft/min | 1,775 ft/min |
| Hover ceiling out of ground effect | 3076 m | ~3,100 m (estimate) |
| Service ceiling | 6451 m | 6,400 m |
| Pitch at cruise speed / top speed | -7.5 / -9.3 deg |  |
| 0 to 100 kt | 18 s, 18 deg nose down |  |
| Stop from cruise speed | 58 s, 1685 m |  |
| Turn at cruise speed | 5.4 deg/s, 35 deg bank, radius 787 m |  |

### `transport_helicopter()` — CH-47F Chinook sınıfı, 22.680 kg (azami brüt)

Veri: azami brüt ağırlık 50.000 lb, 2× T55-GA-714A (her biri 4.777 shp), iki adet 18,29 m
rotor (tandem, kuyruk rotoru yok), azami hız 170 kt, seyir 155-160 kt, tırmanma
1.522 ft/dk, tavan 20.000 ft. Preset azami ağırlıktadır ("büyük ve hızlı olması gerekmeyen"
helikopter). Deniz seviyesinde OGE ancak askıda kalır, yer etkisiyle rahat kalkar.
Daha hafif bir yük için `mass` düşürülürse (ör. 18.000 kg) model ~160 kt ve
~2.700 ft/dk verir.

| | Measured | Reference |
|---|---|---|
| Top speed, sea level / 3,000 m | 155 / 162 kt | 155-160 kt cruise, 170 kt Vne |
| Best climb | 1702 ft/min at 74 kt | 1,522 ft/min |
| Vertical climb (out of ground effect) | 90 ft/min |  |
| Hover ceiling out of ground effect | 1723 m |  |
| Service ceiling | 5432 m | 6,100 m (lighter) |
| Pitch at cruise speed / top speed | -4.8 / -5.8 deg |  |
| 0 to 100 kt | 28 s, 12 deg nose down |  |
| Stop from cruise speed | 86 s, 2521 m |  |
| Turn at cruise speed | 3.6 deg/s, 25 deg bank, radius 1134 m |  |

## Kendi uçağını ayarlamak

1. En yakın preset'ten başla (ör. `auto p = fm::presets::twin_engine_fighter();`).
2. Bilinen verileri doğrudan gir: `mass`, `wing_area`, `engine.max_thrust` /
   `engine.max_power`, helikopterde `engine_power`, `rotor_disc_area` (π r²).
3. Raporu kendi preset'in için çalıştır (`tests/performance_report.cpp`'ye bir satır ekle)
   ve aşağıdaki tabloya göre düzelt.

| Ölçülen | Sabit kanatta asıl etkileyen | Helikopterde asıl etkileyen |
|---|---|---|
| Azami hız (alçak) | `cd0`, itki / güç; operasyonel sınır `max_speed` (EAS) | `flat_plate_area`, `transmission_limit` |
| Azami hız (yüksek) | `engine.density_exponent`, `engine.mach_lapse`, `wave_drag`, `mach_critical`; sınır `mach_max` | `power_lapse_exponent` |
| Tırmanma | itki / güç fazlası; AI sınırı `climb_rate_max`, `climb_angle_max` | `profile_power`, `induced_power_factor`, güç |
| Tavan | `engine.density_exponent`, `induced_drag_k` | `power_lapse_exponent`, `profile_power` |
| Stall / yaklaşma hızı | `cl_max`, `flaps_dcl_max`, `approach_speed_factor` | — |
| Kalkış | `rotate_speed`, `rotation_pitch`, `cl_alpha`, `cl0` + `flaps_dcl0` | yer etkisi, güç fazlası |
| İniş mesafesi | `touchdown_distance`, `brake_deceleration`, `approach_speed_factor` | — |
| Seyirde burun açısı | `cl0`, `cl_alpha` | `flat_plate_area` |
| Dönüş | AI: `bank_max`, `load_factor_max` | `bank_max` (`max_tilt` en az 5° fazla olmalı) |
| İvmelenme | itki | `max_acceleration` (yalnız hız değişimi), `velocity_time_constant` |

## Kaynaklar

* Hürkuş: [GlobalSecurity](https://www.globalsecurity.org/military/world/europe/hurkus-specs.htm),
  [TUSAŞ](https://www.tusas.com/en/product/hurkus-en),
  [EASA tip sertifikası](https://easa.europa.eu/cs/downloads/21107/en),
  [Wikipedia](https://www.wikipedia.com/wiki/TAI_Hurkus)
* F-16C: [GlobalSecurity](https://www.globalsecurity.org/military/systems/aircraft/f-16-specs.htm),
  [USAF fact sheet](https://www.shaw.af.mil/About-Us/Fact-Sheets/Display/Article/663884/f-16c-fighting-falcon/)
* F-15C: [MILAVIA](https://milavia.net/aircraft/f-15/f-15_specs.htm),
  [MiGFlug](https://migflug.com/aircraft/f-15-eagle/)
* Kızılelma: [Wikipedia](https://wikipedia.com/wiki/Bayraktar_MIUS),
  [GlobalMilitary](https://www.globalmilitary.net/compare/aircraft/bayraktar-kizilelma-vs-flyeye/),
  [Airforce Technology](https://www.airforce-technology.com/?p=250868)
* T129 ATAK: [Army Technology](https://www.army-technology.com/projects/t129-attack-helicopter/),
  [GlobalMilitary](https://www.globalmilitary.net/aircraft/t129/)
* AH-64D: [helis.com](https://www.helis.com/database/model/AH-64D-Apache/),
  [Aeropedia](https://aeropedia.com.au/?p=23448)
* CH-47F: [MiGFlug](https://migflug.com/aircraft/ch-47-chinook/),
  [helis.com](https://www.helis.com/database/model/CH-47F-Chinook/)

Kaynaklar bazı değerlerde birbirini tutmuyor; tabloda kullanılan değer ve aralık yazıldı.
