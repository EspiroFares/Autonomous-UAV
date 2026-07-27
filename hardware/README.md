# Hardware — Bill of Materials

A 3-inch ducted quadcopter that carries its own companion computer. Everything
below is on (or directly part of) the aircraft, grouped by subsystem. The radio
and workshop tools are listed separately at the end. Prices are approximate, in
SEK at build time.

## Airframe & propulsion

| Part | Component | Qty |
|------|-----------|-----|
| Frame | GEP-CL30 (3", ducted) | 1 |
| Flight controller (AIO, ESC included) | JHEMCU GHF405AIO | 1 |
| Motors | LANNRC 1404 PLUS | 4 |
| Propellers | Gemfan D76 5-blade PC ducted | 4 (+ spares) |

## Autonomy — onboard compute & sensors

| Part | Component | Qty |
|------|-----------|-----|
| Companion computer | Raspberry Pi 4 | 1 |
| Camera | Raspberry Pi Camera v2 | 1 |
| Camera cable | Pi Camera FPC, 30 cm | 1 |
| Altitude rangefinder (Z) | Benewake TF-Luna LiDAR | 1 |
| Optical-flow sensor (X/Y, GPS-denied) | Holybro PMW3901 | 1 |
| Pi cooling | Chipset heatsink | 1 |

## Radio & power

| Part | Component | Qty |
|------|-----------|-----|
| Receiver | ExpressLRS EP2 | 1 |
| Transmitter | RadioMaster Pocket (ELRS) | 1 |
| Flight battery | Tattu 4S 850 mAh (XT60) | 2 |
| 5 V rail / BEC (powers the Pi) | UBEC 5V 5A | 1 |

## Mounting & wiring

| Part | Component | Qty |
|------|-----------|-----|
| FC power lead | XT60 pigtail | 1 |
| Wire | 22 AWG silicone | ~5 m |
| Standoffs | Nylon spacers M2 / M2.5 | kit |
| Insulation | Heat-shrink tube | kit |
| Vibration mount | Acrylic foam tape | 1 |

**Approximate aircraft cost:** ~3,700 SEK (parts on the drone; excludes radio
and tools).

<details>
<summary><b>Radio & tools / consumables</b> (not part of the aircraft)</summary>

<br>

Ground-side gear and workshop items used to build and fly it — listed for
completeness, not part of the airframe:

- **Radio power:** 2× 18650 cells + charger (for the RadioMaster Pocket)
- **Soldering:** soldering iron (ANENG SL108), solder, flux, brass tip cleaner
- **Assembly:** helping-hands, wire stripper, screwdriver set, digital caliper,
  threadlocker
- **Batteries:** LiPo charger (SkyRC B6 Neo)

</details>

---

_Notes: the GHF405AIO is an all-in-one board, so the ESC is integrated (no
separate ESC). Altitude comes from the downward TF-Luna, horizontal velocity
from the optical-flow sensor — the vehicle flies fully GPS-denied indoors._
