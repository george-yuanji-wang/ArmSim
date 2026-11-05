# BLDC motor manufacturer survey

Retrieved **2026-11-01**. This is a sourced research catalog, separate from the
optimizer's validated `data/motor.csv`. It contains **22 voltage-specific records
for 18 physical model/connector variants** from CubeMars and Nanotec. It is a
selection of relevant motors, not a claim to cover every BLDC motor on sale.

`motors.csv` is the flat table; `motors.json` contains the same records. Numeric
cells use the units in their headers. A blank means the source did not establish
that value. `excluded_motors.csv` records seven specific size exclusions.

## Size and construction

- The inclusion filter is a **30–70 mm cylindrical body envelope** and a body
  axial length **strictly less than 100 mm**. Motor model numbers are not used as
  dimensions. For example, RI50 is actually 54 mm OD; RI70 is 76 mm OD and is
  excluded; RI75-PH is 70 mm OD and is included.
- The 42 x 42 mm square Nanotec flanges are represented by a conservative
  **59.397 mm bounding circle**, calculated as `sqrt(42^2+42^2)`. Their
  `outer_diameter_mm` cell stays blank because the source specifies a square.
  Nominal drawing tolerances remain in the source drawings.
- The length filter uses **body length**, excluding protruding shafts and wires.
  Nanotec DB42 front/rear shaft dimensions establish a separate overall length.
  In particular, DB42L01 has an 80.3 mm body but is **117.8 mm shaft to shaft**.
  It meets the body-only filter and exceeds a 100 mm complete axial envelope.
- CubeMars RI/RO entries are **frameless rotor/stator sets**. Their published
  masses do not include a custom bearing/housing/encoder installation. Nanotec
  entries are framed motors. No integrated geared actuator output rating is
  presented as bare motor-shaft torque.

## Ratings and unknowns

Rated and peak torque are separate. Published peak torque is **not** copied into
stall torque. Peak current is not converted into torque where the manufacturer
does not publish the torque rating. Peak-duration and mechanical maximum-speed
fields remain blank for all these records. No-load speed is not a substitute for
a mechanical speed limit.

RI50 and RI60 have separate 24/36/48 V rows for one physical winding. The speeds
are voltage-specific; the family comparison power of 66/85 W has no explicit
voltage basis, so `rated_power_W` is left blank for those operating points.
Power can later be calculated from rated torque and rated speed if explicitly
labeled as a derived quantity.

The RI75-PH Core Data/comparison gives 3360 rpm at 48 V, while its detailed table
also lists 1602 rpm at 24 V. These are preserved in their stated voltage context.
The same page conflicts on phase resistance and inductance (453 vs 495 mOhm,
485 vs 673 uH); those optional fields are left blank and both values are noted.

Rotor inertia is converted with **1 g·cm² = 1e-7 kg·m²**. Nanotec individual
drawings take priority over rounded family-overview figures: DB42L01's drawing
specifies 6.68e-6 kg·m². Resistance/inductance values retain each manufacturer's
phase or phase-to-phase convention; they are not silently normalized between
winding connections.

Ambient limits are recorded when published. Continuous-rating heatsink and
mounting conditions remain unestablished where the selected source omits them.
Availability means a manufacturer listing/document was found, not verified stock.

## Source provenance

Every row has official URLs, a source locator and retrieval date. `sources/`
holds the downloaded official HTML/PDF snapshots and a SHA-256 manifest. Scanned
Nanotec specification drawings and the DBL42 diameter drawing were rendered and
visually checked. The PDF pages are primary manufacturer documents.

Main official sources:

- [CubeMars RI50](https://www.cubemars.com/product/ri50-kv100-frameless-inrunner-torque-motor.html)
- [CubeMars RI60](https://www.cubemars.com/product/ri60-kv120-frameless-inrunner-torque-motor.html)
- [CubeMars RI30](https://www.cubemars.com/product/ri30-kv260-frameless-torque-motor.html)
- [CubeMars RI75-PH](https://www.cubemars.com/product/ri75-ph-kv70-potting-frameless-torque-motor.html)
- [CubeMars RO40 Lite](https://www.cubemars.com/product/ro40-kv140-lite-without-hall-frameless-torque-motor.html)
- [CubeMars RO50 Lite](https://www.cubemars.com/product/ro50-kv108-lite-without-hall-frameless-torque-motor.html)
- [Nanotec DB42 overview](https://www.nanotec.com/fileadmin/files/Baureihenuebersichten/BLDC_Motoren/Product_Overview_DB42.pdf)
- [Nanotec DB43 overview](https://www.nanotec.com/fileadmin/files/Baureihenuebersichten/BLDC_Motoren/Product_Overview_DB43.pdf)
- [Nanotec DBL42 overview](https://www.nanotec.com/fileadmin/files/Baureihenuebersichten/BLDC_Motoren/Product_Overview_DBL42.pdf)

`collect_sources.py` downloads the snapshots; `build_catalog.py` records the
verified facts and unit conversions reproducibly. Neither changes optimizer
inputs or fills unavailable motor ratings.
