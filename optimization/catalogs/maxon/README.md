# maxon motor supplement

Fifteen real winding/part-number variants from the official maxon product pages,
retrieved 2026-11-01. This supplements the CubeMars/Nanotec research in `../bldc/`.

| Family | Part numbers | Body diameter | Maximum body length | Shaft-to-shaft length |
|---|---|---:|---:|---:|
| EC-i 30, 30 W | 539472–539475 | 30 mm | 42 mm | 77 mm |
| EC-i 30, 45 W High Torque | 539480–539484 | 30 mm | 42 mm | 77 mm |
| EC-i 30, 50 W | 539476–539479 | 30 mm | 64 mm | 99 mm |
| EC-i 40, 50 W | 449463–449464 | 40 mm | 26 mm | 64.5 mm |

All are framed brushless motors with Hall sensors and an external controller.
The numbers above exclude cable bend space. Exact body and shaft dimensions
were visually checked against the official March 2025 dimension drawings on
printed pages 302, 303, 304 and 306. The source PDFs and rendered review images
are in `sources/`. The 50 W EC-i 30 body length is **64 mm**, not 42 mm.

`motors.csv` and `motors.json` contain current online numerical ratings, including
continuous torque, rated speed/current, stall torque, no-load speed, mechanical
maximum speed, rotor inertia and mass. Each row links its product page and
dimension PDF. Electrical data on provisional catalog pages can differ from
the current product page; the current product page takes precedence. For
example, 539477 currently lists 55.5 mN·m and 629 mN·m stall, while its catalog
page lists 55.2 and 624. These are preserved as documented source differences.

Family power is the manufacturer's assigned power class, not a recalculation
of a torque/speed point. Peak torque and permitted peak duration are left blank;
stall torque is recorded separately. Conversion: **1 g·cm² = 1e-7 kg·m²**.
Listed products and their specifications are verified; inventory and lead time
are not inferred. No pricing is used in optimization.

`extract.py` can refresh this supplement from the official product pages. It
uses the already reviewed drawing dimensions, so a new drawing revision should
be checked before changing those values. `sources/manifest.json` records hashes
of downloaded source files. The combined research and strictly complete motor
input subset are one directory above; active optimizer inputs are not changed.
