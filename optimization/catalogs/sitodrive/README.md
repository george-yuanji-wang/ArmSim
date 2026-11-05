# SITO harmonic reducer catalog

Retrieved **November 1, 2026**, from the manufacturer's public product pages and
[official strain-wave gear catalog](https://www.sitodrive.com/docs/Strain_Wave_Gear_Catalog_en.pdf).
The catalog contains manufacturer facts and their provenance. It is separate
from the simulator's validated input catalogs.

## Coverage

The complete public store inventory contains **66 reducer models and 243
selector-listed model/ratio choices**, of which **242 have matching store SKU
records**:

| Store family | Models | Selector-listed ratio choices |
|---|---:|---:|
| [HD](https://www.sitodrive.com/en/store/reducer/hd) | 12, including FD-03 | 38 |
| [HS](https://www.sitodrive.com/en/store/reducer/hs) | 20 | 88 |
| [CS](https://www.sitodrive.com/en/store/reducer/cs) | 10 | 44 |
| [CS-Mini](https://www.sitodrive.com/en/store/reducer/csmini) | 21 | 61 |
| [CD](https://www.sitodrive.com/en/store/reducer/cd) | 3 | 12 |
| **Total** | **66** | **243** |

The store's displayed **67** harmonic-reducer entries include one additional
customization/parts/payment listing. It is recorded in
`excluded_store_entries.csv`, rather than treated as a physical reducer.

The PDF additionally lists **CS Mini-2.5**, ratios 30 and 50, with a 5.9 g mass
and a dimensioned Ø11 mm case / 17.4 mm length. Those two variants are separately
recorded in `catalog_only.csv`; the headline says 11 × 11 mm while its
dimensioned drawing shows a circular Ø11 mm case, so that distinction is
retained. A current store listing or stock availability
was not found. The TA rotary actuators and FRC kits are separate product
categories and are excluded from the reducer inventory.

This covers every reducer in the captured public store and the additional
model in its linked PDF. It establishes public listing coverage, not inventory
on hand, unlisted custom products, or guaranteed procurement availability.

## Files

- `reducer_models.csv`: one row per physical store model, with dimensions,
  weight qualifiers, ratios, interface classification and drawing links.
- `reducer_variants.csv`: one row per selectable model/ratio, with rated,
  repeated-peak, average and momentary-peak output torque kept distinct;
  maximum/average input speed; published inertia and backlash.
- `catalog_only.csv`: the additional CS Mini-2.5 variants from the PDF.
- `source_conflicts.csv`: each numeric difference between the official product
  page and its matching official PDF table/envelope. Neither source is silently
  overwritten.
- `catalog_sources.json`: extracted product tables, selectors, source URLs,
  source hashes and exact category inventory counts.
- `catalog_pdf_facts.json`: the independently extracted PDF dimensions and
  ratings used for the comparison.
- `source_verification.json`: fresh HTTP status, matching product heading and
  response hash for every product URL, plus the official PDF signature/hash.
- `coverage.json`: machine-readable record counts and missing-field counts.
- `reducer_selection.json`: the current study's selected HD-14, CD-20 and
  CS-11-2XH-A units at 30:1, with fixed joint assignments and model-specific
  drawing facts checked November 4, 2026.
- `reducer_sources/`: retained official PDFs and product-page snapshots for
  those three units. These manufacturer assets are kept in this SITO folder.

CSV numeric fields contain plain numbers; blanks mean unpublished or
unresolved, not zero. Values retain the precision supplied by SITO.
`catalog_pdf_page` means the **1-based file page**, which differs from printed
page numbering; for example, HS-25-II is file page 41 / printed page 48.

## Definitions and important source differences

`diameter_mm`, `length_mm` and `mass_kg` contain the product-page values.
Their adjacent qualifier fields distinguish a nominal published value from an
upper bound such as ≤0.374 kg. A maximum weight must not be presented as an
exact measured weight. CS-Mini page envelopes are often common upper bounds
across different input/output variants; their individual drawings should be
checked before geometry import.

For example, HS-25-II's product page reports 110 mm diameter, 37.3 mm thickness
and ≤1.25 kg. Its official catalog drawing instead reports **115 mm, 55.5 mm,
and ≤1.663 kg**. Both sets are retained and flagged for manufacturer/drawing
confirmation. CS-Mini thickness bounds also differ between product pages and
the PDF; these can describe different envelope scopes or revisions.

The product page calls CS-03's dimension a 13 mm diameter, but the catalog shows
a **13 × 13 mm square section**. The square dimensions are retained separately;
using a 13 mm cylinder would underestimate its corner envelope.

The current simulation selection uses HD-14 on J1, CD-20 on J2/J3 and
CS-11-2XH-A on J4/J5/J6. The selected CS-11-2XH-A drawing establishes a 32 mm
axial envelope and a 58 mm bounding flange circle; its model-specific facts
and the generic product-page discrepancy remain in `reducer_selection.json`.
Regenerate the study inputs in `optimization/data/real/` with
`uv run python -m optimization.real_catalog`, then run them with
`uv run python run_optimization.py --real`. The broader supplier inventory
remains research; it is not the set of selectable reducers for this study.

Three listed variants have no published rated torque on their product pages:
**FD-03, ratios 30/50**, and **HD-08, ratio 80**. A generic table header listing
30/50/80/100/120 does not prove all five ratios are offered. Variant rows follow
the actual product selectors and retain missing rating cells. FD-03 ratio 30
appears in the selector but has no matching embedded SKU record; its
availability is explicitly unconfirmed. The ratio-50 FD-03 SKU is present.

Rated torque at the table's 2000 rpm point, average torque limit, repeated peak
and momentary peak are different ratings. Peak ratings must not be substituted
for a continuous output rating. This extraction preserves those meanings.

`published_inertia_kg_m2` converts the table's `I × 10^-4 kg·m²` values into SI,
including values already written in scientific notation. The table does not
explicitly identify the reference side on each product page, so
`inertia_reference` preserves that limitation. It must not automatically be
added as motor-side rotor inertia or reflected twice.

**Maximum allowable motor-input torque is not published in these captured
product tables.** `max_input_torque_Nm` therefore remains blank for every row.
Output torque divided by reduction ratio is not a manufacturer-published input
torque rating. Adapter mass/dimensions, efficiencies and detailed mounting
compatibility are also not supplied by this extraction.

The HD/HS descriptions identify modular output-bearing units; CS describes
four-point contact output bearings. Miniature variants and CD units retain
conservative descriptions where bearing/interface details require their
drawings. No bare component set or integrated motor actuator is silently
treated as the same assembly type.

## Reproduce or inspect

`collect_site.py` follows observed inventory links with pagination expanded to
100 entries, checks each category's displayed count, and caches source HTML.
`build_catalog.py` normalizes facts and compares the PDF. `verify_sources.py`
checks every observed product URL and the PDF download.

The research scripts use the analysis runtime's `lxml` and `pypdf`; they add no
dependencies to the ArmSim simulation environment. Local downloaded HTML/PDF
and image previews are ignored by Git. Final CSV/JSON facts and their source
URLs remain in this folder.
