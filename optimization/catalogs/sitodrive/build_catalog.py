"""Normalize extracted manufacturer facts into auditable research CSV tables.

CSV files are research/interchange data, not solver-ready substitutes for
unpublished motor-input limits or unconfirmed drawing envelopes.
"""

import csv
from decimal import Decimal
import hashlib
import json
import math
import re
from urllib.request import Request, urlopen
from collections import Counter
from pathlib import Path


ROOT = Path(__file__).resolve().parent
PDF_URL = "https://www.sitodrive.com/docs/Strain_Wave_Gear_Catalog_en.pdf"
FIELDS = (
    "rated_output_torque_at_2000rpm_Nm", "repeated_peak_output_torque_Nm",
    "average_output_torque_limit_Nm", "momentary_peak_output_torque_Nm",
    "max_input_speed_rpm", "average_input_speed_limit_rpm",
    "published_inertia_kg_m2", "backlash_max_arcsec",
)
LABELS = {
    "Rate Torque": FIELDS[0], "Limit for Repeated Peak Torque": FIELDS[1],
    "Limit for Average Torque": FIELDS[2], "Limit for Momentary Peak Torque": FIELDS[3],
    "Maximum Input Speed": FIELDS[4], "Limit for Average Input Speed": FIELDS[5],
    "Moment of Inertia": FIELDS[6], "Backlash": FIELDS[7],
}


def number(value):
    """Parse a published scalar; never replace an empty cell with zero."""
    value = value.strip().replace(" ", "").replace("≤", "").replace("Φ", "")
    if not value or value in {"/", "-"}:
        return None
    value = re.sub(r"(?:x|×)10\^?(-?\d+)", r"e\1", value, flags=re.I)
    return float(value)


def dimension_facts(text):
    text = text.replace("\x00", "")
    found = {}
    patterns = {
        "diameter": r"Diameter\s*:\s*Φ?\s*(≤?)\s*([\d.]+)(?:[×x]([\d.]+))?\s*mm",
        "length": r"Thickness\s*:\s*(≤?)\s*([\d.]+)\s*mm",
        "mass": r"Weight\s*:?\s*(≤?)\s*([\d.]+)\s*(kg|g)\b",
    }
    for field, pattern in patterns.items():
        match = re.search(pattern, text, re.I)
        if not match:
            found[field] = {"value": None, "qualifier": "not_published"}
            continue
        val = float(match.group(2))
        if field == "mass" and match.group(3).lower() == "g":
            val = float(Decimal(match.group(2))/Decimal(1000))
        found[field] = {"value": val, "qualifier": "upper_bound" if match.group(1) else "published_value"}
        if field == "diameter" and match.group(3):
            found[field]["second_dimension_mm"] = float(match.group(3))
            found[field]["shape"] = "rectangular"
    return found


def available_ratios(product):
    return sorted({int(value["value"]) for attribute in product["product_attributes"]
                   if attribute["name"].lower() == "speed ratio"
                   for value in attribute["values"]})


def web_ratings(product):
    if not product["tables"]:
        return {}
    table = product["tables"][0]
    header = next(row for row in table if row[0]["text"] == "Gear Ratio")
    result = {int(cell["text"]): {} for cell in header[1:]}
    for row in table:
        field = next((field for prefix, field in LABELS.items() if row[0]["text"].startswith(prefix)), None)
        if field:
            for header_cell, cell in zip(header[1:], row[1:]):
                value = number(cell["text"])
                if field == "published_inertia_kg_m2" and value is not None:
                    value = float(Decimal(str(value))*Decimal("0.0001"))
                result[int(header_cell["text"])][field] = value
    return result


def pdf_page_for(product):
    name = product["model"]
    hd = {"HD-03":13, "HD-05":14, "HD-08":15, "HD-11":16,
          "HD-14":17, "HD-17":18, "HD-20":19, "HD-25":20,
          "HD-11Pro":21, "HD-14Pro":22, "HD-17Pro":23}
    if name in hd:
        return hd[name]
    match = re.fullmatch(r"HS-(14|17|20|25|32)-(I|II|III|IV)", name)
    if match:
        return 28+4*(14,17,20,25,32).index(int(match.group(1)))+("I","II","III","IV").index(match.group(2))
    match = re.fullmatch(r"CS-(14|17|20|25|32)-(I|II)", name)
    if match:
        return 52+2*(14,17,20,25,32).index(int(match.group(1)))+("I","II").index(match.group(2))
    match = re.match(r"CS-(03|05|08|11)-", name)
    if match:
        return {"03":65, "05":66, "08":68, "11":70}[match.group(1)]
    if name.startswith("CD-"):
        return {"CD-14":75, "CD-17":76, "CD-20":77}[name]
    return None


def pdf_ratings(text, ratios):
    """Read column-major numeric tokens; HTML ratio placement supplies mapping.

    Every scalar token is checked, with seven fields when inertia is blank and
    eight otherwise. PDF pages are retained for independent visual verification.
    """
    if not ratios:
        return {}
    section = text.split("Gear Ratio",1)[1].split("Rate Torque",1)[0]
    lines = [line.strip() for line in section.splitlines() if line.strip()]
    tokens = [line for line in lines if re.fullmatch(r"≤?[\d.]+(?:(?:x|×)10\^-?\d+)?", line)]
    assert tokens[0] == "30", tokens[:4]  # Common PDF template header, including blank ratio columns.
    tokens = tokens[1:]
    width, remainder = divmod(len(tokens), len(ratios))
    if remainder or width not in (7,8):
        raise ValueError(f"Unexpected PDF table token count {len(tokens)} for {ratios}")
    fields = FIELDS if width == 8 else tuple(field for field in FIELDS if field != "published_inertia_kg_m2")
    result = {}
    for index, ratio in enumerate(ratios):
        values = [number(value) for value in tokens[index*width:(index+1)*width]]
        row = dict(zip(fields,values))
        if row.get("published_inertia_kg_m2") is not None:
            row["published_inertia_kg_m2"] = float(Decimal(str(row["published_inertia_kg_m2"]))*Decimal("0.0001"))
        result[ratio] = row
    return result


def family_and_type(product):
    name = product["model"]
    if name == "FD-03":
        return "FD (listed under HD)", "micro reducer; output bearing details not published"
    if name.startswith("HD-"):
        return "HD", "flat modular reducer; HD family catalog describes cross-roller output support"
    if name.startswith("HS-"):
        kind = {"I":"simplicity unit", "II":"hollow shaft", "III":"standard", "IV":"input shaft"}[name.rsplit("-",1)[1]]
        return "HS", kind+"; cross-roller output bearing unit"
    if name.startswith("CD-"):
        return "CD", "compact reducer; bearing/interface details require drawing review"
    if "csmini" in product["categories"]:
        return "CS-Mini", "mini reducer; variant interface requires its linked drawing"
    return "CS", ("simplicity unit" if name.endswith("-I") else "standard")+"; four-point contact output bearing unit"


def write_csv(name, rows):
    columns = list(dict.fromkeys(key for row in rows for key in row))
    with (ROOT/name).open("w",newline="",encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream,fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)


def load_pdf_pages():
    cache = ROOT/".cache/pdf_text.json"
    if cache.exists():
        return json.loads(cache.read_text(encoding="utf-8"))
    from pypdf import PdfReader
    pdf = ROOT/".cache/catalog.pdf"
    pdf.parent.mkdir(exist_ok=True)
    if not pdf.exists():
        with urlopen(Request(PDF_URL,headers={"User-Agent":"Mozilla/5.0"}),timeout=60) as response:
            content=response.read()
        if not content.startswith(b"%PDF"):
            raise ValueError("Official catalog response is not a PDF")
        pdf.write_bytes(content)
    result=[{"pdf_page":index+1,"text":page.extract_text()} for index,page in enumerate(PdfReader(pdf).pages)]
    cache.write_text(json.dumps(result,ensure_ascii=False),encoding="utf-8")
    return result


def main():
    source = json.loads((ROOT/"catalog_sources.json").read_text(encoding="utf-8"))
    pages = load_pdf_pages()
    products, variants, conflicts, pdf_facts, excluded = [], [], [], [], []
    for product in source["products"]:
        ratios = available_ratios(product)
        if not ratios:
            excluded.append({"product_id":product["product_id"],"listing":product["model"],
                             "reason":"Fee/payment entry, not a reducer", "source_url":product["url"]})
            continue
        family, kind = family_and_type(product)
        dims = dimension_facts(product["dimension_text"])
        ratings = web_ratings(product)
        page = pdf_page_for(product)
        pdf_dims, pdf_rates = {}, {}
        product_conflicts = []
        if page:
            text = pages[page-1]["text"]
            pdf_dims = dimension_facts(text)
            rated_ratios = sorted(ratio for ratio, values in ratings.items()
                                  if values.get(FIELDS[0]) is not None)
            pdf_rates = pdf_ratings(text, rated_ratios)
            pdf_facts.append({"model":product["model"], "pdf_page":page,
                              "dimensions":pdf_dims,"ratings":pdf_rates,"source_url":PDF_URL})
            for dimension in ("diameter","length","mass"):
                web = dims[dimension]["value"]
                pdf = pdf_dims[dimension]["value"]
                if web is not None and pdf is not None and not math.isclose(web,pdf,rel_tol=1e-10):
                    field = {"diameter":"diameter_mm","length":"length_mm","mass":"mass_kg"}[dimension]
                    product_conflicts.append(field)
                    conflicts.append({"model":product["model"], "gear_ratio":"all", "field":field,
                        "web_value":web,"web_qualifier":dims[dimension]["qualifier"],
                        "pdf_value":pdf,"pdf_qualifier":pdf_dims[dimension]["qualifier"],
                        "web_source_url":product["url"],"pdf_source_url":PDF_URL,"pdf_page":page,
                        "resolution":"Both published values retained; confirm applicable variant and envelope before simulation."})
        base = {"manufacturer":"SITO Drive", "series":family, "model":product["model"],
                "assembly_type":kind,
                "diameter_mm":dims["diameter"]["value"], "diameter_qualifier":dims["diameter"]["qualifier"],
                "length_mm":dims["length"]["value"], "length_qualifier":dims["length"]["qualifier"],
                "mass_kg":dims["mass"]["value"], "mass_qualifier":dims["mass"]["qualifier"],
                "cross_section_shape":"rectangular" if pdf_dims.get("diameter",{}).get("shape") else "nominal circular envelope; verify drawing",
                "square_width_mm": pdf_dims.get("diameter",{}).get("value") if pdf_dims.get("diameter",{}).get("shape") else None,
                "square_height_mm":pdf_dims.get("diameter",{}).get("second_dimension_mm"),
                "availability_scope":"listed in public store; stock not verified",
                "source_url":product["url"], "catalog_pdf_url":PDF_URL if page else "",
                "catalog_pdf_page":page, "retrieved_date":source["retrieved_date"]}
        if any(item["value"] is None for item in dims.values()):
            raise ValueError(f"Published dimensions/mass extraction incomplete for {product['model']}: {dims}")
        model_conflicts = list(product_conflicts)
        for ratio in ratios:
            row_conflicts = list(product_conflicts)
            values = ratings.get(ratio,{})
            for field in FIELDS:
                web, pdf = values.get(field), pdf_rates.get(ratio,{}).get(field)
                if web is not None and pdf is not None and not math.isclose(web,pdf,rel_tol=1e-9,abs_tol=1e-12):
                    row_conflicts.append(field)
                    model_conflicts.append(field)
                    conflicts.append({"model":product["model"],"gear_ratio":ratio,"field":field,
                        "web_value":web,"web_qualifier":"published_value", "pdf_value":pdf,
                        "pdf_qualifier":"published_value", "web_source_url":product["url"],
                        "pdf_source_url":PDF_URL,"pdf_page":page,
                        "resolution":"Conflicting official sources; no automatic solver import."})
            notes = []
            skus=[item["sku_id"] for item in product.get("store_skus",[])
                  if str(ratio) == item["attributes"].get("Speed Ratio")]
            if not skus:
                notes.append("The page selector lists this ratio, but its embedded store data has no matching SKU. Availability requires confirmation.")
            if values.get(FIELDS[0]) is None:
                notes.append("This ratio is selectable, but its rated torque is not published on the product page.")
            if base["cross_section_shape"] == "rectangular":
                notes.append("PDF shows a rectangular section; website diameter label must not be used as a circumscribing cylinder.")
            if family == "CS-Mini":
                notes.append("Product-page envelope values are shared across variants; linked variant drawings require verification.")
            if row_conflicts:
                notes.append("Official page/PDF differences: "+", ".join(sorted(set(row_conflicts)))+"; see source_conflicts.csv.")
            variants.append({"reducer_id":f"SITO_{product['model'].replace('.','p')}_{ratio}",
                             **base,"gear_ratio":ratio, **{field:values.get(field) for field in FIELDS},
                             "store_sku_ids":"/".join(map(str,skus)),
                             "availability_scope":"SKU-listed in public store; stock not verified" if skus else "selector-listed only; no matching SKU; availability unconfirmed",
                             "max_input_torque_Nm":None,
                             "inertia_reference":"Manufacturer I table, converted from I x 10^-4 kg m^2; shaft/reference side not explicit on product page",
                             "spec_status":"source_difference_requires_review" if row_conflicts else
                                           "rated_torque_missing" if values.get(FIELDS[0]) is None else "published_fields_recorded",
                             "notes":" ".join(notes)})
        products.append({**base,"available_ratios":"/".join(map(str,ratios)),"variant_count":len(ratios),
                         "spec_source_differences":"; ".join(sorted(set(model_conflicts))),
                         "drawing_2d_url":next((item["url"] for item in product["downloads"] if "2d" in item["label"].lower()),""),
                         "model_3d_url":next((item["url"] for item in product["downloads"] if "3d model" in item["label"].lower()),"")})
    # One additional reducer is cataloged but not offered in the captured store inventory.
    extra_text = pages[62]["text"]
    extra_dims = dimension_facts(extra_text)
    extra_rates = pdf_ratings(extra_text,[30,50])
    catalog_only = [{"reducer_id":f"SITO_CS_Mini_2p5_{ratio}","manufacturer":"SITO Drive",
        "series":"CS-Mini", "model":"CS Mini-2.5", "gear_ratio":ratio,
        "diameter_mm":11,"cross_section_shape":"circular per dimensioned drawing", "width_mm":None,"height_mm":None,
        "length_mm":extra_dims["length"]["value"], "mass_kg":extra_dims["mass"]["value"],
        **{field:extra_rates[ratio].get(field) for field in FIELDS}, "max_input_torque_Nm":None,
        "availability_scope":"PDF catalog only; no matching current store listing found",
        "source_url":PDF_URL,"catalog_pdf_page":63,"retrieved_date":source["retrieved_date"],
        "notes":"Catalog headline says 11 x 11 mm, but the dimensioned drawing and photo show a circular diameter-11 mm case. Verify procurement and applicable drawing revision."}
        for ratio in (30,50)]
    products.sort(key=lambda row:(row["series"],row["model"]))
    variants.sort(key=lambda row:(row["series"],row["model"],row["gear_ratio"]))
    write_csv("reducer_models.csv",products)
    write_csv("reducer_variants.csv",variants)
    write_csv("source_conflicts.csv",conflicts)
    write_csv("catalog_only.csv",catalog_only)
    write_csv("excluded_store_entries.csv",excluded)
    (ROOT/"catalog_pdf_facts.json").write_text(json.dumps(pdf_facts,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
    counts = {"store_model_count":len(products),"store_ratio_variant_count":len(variants),
              "catalog_only_model_count":1,"catalog_only_ratio_variant_count":2,
              "store_models_by_series":dict(Counter(row["series"] for row in products)),
              "variants_with_missing_rated_torque":sum(row[FIELDS[0]] is None for row in variants),
              "store_sku_variant_count":sum(bool(row["store_sku_ids"]) for row in variants),
              "selector_only_variant_count":sum(not row["store_sku_ids"] for row in variants),
              "variants_with_source_differences":sum(row["spec_status"]=="source_difference_requires_review" for row in variants),
              "source_difference_field_count":len(conflicts),
              "all_max_input_torque_unpublished":all(row["max_input_torque_Nm"] is None for row in variants)}
    (ROOT/"coverage.json").write_text(json.dumps(counts,indent=2)+"\n",encoding="utf-8")
    print(json.dumps(counts,indent=2))


if __name__ == "__main__":
    main()
