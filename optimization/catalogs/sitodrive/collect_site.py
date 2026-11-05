"""Collect public SITO store facts without guessing unpublished ratings.

Run with the bundled analysis Python (lxml is required). Cached HTML is local
research material; catalog_sources.json retains the extracted facts and URLs.
"""

from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import json
import re
import sys
import time
from pathlib import Path
from urllib.parse import urljoin
from urllib.request import Request, urlopen

from lxml import html


ROOT = Path(__file__).resolve().parent
CACHE = ROOT / ".cache"
BASE = "https://www.sitodrive.com"
RETRIEVED = "2026-11-01"
CATEGORIES = ("hd", "hs", "cs", "csmini", "cd", "all")


def fetch(url, key):
    CACHE.mkdir(exist_ok=True)
    path = CACHE / (key + ".html")
    if path.exists():
        return path.read_bytes()
    for attempt in range(3):
        try:
            with urlopen(Request(url, headers={"User-Agent": "Mozilla/5.0"}), timeout=45) as response:
                raw = response.read()
            path.write_bytes(raw)
            return raw
        except Exception:
            if attempt == 2:
                raise
            time.sleep(.5)


def clean(element):
    return " ".join(" ".join(element.itertext()).split())


def inventory():
    products, categories = {}, []
    for family in CATEGORIES:
        route = "/en/store/reducer" + ("/" + family if family != "all" else "")
        url = BASE + route + "?pagesize=100"
        raw = fetch(url, "category_" + family)
        doc = html.fromstring(raw)
        listings = doc.xpath('//div[contains(concat(" ",normalize-space(@class)," ")," product_list ")]')
        expected = int(listings[0].get("data-total")) if listings else None
        found = []
        for item in doc.xpath('//a[contains(concat(" ",normalize-space(@class)," ")," product_item ")]'):
            image = item.xpath('.//*[@data-productid]')[0]
            pid, name = image.get("data-productid"), image.get("data-name")
            product = products.setdefault(pid, {"product_id": pid, "model": name,
                "url": urljoin(BASE, item.get("href")), "categories": []})
            product["categories"].append(family)
            found.append(pid)
        categories.append({"category": family, "url": url, "displayed_count": expected,
                           "collected_count": len(found), "product_ids": found,
                           "source_sha256": hashlib.sha256(raw).hexdigest()})
        print(f"Inventory {family}: {len(found)}/{expected}", flush=True)
        if expected != len(found):
            raise ValueError(f"Pagination incomplete: {family} {len(found)} != {expected}")
    return products, categories


def collect(product):
    product = dict(product)
    raw = fetch(product["url"], "product_" + product["product_id"])
    doc = html.fromstring(raw)
    # Scripts/styles contain repeated builder CSS rather than product facts.
    for element in doc.xpath('//script|//style'):
        element.getparent().remove(element)
    product["source_sha256"] = hashlib.sha256(raw).hexdigest()
    product["retrieved_date"] = RETRIEVED
    text = clean(doc)
    product["tables"] = [
        [[{"text": clean(cell), "colspan": int(cell.get("colspan", "1")),
           "rowspan": int(cell.get("rowspan", "1"))}
          for cell in row.xpath('./td|./th')] for row in table.xpath('.//tr')]
        for table in doc.xpath('//table')]
    product["product_attributes"] = []
    sku_nodes = doc.xpath('//*[@data-skus]')
    product["store_skus"] = [{"sku_id":item.get("productSkuid"),
                               "attributes":{attribute["attr_name"]:attribute["attr_val"]
                                             for attribute in item.get("productAttr",[])}}
                              for item in json.loads(sku_nodes[0].get("data-skus"))] if sku_nodes else []
    for attribute in doc.xpath('//*[contains(concat(" ",normalize-space(@class)," ")," product_control_item ")]'):
        name = attribute.xpath('.//*[contains(@class,"product_control_item_name")]')
        values = attribute.xpath('.//*[@data-productattrvalid]')
        product["product_attributes"].append({
            "name": clean(name[0]) if name else "",
            "values": [{"value": item.get("data-val") or clean(item),
                        "attribute_value_id": item.get("data-productattrvalid"),
                        "attribute_id": item.get("data-productattrid")}
                       for item in values]})
    product["downloads"] = [{"label": clean(link), "url": urljoin(BASE, link.get("href"))}
                             for link in doc.xpath('//a[@href]')
                             if any(token in clean(link).lower() for token in
                                    ("2d drawing", "2d model", "3d model", "3d view"))]
    product["dimension_text"] = ""
    for match in re.finditer(r"Diameter\s*:", text, flags=re.I):
        product["dimension_text"] = text[match.start():match.start()+240]
        break
    return product


def main():
    products, categories = inventory()
    results = []
    with ThreadPoolExecutor(max_workers=6) as pool:
        jobs = {pool.submit(collect, product): pid for pid, product in products.items()}
        for job in as_completed(jobs):
            result = job.result()
            results.append(result)
            print(f"Product {result['model']}: {len(result['tables'])} tables", flush=True)
    results.sort(key=lambda item: (item["categories"][0], item["model"]))
    payload = {"manufacturer": "SITO Drive", "retrieved_date": RETRIEVED,
               "category_inventory": categories, "products": results}
    (ROOT / "catalog_sources.json").write_text(json.dumps(payload, indent=2, ensure_ascii=False)+"\n",
                                               encoding="utf-8")
    print(f"Saved {len(results)} public product pages", flush=True)


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    main()
