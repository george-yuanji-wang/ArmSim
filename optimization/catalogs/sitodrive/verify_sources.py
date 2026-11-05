"""Audit every observed primary product URL and the official PDF download."""

from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
import hashlib
import json
import time
from urllib.request import Request, urlopen

from lxml import html

ROOT = Path(__file__).resolve().parent


def verify(product):
    for attempt in range(3):
        try:
            with urlopen(Request(product["url"],headers={"User-Agent":"Mozilla/5.0"}),timeout=45) as response:
                raw = response.read()
                status, final, content_type = response.status, response.geturl(), response.headers.get("Content-Type","")
            break
        except Exception:
            if attempt == 2:
                raise
            time.sleep(.5)
    doc = html.fromstring(raw)
    headings = [" ".join(" ".join(element.itertext()).split()) for element in doc.xpath('//h1|//h2|//h3')]
    if status != 200 or product["model"] not in headings:
        raise ValueError(f"Source did not identify intended product: {product['url']}: {status}, {headings}")
    return {"model":product["model"],"observed_inventory_url":product["url"],
            "final_response_url":final,"http_status":status,"content_type":content_type,
            "confirmed_product_heading":product["model"], "table_count":len(doc.xpath('//table')),
            "response_sha256":hashlib.sha256(raw).hexdigest()}


def main():
    source = json.loads((ROOT/"catalog_sources.json").read_text(encoding="utf-8"))
    products = [p for p in source["products"] if any(a["name"].lower()=="speed ratio" for a in p["product_attributes"])]
    results = []
    with ThreadPoolExecutor(max_workers=6) as pool:
        jobs = [pool.submit(verify,p) for p in products]
        for job in as_completed(jobs):
            results.append(job.result())
    url = "https://www.sitodrive.com/docs/Strain_Wave_Gear_Catalog_en.pdf"
    with urlopen(Request(url,headers={"User-Agent":"Mozilla/5.0"}),timeout=60) as response:
        raw=response.read()
        pdf={"url":url,"final_response_url":response.geturl(),"http_status":response.status,
             "content_type":response.headers.get("Content-Type", ""),"bytes":len(raw),
             "sha256":hashlib.sha256(raw).hexdigest(),"valid_pdf_signature":raw.startswith(b"%PDF")}
    if not pdf["valid_pdf_signature"] or pdf["http_status"] != 200:
        raise ValueError("Official catalog download was not a PDF response")
    results.sort(key=lambda row:row["model"])
    payload = {"retrieved_date":source["retrieved_date"], "product_source_count":len(results),
               "all_product_sources_http_200_and_correct_model":True,
               "products":results,"catalog_pdf":pdf}
    (ROOT/"source_verification.json").write_text(json.dumps(payload,indent=2)+"\n",encoding="utf-8")
    print(f"Verified {len(results)} product URLs: HTTP 200 and matching product heading; official PDF HTTP200 and %PDF signature.")


if __name__ == "__main__":
    main()
