"""Save the official source snapshots used by this motor survey."""
from concurrent.futures import ThreadPoolExecutor
from hashlib import sha256
from html.parser import HTMLParser
import json
from pathlib import Path
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parent
CUBE = 'https://www.cubemars.com/product/'
NANO = 'https://www.nanotec.com/fileadmin/files/'
SOURCES = {
    'cubemars_ri30': CUBE+'ri30-kv260-frameless-torque-motor.html',
    'cubemars_ri50': CUBE+'ri50-kv100-frameless-inrunner-torque-motor.html',
    'cubemars_ri60': CUBE+'ri60-kv120-frameless-inrunner-torque-motor.html',
    'cubemars_ri75_welded': CUBE+'ri75-ph-kv70-potting-frameless-torque-motor-welded-model.html',
    'cubemars_ri75_connector': CUBE+'ri75-ph-kv70-potting-frameless-torque-motor.html',
    'cubemars_ro40_lite': CUBE+'ro40-kv140-lite-without-hall-frameless-torque-motor.html',
    'cubemars_ro50_lite': CUBE+'ro50-kv108-lite-without-hall-frameless-torque-motor.html',
    'nanotec_db42_overview': NANO+'Baureihenuebersichten/BLDC_Motoren/Product_Overview_DB42.pdf',
    'nanotec_db43_overview': NANO+'Baureihenuebersichten/BLDC_Motoren/Product_Overview_DB43.pdf',
    'nanotec_dbl42_overview': NANO+'Baureihenuebersichten/BLDC_Motoren/Product_Overview_DBL42.pdf',
    **{f'nanotec_{model.lower()}': NANO+f'Datenblaetter/BLDC/DB42/{model}.pdf'
       for model in ('DB42S01','DB42S02','DB42S03','DB42M01','DB42M03','DB42L01')},
}

class PlainText(HTMLParser):
    def __init__(self):
        super().__init__(); self.skip=0; self.parts=[]
    def handle_starttag(self,tag,attrs):
        if tag in ('script','style'): self.skip+=1
        if tag in ('tr','p','h1','h2','h3','h4','div'): self.parts.append('\n')
        if tag in ('td','th'): self.parts.append(' | ')
    def handle_endtag(self,tag):
        if tag in ('script','style'): self.skip=max(0,self.skip-1)
    def handle_data(self,data):
        if not self.skip: self.parts.append(data)

def fetch(item):
    name,url=item
    try:
        with urlopen(Request(url,headers={'User-Agent':'Mozilla/5.0'}),timeout=40) as response:
            blob=response.read()
        suffix='.pdf' if blob.startswith(b'%PDF') else '.html'
        path=ROOT/'sources'/(name+suffix);path.write_bytes(blob)
        if suffix=='.html':
            parser=PlainText();parser.feed(blob.decode('utf-8',errors='replace'))
            text='\n'.join(line.strip() for line in ''.join(parser.parts).splitlines() if line.strip())
        else:
            from pypdf import PdfReader
            text='\n'.join(page.extract_text() or '' for page in PdfReader(path).pages)
        path.with_suffix('.txt').write_text(text,encoding='utf-8')
        return dict(source_id=name,url=url,file=str(path.relative_to(ROOT)),sha256=sha256(blob).hexdigest(),retrieved_date='2026-11-01',error='')
    except Exception as exc:
        return dict(source_id=name,url=url,error=str(exc),retrieved_date='2026-11-01')

if __name__=='__main__':
    (ROOT/'sources').mkdir(parents=True,exist_ok=True)
    with ThreadPoolExecutor(max_workers=8) as pool: manifest=list(pool.map(fetch,SOURCES.items()))
    (ROOT/'sources'/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    print(json.dumps([{'source':row['source_id'],'error':row['error']} for row in manifest],indent=2))
