"""Fetch pinned, licensed model definitions from MuJoCo Menagerie (no meshes)."""

import argparse
import hashlib
import json
from pathlib import Path
import urllib.request

ROOT=Path(__file__).resolve().parents[1]
BASE='https://api.github.com/repos/google-deepmind/mujoco_menagerie'


def fetch(url):
    request=urllib.request.Request(url,headers={'User-Agent':'ArmSim-reference-audit'})
    with urllib.request.urlopen(request,timeout=30) as response:
        return response.read()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--latest',action='store_true',help='Resolve current main rather than reuse the saved revision.')
    args=parser.parse_args()
    output=ROOT/'references/menagerie'
    output.mkdir(parents=True,exist_ok=True)
    saved=output/'sources.json'
    revision=(json.loads(fetch(BASE+'/commits/main'))['sha'] if args.latest or not saved.exists()
              else json.loads(saved.read_text())['revision'])
    manifest={'repository':'https://github.com/google-deepmind/mujoco_menagerie','revision':revision,'files':[]}
    for folder in ('universal_robots_ur5e','ufactory_lite6','fanuc_crx5ia'):
        entries=json.loads(fetch(BASE+'/contents/'+folder+'?ref='+revision))
        for item in entries:
            if not (item['name'].endswith('.xml') or item['name'].lower().startswith(('readme','license'))):
                continue
            content=fetch(item['download_url'])
            target=output/folder/item['name']
            target.parent.mkdir(parents=True,exist_ok=True)
            target.write_bytes(content)
            manifest['files'].append({'path':str(target.relative_to(ROOT)).replace('\\','/'),
                                      'source':item['html_url'],'sha256':hashlib.sha256(content).hexdigest()})
    saved.write_text(json.dumps(manifest,indent=2)+'\n')
    print(f"Saved {len(manifest['files'])} source/license files at {revision}")


if __name__=='__main__':main()
