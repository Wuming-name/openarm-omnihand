"""Reproducible content set; no build caches, links, or recursive archives. Archive hashes live outside archives."""
from pathlib import Path
import tarfile,zipfile,hashlib,json
P=Path(__file__).resolve().parents[1];prefix='openarm_omnihand_portable_model';excluded={'runtime','dist','__pycache__','.git'}
def files():return sorted(f for f in P.rglob('*') if f.is_file() and not any(x in excluded for x in f.relative_to(P).parts) and not f.name.endswith('.pyc') and f.name not in ['archive_receipt.json','OPENARM_OMNIHAND_PORTABLE_MODEL_REPORT.md'])
fs=files();assert not any(f.is_symlink() for f in fs)
manifest=P/'manifest/asset_hashes.json';payload={str(f.relative_to(P)):hashlib.sha256(f.read_bytes()).hexdigest() for f in fs if f!=manifest};manifest.write_text(json.dumps(payload,indent=2));fs=files();(P/'dist').mkdir(exist_ok=True)
with tarfile.open(P/'dist'/(prefix+'.tar.gz'),'w:gz',compresslevel=6) as a:
 for f in fs:a.add(f,arcname=prefix+'/'+str(f.relative_to(P)),recursive=False)
with zipfile.ZipFile(P/'dist'/(prefix+'.zip'),'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as a:
 for f in fs:a.write(f,arcname=prefix+'/'+str(f.relative_to(P)))
receipt={f.name:{'bytes':f.stat().st_size,'sha256':hashlib.sha256(f.read_bytes()).hexdigest()} for f in (P/'dist').iterdir() if f.suffix in ['.zip','.gz']};(P/'dist/archive_receipt.json').write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt,indent=2))
