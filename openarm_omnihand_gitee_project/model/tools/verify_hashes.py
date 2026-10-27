from pathlib import Path
import json,hashlib
P=Path(__file__).resolve().parents[1];m=json.loads((P/'manifest/asset_hashes.json').read_text());errors=[n for n,h in m.items() if not (P/n).is_file() or hashlib.sha256((P/n).read_bytes()).hexdigest()!=h];assert not errors,errors;print('HASHES_PASS',len(m))
