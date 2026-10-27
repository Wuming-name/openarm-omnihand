"""Store the large USD in normal Git as verified 20 MiB chunks; no LFS service required."""
from pathlib import Path
import hashlib
import json
import os
import tempfile

RELATIVE_USD = "model/simulation/usd/openarm_omnihand.usda"
METADATA = "config/large_assets.json"
LOCAL_STATE = "outputs/large_assets_state.json"


def save_state(root, state):
    path = root / LOCAL_STATE
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(state, indent=2) + "\n", encoding="utf-8")


def safe_path(root, relative):
    result = (root / relative).resolve()
    if not result.is_relative_to(root.resolve()):
        raise ValueError("Asset path escapes project directory")
    return result


def pack_usd(root, chunk_size=20 * 1024 * 1024):
    root = Path(root)
    if chunk_size <= 0:
        raise ValueError("Chunk size must be positive")
    source = root / RELATIVE_USD
    if not source.is_file():
        raise RuntimeError("USD not present; first run prepare-assets")
    directory = root / "assets/usd_parts"
    directory.mkdir(parents=True, exist_ok=True)
    parts = []
    digest = hashlib.sha256()
    size = 0
    with source.open("rb") as stream:
        index = 1
        while True:
            data = stream.read(chunk_size)
            if not data:
                break
            name = f"assets/usd_parts/openarm_omnihand.usda.part{index:03d}.bin"
            (root / name).write_bytes(data)
            parts.append({"path": name, "bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()})
            digest.update(data)
            size += len(data)
            index += 1
    info = {"schema_version": 1, "assets": [{"target": RELATIVE_USD, "bytes": size,
            "sha256": digest.hexdigest(), "parts": parts}]}
    (root / METADATA).parent.mkdir(parents=True, exist_ok=True)
    (root / METADATA).write_bytes((json.dumps(info, indent=2) + "\n").encode("utf-8"))
    save_state(root, {RELATIVE_USD: digest.hexdigest()})
    return info


def restore(root):
    root = Path(root)
    metadata = json.loads((root / METADATA).read_text(encoding="utf-8"))
    state_path = root / LOCAL_STATE
    state = json.loads(state_path.read_text(encoding="utf-8")) if state_path.is_file() else {}
    restored = []
    for asset in metadata["assets"]:
        target = safe_path(root, asset["target"])
        if target.exists():
            current = hashlib.sha256(target.read_bytes()).hexdigest()
            if current == asset["sha256"]:
                state[asset["target"]] = current
                continue
            # Regenerate a known unchanged cache after git pull, but preserve local edits.
            if state.get(asset["target"]) != current:
                raise RuntimeError("Local USD differs from tracked chunks. Preserve edits and use pack-usd before syncing.")
        target.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(dir=target.parent, suffix=".tmp", delete=False) as stream:
                temporary = Path(stream.name)
                digest = hashlib.sha256()
                size = 0
                for part in asset["parts"]:
                    data = safe_path(root, part["path"]).read_bytes()
                    if len(data) != part["bytes"] or hashlib.sha256(data).hexdigest() != part["sha256"]:
                        raise RuntimeError("Asset chunk is missing or corrupt: " + part["path"])
                    stream.write(data)
                    digest.update(data)
                    size += len(data)
            if size != asset["bytes"] or digest.hexdigest() != asset["sha256"]:
                raise RuntimeError("Reassembled USD checksum mismatch")
            os.replace(temporary, target)
            state[asset["target"]] = asset["sha256"]
            restored.append(asset["target"])
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
    save_state(root, state)
    return restored
