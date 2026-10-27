from pathlib import Path
import sys
import tempfile
import shutil
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from large_assets import pack_usd, restore, RELATIVE_USD, safe_path


class LargeAssets(unittest.TestCase):
    def test_chunks_restore_exact_bytes_without_overwriting_local_changes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / RELATIVE_USD
            source.parent.mkdir(parents=True)
            data = b"#usda 1.0\n" + bytes(range(256))
            source.write_bytes(data)
            metadata = pack_usd(root, chunk_size=32)
            self.assertGreater(len(metadata["assets"][0]["parts"]), 1)
            source.unlink()
            self.assertEqual(restore(root), [RELATIVE_USD])
            self.assertEqual(source.read_bytes(), data)
            source.write_bytes(b"local edit")
            with self.assertRaises(RuntimeError):
                restore(root)
            self.assertEqual(source.read_bytes(), b"local edit")

    def test_pulled_chunks_refresh_an_unmodified_local_cache(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "local"
            source = root / RELATIVE_USD
            source.parent.mkdir(parents=True)
            source.write_bytes(b"old USD")
            pack_usd(root, chunk_size=8)
            remote = Path(directory) / "remote"
            shutil.copytree(root, remote)
            (remote / RELATIVE_USD).write_bytes(b"updated USD from other computer")
            pack_usd(remote, chunk_size=8)
            shutil.copy2(remote / "config/large_assets.json", root / "config/large_assets.json")
            shutil.copytree(remote / "assets", root / "assets", dirs_exist_ok=True)
            restore(root)
            self.assertEqual(source.read_bytes(), b"updated USD from other computer")

    def test_corrupt_chunk_does_not_publish_partial_asset(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / RELATIVE_USD
            source.parent.mkdir(parents=True)
            source.write_bytes(b"original USD bytes")
            metadata = pack_usd(root, chunk_size=8)
            source.unlink()
            (root / metadata["assets"][0]["parts"][0]["path"]).write_bytes(b"corrupt")
            with self.assertRaises(RuntimeError):
                restore(root)
            self.assertFalse(source.exists())
            self.assertEqual(list(source.parent.glob("*.tmp")), [])

    def test_asset_paths_cannot_escape_project(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError):
                safe_path(Path(directory), "../outside")


if __name__ == "__main__":
    unittest.main()
