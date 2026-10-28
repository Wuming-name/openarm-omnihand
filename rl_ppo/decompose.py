"""Offline optional rebuild of wrist convex pieces. Never disables a contact pair."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

from .common import DEFAULT_CONFIG, common_parser, load_config, root_path, sha256, write_json

GEOMS = ["collision_openarm_right_link5_0", "collision_openarm_right_link7_0"]
ASSETS = Path(__file__).parent / "assets/wrist"


def main():
    parser = common_parser(__doc__)
    parser.add_argument("--output", default=str(ASSETS))
    parser.add_argument("--method", choices=["components", "coacd"], default="components")
    args = parser.parse_args()
    root = root_path(args.project_root)
    import trimesh
    config = load_config(DEFAULT_CONFIG)
    source = root / config["source_model"]
    xml = ET.parse(source).getroot()
    directory = source.parent / xml.find("compiler").get("meshdir", "")
    output = Path(args.output)
    if output.exists() and any(output.iterdir()):
        raise FileExistsError("Output exists; use a new directory")
    output.mkdir(parents=True, exist_ok=True)
    meshes = {m.get("name"): directory / m.get("file") for m in xml.findall("asset/mesh")}
    parameters = dict(threshold=0.025, preprocess_mode="auto", preprocess_resolution=50,
                      resolution=1000, mcts_nodes=8, mcts_iterations=10, mcts_max_depth=2,
                      max_convex_hull=32, merge=True, decimate=True, max_ch_vertex=64, seed=28)
    manifest = {"tool": "coacd" if args.method == "coacd" else "trimesh", "version": "1.0.7" if args.method == "coacd" else "4.7.4", "method": args.method,
                "parameters": parameters if args.method == "coacd" else {"split_connected_components": True}, "geoms": {},
                "status": "approximate convex decomposition; inspect visually before physical deployment"}
    if args.method == "coacd":
        import coacd
        coacd.set_log_level("warn")
    for name in GEOMS:
        geom = next(g for g in xml.findall(".//geom") if g.get("name") == name)
        path = meshes[geom.get("mesh")]
        mesh = trimesh.load_mesh(path, process=True)
        print("Decomposing", name, "vertices", len(mesh.vertices), flush=True)
        components = list(mesh.split(only_watertight=False))
        # Source CAD also contains isolated planar cap patches. A planar patch
        # has no 3-D hull, so retain its triangles with the nearest solid part.
        solids, planar = [], []
        for component in components:
            singular = np.linalg.svd(component.vertices-component.vertices.mean(axis=0), compute_uv=False)
            (solids if singular[-1] > 1e-8 else planar).append(component)
        if not solids:
            raise ValueError("No volumetric component found")
        for patch in planar:
            distances = [float(np.min(np.sum((solid.vertices-patch.vertices.mean(axis=0))**2, axis=1))) for solid in solids]
            nearest = int(np.argmin(distances))
            solids[nearest] = trimesh.util.concatenate([solids[nearest], patch])
        components = solids
        parts = []
        for component in components:
            if args.method == "coacd":
                parts.extend(coacd.run_coacd(coacd.Mesh(np.asarray(component.vertices, dtype=np.float64), np.asarray(component.faces, dtype=np.int32)), **parameters))
            else:
                parts.append((component.vertices, component.faces))
        files = []
        for i, (vertices, faces) in enumerate(parts):
            filename = f"{name}_part{i:02}.obj"
            trimesh.Trimesh(vertices, faces, process=False).export(output / filename)
            files.append({"file": filename, "sha256": sha256(output / filename)})
        manifest["geoms"][name] = {"source_mesh_sha256": sha256(path), "planar_patches_retained_with_nearest_solid": len(planar), "parts": files}
        print(name, "pieces", len(files), flush=True)
    write_json(output / "manifest.json", manifest)


if __name__ == "__main__":
    main()
