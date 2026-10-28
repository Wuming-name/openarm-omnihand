"""Render recent read-only seven-axis probe versus pose A. Never opens CAN."""
import argparse
import json
from pathlib import Path
import time

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from .control import RunBundle
from .settings import NAMES
from rl_ppo.model import contacts


def read_probe(path):
    report = json.loads(path.read_text(encoding='utf-8'))
    if not report.get('official_protocol_match') or report.get('motion_commands_sent') is not False:
        raise ValueError('Expected successful read-only seven-axis probe')
    motors = report['motors']
    if len(motors) != 7 or [m['mapping']['joint'] for m in motors] != NAMES:
        raise ValueError('Probe joint ordering differs from trained deployment')
    for m in motors:
        if m['mapping']['sign'] != 1 or m['mapping']['offset_rad'] != 0.:
            raise ValueError('Probe contains non-default mapping; review mapping before comparison')
    q = np.asarray([m['state']['q'] for m in motors], dtype=float)
    if not np.isfinite(q).all():
        raise ValueError('Nonfinite measured joints')
    return q


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', required=True)
    parser.add_argument('--probe', help='Explicit saved successful probe; omit to use newest probe from last 60 seconds')
    parser.add_argument('--output', default='outputs/real_robot/start_pose_comparison.png')
    args = parser.parse_args()
    if args.probe:
        probe = Path(args.probe)
    else:
        candidates = list(Path('outputs/real_robot').glob('*_probe.json'))
        if not candidates:
            raise ValueError('Run python -m real_robot.probe first')
        probe = max(candidates, key=lambda path: path.stat().st_mtime)
        if time.time()-probe.stat().st_mtime > 60:
            raise ValueError('Probe older than 60 seconds; refresh read-only probe')
    q = read_probe(probe)
    bundle = RunBundle(args.run, load_policy=False)
    model = bundle.env.model
    result = {'mode': 'offline_model_diagnostic', 'source_probe': str(probe),
              'hardware_commands_sent': False, 'measured_q_rad': q.tolist(),
              'trained_q_A_rad': bundle.q_a.tolist(), 'model_sha256': bundle.env.task['model_sha256'],
              'note': 'Default joint mapping and frozen training hand used. Compare against physical arm.'}
    model.vis.global_.offwidth = 600
    model.vis.global_.offheight = 600
    canvas = Image.new('RGB', (1200, 1280), '#edf1f5')
    draw = ImageDraw.Draw(canvas)
    draw.text((12, 12), 'MEASURED JOINTS vs TRAINED A - model prediction, not real robot photograph', fill='black')
    draw.text((12, 30), 'Red markers: collision contacts beyond configured tolerance', fill='black')
    renderer = mujoco.Renderer(model, 600, 600)
    try:
        for row, (title, pose) in enumerate((('MEASURED', q), ('TRAINED A', bundle.q_a))):
            bundle.update_state(pose, np.zeros(7))
            collision = contacts(model, bundle.env.data, bundle.env.physics['contact_tolerance_m'])
            result[title] = {'contacts': collision, 'palm_base_m': bundle.env.position.tolist()}
            for col, azimuth in enumerate((0., 90.)):
                camera = mujoco.MjvCamera()
                camera.lookat[:] = [0., -.2, .3]
                camera.distance, camera.azimuth, camera.elevation = 1.35, azimuth, -15.
                renderer.update_scene(bundle.env.data, camera=camera)
                for c in bundle.env.data.contact:
                    if c.dist >= -bundle.env.physics['contact_tolerance_m']:
                        continue
                    scene = renderer.scene
                    if scene.ngeom >= scene.maxgeom:
                        break
                    mujoco.mjv_initGeom(scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
                                       np.array([.012]*3), c.pos, np.eye(3).ravel(),
                                       np.array([1., .1, .05, 1.]))
                    scene.ngeom += 1
                x, y = col*600, 60+row*610
                canvas.paste(Image.fromarray(renderer.render().copy()), (x, y))
                draw.text((x+12, y+12), f'{title}: azimuth {azimuth:g}, contacts {len(collision)}', fill='white')
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        canvas.save(output)
        output.with_suffix('.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
        print(json.dumps(result, ensure_ascii=False, indent=2))
        print('IMAGE:', output.resolve())
    finally:
        renderer.close()
        bundle.env.close()


if __name__ == '__main__':
    main()
