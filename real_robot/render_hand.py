"""Render the actual frozen MuJoCo hand geometry saved with a PPO task."""
import argparse
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    task_path = Path(args.task)
    task = json.loads(task_path.read_text(encoding='utf-8'))
    config = json.loads(task_path.with_name('config.json').read_text(encoding='utf-8'))
    model_path = task_path.parent / task['model_file']
    digest = hashlib.sha256(model_path.read_bytes()).hexdigest()
    if digest != task['model_sha256']:
        raise ValueError('Saved model SHA256 does not match task')
    model = mujoco.MjModel.from_binary_path(str(model_path))
    data = mujoco.MjData(model)
    data.qpos[:] = task['q_A']
    mujoco.mj_forward(model, data)
    hand_ids = [i for i in range(model.nbody)
                if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or '').startswith('omni_')]
    center = data.xpos[hand_ids].mean(axis=0)
    # Hide arm links 0..6 only for close-ups; leave wrist/palm and all fingers.
    for i in range(model.ngeom):
        body = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[i])) or ''
        if body.startswith('openarm_right_link') and body != 'openarm_right_link7':
            model.geom_rgba[i, 3] = 0
    model.vis.global_.offwidth = 640
    model.vis.global_.offheight = 640
    camera = mujoco.MjvCamera()
    camera.lookat[:] = center
    camera.distance = .32
    camera.elevation = -15
    canvas = Image.new('RGB', (1280, 1360), '#eff2f6')
    draw = ImageDraw.Draw(canvas)
    draw.text((20, 12), 'TRAINING FIXED HAND - saved MuJoCo geometry at pose A', fill='black')
    draw.text((20, 30), 'Model SHA256: '+digest, fill='black')
    renderer = mujoco.Renderer(model, 640, 640)
    try:
        for index, azimuth in enumerate((0, 90, 180, 270)):
            camera.azimuth = azimuth
            renderer.update_scene(data, camera=camera)
            x, y = (index % 2)*640, 60+(index//2)*650
            canvas.paste(Image.fromarray(renderer.render().copy()), (x, y))
            draw.text((x+12, y+10), f'View {index+1}: azimuth {azimuth} deg', fill='white')
    finally:
        renderer.close()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output)
    output.with_suffix('.json').write_text(json.dumps({
        'source_task': str(task_path), 'model_sha256': digest,
        'hand_pose_rad': config['hand_pose_rad'],
        'note': 'Frozen geometry from saved model; not an image of the real hand.'
    }, indent=2)+'\n', encoding='utf-8')
    print(output.resolve())


if __name__ == '__main__':
    main()
