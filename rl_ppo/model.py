from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from .common import ARM_JOINTS, config_hash, sha256, write_json, read_json


def object_id(model, kind, name):
    result = mujoco.mj_name2id(model, kind, name)
    if result < 0:
        raise ValueError(f"Required model object not found: {name}")
    return result


def rotation_vector(matrix):
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, np.asarray(matrix).reshape(9))
    if quat[0] < 0:
        quat *= -1
    norm = np.linalg.norm(quat[1:])
    return 2 * quat[1:] if norm < 1e-9 else quat[1:] / norm * (2 * np.arctan2(norm, quat[0]))


def ids(model, config):
    joints = np.array([object_id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in ARM_JOINTS])
    if model.nq != 7 or model.nv != 7 or model.nu != 7:
        raise ValueError("Training model must have exactly seven arm DOFs and seven actuators")
    if (not np.array_equal(model.jnt_qposadr[joints], np.arange(7))
            or not np.array_equal(model.jnt_dofadr[joints], np.arange(7))
            or not np.all(model.jnt_type[joints] == mujoco.mjtJoint.mjJNT_HINGE)
            or not np.array_equal(model.actuator_trnid[:, 0], joints)):
        raise ValueError("Joint/actuator order or type differs from the seven-axis policy contract")
    return {"q": model.jnt_qposadr[joints], "v": model.jnt_dofadr[joints],
            "joints": joints, "tcp": object_id(model, mujoco.mjtObj.mjOBJ_SITE, "palm_tcp"),
            "base": object_id(model, mujoco.mjtObj.mjOBJ_SITE, config["base_site"])}


def tcp_pose(data, indices):
    base = indices["base"]
    rb = data.site_xmat[base].reshape(3, 3)
    p = rb.T @ (data.site_xpos[indices["tcp"]] - data.site_xpos[base])
    r = rb.T @ data.site_xmat[indices["tcp"]].reshape(3, 3)
    return p.copy(), r.copy()


def tcp_velocity(model, data, indices):
    jp = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, jp, None, indices["tcp"])
    return data.site_xmat[indices["base"]].reshape(3, 3).T @ (jp @ data.qvel)


def contacts(model, data, tolerance):
    result = []
    for c in data.contact[:data.ncon]:
        if c.dist < -tolerance:
            result.append({"a": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, c.geom1),
                           "b": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, c.geom2),
                           "distance_m": float(c.dist)})
    return result


def within_workspace(p, config):
    return bool(np.all(p >= config["task"]["tcp_workspace_low"]) and np.all(p <= config["task"]["tcp_workspace_high"]))


def apply_wrist_decomposition(xml):
    directory = Path(__file__).parent / "assets/wrist"
    manifest = read_json(directory / "manifest.json")
    assets = xml.find("asset")
    originals = {m.get("name"): m for m in assets.findall("mesh")}
    for name, entry in manifest["geoms"].items():
        geom = next(g for g in xml.findall(".//geom") if g.get("name") == name)
        if sha256(originals[geom.get("mesh")].get("file")) != entry["source_mesh_sha256"]:
            raise ValueError("Wrist collision source changed; regenerate convex decomposition")
        parent = next(b for b in xml.iter() if geom in list(b))
        for i, part in enumerate(entry["parts"]):
            path = directory / part["file"]
            if sha256(path) != part["sha256"]:
                raise ValueError("Convex decomposition hash mismatch")
            mesh_name = f"rl_{name}_{i}"
            ET.SubElement(assets, "mesh", name=mesh_name, file=str(path.resolve()))
            attributes = dict(geom.attrib)
            attributes.update(name=f"{name}_piece{i}", mesh=mesh_name)
            ET.SubElement(parent, "geom", **attributes)
        parent.remove(geom)
    return manifest


def build_model(root, config, output):
    source = Path(root) / config["source_model"]
    tree = ET.parse(source)
    xml = tree.getroot()
    compiler = xml.find("compiler")
    meshdir = source.parent / compiler.get("meshdir", "")
    source_mesh_hashes = {}
    for mesh in xml.findall("asset/mesh"):
        path = (meshdir / mesh.attrib["file"]).resolve()
        source_mesh_hashes[path.name] = sha256(path)
        mesh.set("file", str(path))
    compiler.attrib.pop("meshdir", None)
    full = mujoco.MjModel.from_xml_string(ET.tostring(xml, encoding="unicode"))
    state = mujoco.MjData(full)
    non_arm = []
    for j in range(full.njnt):
        name = mujoco.mj_id2name(full, mujoco.mjtObj.mjOBJ_JOINT, j)
        if name not in ARM_JOINTS:
            non_arm.append(name)
            if name not in config["hand_pose_rad"]:
                raise ValueError(f"Missing fixed hand coordinate: {name}")
            value = float(config["hand_pose_rad"][name])
            if not np.isfinite(value) or not full.jnt_range[j, 0] <= value <= full.jnt_range[j, 1]:
                raise ValueError(f"Hand pose outside model joint range: {name}")
            state.qpos[full.jnt_qposadr[j]] = value
    if set(non_arm) != set(config["hand_pose_rad"]):
        raise ValueError("Hand pose contains unknown/missing joints")
    mujoco.mj_forward(full, state)
    for body in xml.findall(".//body"):
        removed = [j for j in body.findall("joint") if j.get("name") not in ARM_JOINTS]
        if removed:
            bid = object_id(full, mujoco.mjtObj.mjOBJ_BODY, body.attrib["name"])
            parent = full.body_parentid[bid]
            rp = state.xmat[parent].reshape(3, 3)
            pos = rp.T @ (state.xpos[bid] - state.xpos[parent])
            rot = rp.T @ state.xmat[bid].reshape(3, 3)
            quat = np.empty(4)
            mujoco.mju_mat2Quat(quat, rot.reshape(9))
            body.set("pos", " ".join(map(str, pos)))
            body.set("quat", " ".join(map(str, quat)))
            for joint in removed:
                body.remove(joint)
    for name in ("equality", "keyframe", "actuator"):
        for child in list(xml.findall(name)):
            xml.remove(child)
    decomposition = apply_wrist_decomposition(xml)
    physics = config["physics"]
    xml.find("option").set("gravity", " ".join(map(str, physics["gravity"])))
    xml.find("option").set("timestep", str(physics["timestep"]))
    actuator = ET.SubElement(xml, "actuator")
    for i, name in enumerate(ARM_JOINTS):
        joint = next(j for j in xml.findall(".//joint") if j.get("name") == name)
        joint.set("damping", str(physics["joint_damping"]))
        joint.set("armature", str(physics["joint_armature"]))
        limit = physics["torque_limits"][i]
        ET.SubElement(actuator, "motor", name=name, joint=name, gear="1", ctrllimited="true", ctrlrange=f"{-limit} {limit}")
    palm = next(s for s in xml.findall(".//site") if s.get("name") == config["palm_site"])
    parent = next(b for b in xml.iter() if palm in list(b))
    r = np.empty(9)
    mujoco.mju_quat2Mat(r, np.fromstring(palm.get("quat", "1 0 0 0"), sep=" "))
    position = np.fromstring(palm.get("pos", "0 0 0"), sep=" ") + r.reshape(3, 3) @ config["tcp_offset_palm_m"]
    ET.SubElement(parent, "site", name="palm_tcp", pos=" ".join(map(str, position)),
                  quat=palm.get("quat", "1 0 0 0"), size="0.009", rgba="0 1 0 1")
    ET.SubElement(xml.find("worldbody"), "site", name="goal_marker", pos="0 0 1", size="0.02", rgba="1 0.2 0.1 0.6")
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output, encoding="utf-8", xml_declaration=True)
    model = mujoco.MjModel.from_xml_path(str(output))
    indices = ids(model, config)
    mujoco.mj_saveModel(model, str(output.with_suffix(".mjb")), None)
    # Verify that freezing has preserved every source body transform at arm zero.
    frozen = mujoco.MjData(model)
    mujoco.mj_forward(model, frozen)
    errors = []
    for bid in range(1, full.nbody):
        name = mujoco.mj_id2name(full, mujoco.mjtObj.mjOBJ_BODY, bid)
        new_id = object_id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        errors.append(float(np.linalg.norm(state.xpos[bid] - frozen.xpos[new_id])))
        errors.append(float(np.linalg.norm(state.xmat[bid] - frozen.xmat[new_id])))
    if max(errors) > 1e-7:
        raise RuntimeError("Freezing changed hand geometry unexpectedly")
    if not np.isclose(np.sum(full.body_mass), np.sum(model.body_mass), atol=1e-9, rtol=0):
        raise RuntimeError("Freezing changed total modeled mass")
    manifest = {"source_model_sha256": sha256(source), "source_mesh_sha256": source_mesh_hashes,
                "config_sha256": config_hash(config), "training_model_sha256": sha256(output),
                "training_binary_sha256": sha256(output.with_suffix(".mjb")),
                "nq": model.nq, "nv": model.nv, "nu": model.nu,
                "frozen_joint_names": non_arm, "max_freeze_transform_error": max(errors),
                "total_modeled_mass_kg": float(np.sum(model.body_mass)),
                "collision_policy": "link5/link7 split into component meshes with per-component convex collision; planar patches retained with nearest solid; other meshes/exclusions unchanged",
                "wrist_decomposition": decomposition,
                "hardware_ready": False}
    write_json(output.with_suffix(".manifest.json"), manifest)
    return model, indices, manifest
