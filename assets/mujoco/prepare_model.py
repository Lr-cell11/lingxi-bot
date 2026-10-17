from pathlib import Path
import hashlib
import json
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


SOURCE = Path('/home/lirui/桌面/lingxi-bot/research/agibot_x2_urdf-main/X2_URDF-v1.4.0')
OUTPUT = Path('/home/lirui/桌面/lingxi-bot/research/x2_sim2sim')
NAMES = [f'{side}_{part}_joint' for part in (
    'hip_pitch', 'hip_roll', 'hip_yaw', 'knee', 'ankle_pitch', 'ankle_roll'
) for side in ('left', 'right')]
DEFAULT = np.array([-.2, -.2, 0, 0, 0, 0, .42, .42, -.23, -.23, 0, 0])
EXPECTED_COM = np.array([.00013845636567566544, .0003156615130137652, .25468626618385315])


def main():
    # Refuse overwrites and retain official source assets unchanged.
    paths = [OUTPUT / name for name in (
        'X2-legs-only.xml', 'scene.xml', 'model_check.json', 'README.md', 'prepare_model.py'
    )]
    if any(path.exists() for path in paths):
        raise FileExistsError('Output already exists; review it before regenerating.')
    original_hash = hashlib.sha256((SOURCE / 'X2-Ultra.xml').read_bytes()).hexdigest()
    root = ET.parse(SOURCE / 'X2-Ultra.xml').getroot()
    root.find('compiler').set('meshdir', str(SOURCE / 'meshes'))
    root.find('option').set('timestep', '0.005')
    removed = []
    for body in root.findall('.//worldbody//body'):
        for joint in list(body.findall('joint')):
            if joint.get('name') not in NAMES:
                removed.append(joint.get('name'))
                body.remove(joint)
    assert len(removed) == 19
    for section in ('actuator', 'sensor'):
        parent = root.find(section)
        for item in list(parent):
            if item.get('joint') in removed:
                parent.remove(item)
    for joint in root.iter('joint'):
        if joint.get('name') in NAMES or joint in root.findall('.//default//joint'):
            joint.set('armature', '0')
            joint.set('damping', '0')
            joint.set('frictionloss', '0')
    pelvis_xml = root.find(".//body[@name='pelvis']")
    pelvis_xml.set('pos', '0 0 0.9')
    # Collision bitmasks permit robot-ground contact, but not robot-robot contact.
    for geom in root.findall('.//worldbody//geom'):
        if geom.get('class') != 'visual':
            geom.set('contype', '1')
            geom.set('conaffinity', '2')
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    assert (model.nq, model.nv, model.nu) == (19, 18, 12)
    data = mujoco.MjData(model)
    mapping = []
    for index, name in enumerate(NAMES):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        aids = np.flatnonzero(model.actuator_trnid[:, 0] == jid)
        assert jid >= 0 and len(aids) == 1
        aid = int(aids[0])
        assert model.actuator_trntype[aid] == mujoco.mjtTrn.mjTRN_JOINT
        assert np.allclose(model.actuator_gear[aid], [1, 0, 0, 0, 0, 0])
        qa, va = int(model.jnt_qposadr[jid]), int(model.jnt_dofadr[jid])
        data.qpos[qa] = DEFAULT[index]
        mapping.append(dict(policy_index=index, joint=name, qpos_addr=qa,
                            qvel_addr=va, actuator_id=aid, default_pos=float(DEFAULT[index])))
    assert np.allclose(model.dof_armature, 0)
    assert np.allclose(model.dof_damping, 0)
    assert np.allclose(model.dof_frictionloss, 0)
    mujoco.mj_forward(model, data)
    pelvis = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'pelvis')
    welded = np.flatnonzero(model.body_weldid == model.body_weldid[pelvis])
    mass = model.body_mass[welded]
    com_world = (data.xipos[welded] * mass[:, None]).sum(axis=0) / mass.sum()
    com = data.xmat[pelvis].reshape(3, 3).T @ (com_world - data.xpos[pelvis])
    assert np.isclose(model.body_mass.sum(), 44.801)
    assert np.isclose(mass.sum(), 26.701)
    assert np.allclose(com, EXPECTED_COM, atol=1e-6, rtol=0)
    keyframe = ET.SubElement(root, 'keyframe')
    ET.SubElement(keyframe, 'key', name='stand',
                  qpos=' '.join(f'{x:.17g}' for x in data.qpos),
                  qvel=' '.join('0' for _ in range(model.nv)))
    scene = ET.parse(SOURCE / 'scene.xml').getroot()
    scene.find('include').set('file', 'X2-legs-only.xml')
    floor = scene.find(".//geom[@name='floor']")
    floor.set('contype', '2')
    floor.set('conaffinity', '1')
    report = dict(mujoco_version=mujoco.__version__, removed_joints=removed,
                  nq=model.nq, nv=model.nv, nu=model.nu, physics_dt=model.opt.timestep,
                  total_mass=float(model.body_mass.sum()), welded_root_mass=float(mass.sum()),
                  welded_root_com_b=com.tolist(), com_difference_meters=(com-EXPECTED_COM).tolist(),
                  mapping=mapping, source_sha256=original_hash, simulation_time=float(data.time),
                  pending=['Friction calibration', 'ONNX interface check', 'Observation unit tests',
                           'DCMotor implementation', 'Policy rollout'])
    OUTPUT.mkdir(parents=True, exist_ok=True)
    ET.indent(root)
    ET.indent(scene)
    ET.ElementTree(root).write(paths[0], encoding='utf-8', xml_declaration=True)
    ET.ElementTree(scene).write(paths[1], encoding='utf-8', xml_declaration=True)
    # Recompile saved scene and verify the standing keyframe without stepping.
    saved = mujoco.MjModel.from_xml_path(str(paths[1]))
    saved_data = mujoco.MjData(saved)
    mujoco.mj_resetDataKeyframe(saved, saved_data, 0)
    mujoco.mj_forward(saved, saved_data)
    assert (saved.nq, saved.nv, saved.nu) == (19, 18, 12)
    assert np.allclose(saved_data.qpos, data.qpos)
    assert saved_data.time == 0
    assert hashlib.sha256((SOURCE / 'X2-Ultra.xml').read_bytes()).hexdigest() == original_hash
    report['saved_scene_verified'] = True
    paths[2].write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    paths[3].write_text('# X2 腿部 MuJoCo 独立模型\n\n'
        '从官方 v1.4.0 MJCF 派生；上身关节固定于零位，保留全部质量与惯性。\n'
        'scene.xml 包含地面；stand keyframe 提供默认站姿。控制器需主动加载该 keyframe。\n'
        '200 Hz 物理，腿关节被动阻尼/摩擦/armature 为零；需另行实现显式 PD 与 DCMotor。\n'
        '碰撞掩码仅允许机器人与地面接触，视觉几何不参与碰撞。\n'
        '保留原始接触摩擦参数，尚未对齐 Isaac 摩擦，不能声称动力学完全等价。\n'
        'meshes 使用官方资产的绝对路径；移动工程后需更新路径。\n'
        'model_check.json 为实际编译检查输出。未运行 mj_step 或策略推理。\n'
        'prepare_model.py 是生成脚本，拒绝覆盖已有输出。\n', encoding='utf-8')
    paths[4].write_text(Path(__file__).read_text(encoding='utf-8'), encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f'OUTPUT_DIR: {OUTPUT}')


if __name__ == '__main__':
    main()
