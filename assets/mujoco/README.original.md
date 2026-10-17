# X2 腿部 MuJoCo 独立模型

从官方 v1.4.0 MJCF 派生；上身关节固定于零位，保留全部质量与惯性。
scene.xml 包含地面；stand keyframe 提供默认站姿。控制器需主动加载该 keyframe。
200 Hz 物理，腿关节被动阻尼/摩擦/armature 为零；需另行实现显式 PD 与 DCMotor。
碰撞掩码仅允许机器人与地面接触，视觉几何不参与碰撞。
保留原始接触摩擦参数，尚未对齐 Isaac 摩擦，不能声称动力学完全等价。
meshes 使用官方资产的绝对路径；移动工程后需更新路径。
model_check.json 为实际编译检查输出。未运行 mj_step 或策略推理。
prepare_model.py 是生成脚本，拒绝覆盖已有输出。
