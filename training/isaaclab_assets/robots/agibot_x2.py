import isaaclab.sim as sim_utils
from isaaclab.actuators import DCMotorCfg
from isaaclab.assets.articulation import ArticulationCfg

X2_USD_PATH = "/home/lirui/桌面/lingxi-bot/research/x2_usd/x2_legs_only.usd"

X2_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=X2_USD_PATH,
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=4,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.90),
        joint_pos={
            ".*_hip_pitch_joint": -0.20,
            ".*_knee_joint": 0.42,
            ".*_ankle_pitch_joint": -0.23,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs": DCMotorCfg(
            joint_names_expr=[
                ".*_hip_pitch_joint",
                ".*_hip_roll_joint",
                ".*_hip_yaw_joint",
                ".*_knee_joint",
                ".*_ankle_pitch_joint",
                ".*_ankle_roll_joint",
            ],
            effort_limit={
                ".*_hip_.*_joint": 120.0,
                ".*_knee_joint": 120.0,
                ".*_ankle_pitch_joint": 60.0,
                ".*_ankle_roll_joint": 36.0,
            },
            velocity_limit={
                ".*_hip_.*_joint": 11.94,
                ".*_knee_joint": 11.94,
                ".*_ankle_pitch_joint": 13.61,
                ".*_ankle_roll_joint": 14.66,
            },
            saturation_effort=120.0,
            stiffness=30.0,
            damping=0.5,
            friction=0.0,
        ),
    },
)
