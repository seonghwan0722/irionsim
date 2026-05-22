from isaaclab.utils import configclass
from isaaclab.envs import ManagerBasedRLEnvCfg, mdp
from isaaclab.managers import SceneEntityCfg, ObsTerm, ObsGroup, RewardTermCfg, EventTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils.noise import Unoise

@configclass # 1. 행동 정의
class CompleteAsmActionsCfg:
    """Action specifications for the MDP."""
    joint_pos = mdp.JointPositionActionCfg(asset_name="robot", joint_names=[".*"], scale=1.0, use_default_offset=True)

@configclass # 2. 명령 정의
class SpotCommandsCfg:
    """Command specifications for the MDP."""
    base_velocity = mdp.UniformVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(10.0, 10.0),
        rel_standing_envs=0.1,
        ... # scale=0.2: 설정 수치만큼 값을 곱해 실제 관절 움직임을 축소 또는 증폭
        ranges=mdp.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(-2.0, 3.0), lin_vel_y=(-1.5, 1.5), ang_vel_z=(-2.0, 2.0)
        ),
    )

@configclass # 3. 관측 정의
class CompleteAsmObservationsCfg:
    """Observation specifications for the MDP."""
    @configclass
    class PolicyCfg(ObsGroup):
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel, params={"asset_cfg": SceneEntityCfg("robot")}, noise=Unoise(n_min=-0.1, n_max=0.1))
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel, params={"asset_cfg": SceneEntityCfg("robot")}, noise=Unoise(n_min=-0.1, n_max=0.1))
        projected_gravity = ObsTerm(func=mdp.projected_gravity, params={"asset_cfg": SceneEntityCfg("robot")}, noise=Unoise(n_min=-0.05, n_max=0.05))
        velocity_commands = ObsTerm(func=mdp.generated_commands, params={"command_name": "base_velocity"})
        joint_pos = ObsTerm(func=mdp.joint_pos_rel, params={"asset_cfg": SceneEntityCfg("robot")}, noise=Unoise(n_min=-0.05, n_max=0.05))
        joint_vel = ObsTerm(func=mdp.joint_vel_rel, params={"asset_cfg": SceneEntityCfg("robot")}, noise=Unoise(n_min=-0.05, n_max=0.05))

@configclass # 4. 랜덤화 이벤트 정의
class CompleteAsmEventCfg:
    """Configuration for randomization."""
    physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.3, 1.0),
            "dynamic_friction_range": (0.3, 0.8),
            ...
        }, # -> 에피소드 시작마다 바닥의 마찰 계수를 0.3 ~ 1.0 사이에서 랜덤하게 설정
    )
    add_base_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="body"),
            "mass_distribution_params": (-2.5, 2.5),
            "operation": "add",
        }, # -> 로봇 몸체(base)의 질량을 랜덤하게 추가하거나 변경
    )
    push_robot = EventTerm(  # 로봇을 강제로 밀어내는 효과
        func=mdp.push_by_setting_velocity,
        mode="interval", # 10~15초마다 한 번씩 주기적으로 실행
        interval_range_s=(10.0, 15.0),
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "velocity_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5)},
        }, 
    )
    reset_robot = EventTerm(
        func=mdp.reset_joints,
        mode="reset",
        params={"asset_cfg": SceneEntityCfg("robot")},
    )

@configclass # 5. 보상 정의
class CompleteAsmRewardsCfg:
    """Reward terms for the MDP."""
    track_lin_vel_xy_exp = RewardTermCfg(
        func=mdp.track_lin_vel_xy_exp, weight=1.0, params={"command_name": "base_velocity", "std": 0.5}
    )
    track_ang_vel_z_exp = RewardTermCfg(
        func=mdp.track_ang_vel_z_exp, weight=0.5, params={"command_name": "base_velocity", "std": 0.5}
    )
    # # -- task-----
    # base_linear_velocity = RewardTermCfg(
    #     func=spot_mdp.base_linear_velocity_reward,
    #     weight=5.0,
    #     params={"std": 1.0, "ramp_rate": 0.5, "ramp_at_vel": 1.0, "asset_cfg": SceneEntityCfg("robot")},
    # ) # -> 로봇이 사용자가 명령한 속도대로 잘 이동하고 있는지 확인하는 변수
    # # -> 명령 속도와 실제 로봇의 속도가 일치할수록 더 높은 보상을 부여
    # gait = RewardTermCfg(
    #     func=spot_mdp.GaitReward,
    #     weight=10.0,
    #     params={
    #         "std": 0.1,
    #         "synced_feet_pair_names": (("fl_foot", "hr_foot"), ("fr_foot", "hl_foot")),
    #         ...
    #     }, # -> 구체적인 보행 박자를 강제하여 로봇의 자연스러운 걸음걸이를 유도
    # )
    # ----- penalties -----
    base_orientation = RewardTermCfg(
        func=spot_mdp.base_orientation_penalty, weight=-3.0, params={"asset_cfg": SceneEntityCfg("robot")}
    ) # -> 로봇의 몸통이 기울어질수록 더 큰 벌점을 부여
    joint_torques = RewardTermCfg(
        func=spot_mdp.joint_torques_penalty,
        weight=-5.0e-4,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*")},
    ) # -> 관절 토크(힘)를 많이 사용할수록 벌점을 부여

@configclass # 0. 환경 구성
class CompleteAsmEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the environment."""
    # Scene settings
    super().__post_init__()

    scene = InteractiveSceneCfg(num_envs=4096, env_spacing=2.5)
    self.scene.terrain = TerrainImporterCfg( # 시뮬레이션 환경의 바닥면(지형)을 설정하는 클래스
        prim_path="/World/ground",
        terrain_type="generator",
        terrain_generator=COBBLESTONE_ROAD_CFG, # 자갈길 생성기 사용 => 로봇이 평지 뿐 아니라 욽퉁불퉁한 노면에서도 적응하며 걸을 수 있도록 환경 조성
        ...
    )
    # Robot
    robot = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/robot",
        spawn=AssetBaseCfg(
            usd_path="/home/korea/isaacsim/isaaclab_real/complete_asm/complete_asm.urdf",
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.5),
            joint_pos={".*": 0.0},
        ),
    )
    
    # MDP settings
    actions = CompleteAsmActionsCfg()
    observations = CompleteAsmObservationsCfg()
    events = CompleteAsmEventCfg()
    rewards = CompleteAsmRewardsCfg()

    def __post_init__(self):
        self.decimation = 4
        self.sim.dt = 0.005
        self.sim.render_dt = 0.02

class CompleteAsmCfg_PLAY(SpotFlatEnvCfg):
    # 학습이 완료된 모델을 실제로 구동하거나 테스트할 때 사용하는 설정 클래스
        def __post_init__(self) -> None:
            ...
            # 병렬 환경 수를 50개로 제한 (데모 및 시각화용) 학습 시 보다 적은 수의 환경만 가볍게 띄워 시각화나 데모에 집중
            self.scene.num_envs = 50
            
            # 센서 잡음(Noise) 비활성화 (적용 여부 설정) => 실행 단계에서는 의도적인 잡음을 제거하여 모델의 순수한 추론 결과를 확인하기 위해 False로 설정
            self.observations.policy.enable_corruption = False