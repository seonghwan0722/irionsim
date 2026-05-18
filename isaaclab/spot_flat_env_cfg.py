# Spot이 평지 및 Cobbles(거친 지형)에서도 사용자가 원하는 속도대로 안정적으로 걷도록 훈련하는 환경 설정 파일
# ----1. 행동 정의 ----
@configclass
class SpotActionsCfg:
    """Action specifications for the MDP."""

    joint_pos = mdp.JointPositionActionCfg(asset_name="robot", joint_names=[".*"], scale=0.2, use_default_offset=True)
    # 토크를 직접 제어하지 않고, 관절의 목표 위치(Position)를 제어하도록 설정
        # scale=0.2: 설정 수치만큼 값을 곱해 실제 관절 움직임을 축소 또는 증폭
            # 관절 떨림과 하드웨어 부담을 줄이는 안전장치 역할을 하며 안정적인 학습을 유도
        # joint_names=[".*"]: 정규표현식을 사용하여 Spot의 모든 관절에 동일한 규칙을 적용


# ---- 2. 명령 정의 ---
@configclass
class SpotCommandsCfg:
    """Command specifications for the MDP."""

    base_velocity = mdp.UniformVelocityCommandCfg( # 학습 도중 무작위 속도 명령을 균일 분포(Uniform Distribution)로 샘플링
        asset_name="robot",
        resampling_time_range=(10.0, 10.0), # 10초마다 로봇에게 새로운 목표 속도를 부여
        rel_standing_envs=0.1,                  #한 가지 동작만 외우는 것이 아니라, 다양한 상황에 적응하는 보행 정책을 학습
        ... # scale=0.2: 설정 수치만큼 값을 곱해 실제 관절 움직임을 축소 또는 증폭
        ranges=mdp.UniformVelocityCommandCfg.Ranges( # 속도 범위 설정
            lin_vel_x=(-2.0, 3.0), lin_vel_y=(-1.5, 1.5), ang_vel_z=(-2.0, 2.0)
        ), # => 뒤로 최대 2m/s, 앞으로 최대 3m/s까지 이동하는 연습
    )

# ---- 3. 관측 정의 ----
# 로봇이 현재 자신의 상태를 파악하기 위해 환경으로부터 받아오는 정보들입니다.
@configclass
class SpotObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # observation terms (order preserved)
        base_lin_vel = ObsTerm(
            func=mdp.base_lin_vel, params={"asset_cfg": SceneEntityCfg("robot")}, noise=Unoise(n_min=-0.1, n_max=0.1)
        )
        base_ang_vel = ObsTerm(
            func=mdp.base_ang_vel, params={"asset_cfg": SceneEntityCfg("robot")}, noise=Unoise(n_min=-0.1, n_max=0.1)
        ) # => 로봇 몸체가 얼마나 빨리 움직이고(선속도) 돌고 있는지(각속도)를 나타내는 물리량
        projected_gravity = ObsTerm(
            func=mdp.projected_gravity,
            params={"asset_cfg": SceneEntityCfg("robot")},
            noise=Unoise(n_min=-0.05, n_max=0.05),
        ) # => 중력 벡터를 기준으로 몸체가 얼마나 기울어졌는지를 측정하는 평형 감각 데이터
        velocity_commands = ObsTerm(func=mdp.generated_commands, params={"command_name": "base_velocity"})
         # => 사용자로부터 입력된 이동 명령(목표 속도 등) 정보
        joint_pos = ObsTerm(
            func=mdp.joint_pos_rel, params={"asset_cfg": SceneEntityCfg("robot")}, noise=Unoise(n_min=-0.05, n_max=0.05)
        ) # => 로봇 관절들의 상대적인 위치 정보를 포함                                         # => 시뮬레이터의 깨끗한 데이터에 의도적으로 랜덤 노이즈를 섞음 (실제 센서에서 받는 미세한 떨림, 오차, 지터의 존재에 대한 대응)

# ----4. 랜덤화 이벤트 정의 ----
# 학습의 강인성(Robustness)을 높이기 위해 시뮬레이션 환경의 물리적 조건을 무작위로 변경하는 설정
@configclass
class SpotEventCfg:
    """Configuration for randomization."""

    # startup
    physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.3, 1.0),
            "dynamic_friction_range": (0.3, 0.8),
            ...
        }, # -> 에피소드 시작마다 바닥의 마찰 계수를 0.3 ~ 1.0 사이에서 랜덤하게 설정
    ) # => 미끄러운 대리석 바닥부터 거친 아스팔트까지 매번 다른 지면 환경에 로봇을 배치하여 학습시키는 효과

    add_base_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="body"),
            "mass_distribution_params": (-2.5, 2.5),
            "operation": "add",
        }, # -> 로봇 몸체(base)의 질량을 랜덤하게 추가하거나 변경
    ) # => mass_distribution_params를 (-2.5, 2.5) 범위로 설정하여 로봇이 다양한 무게 변화에도 균형을 유지하도록 훈련
    
    
    # interval
    push_robot = EventTerm(  # 로봇을 강제로 밀어내는 효과
        func=mdp.push_by_setting_velocity,
        mode="interval", # 10~15초마다 한 번씩 주기적으로 실행
        interval_range_s=(10.0, 15.0),
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "velocity_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5)},
        }, 
    ) # => - 옆에서 누군가 툭 치는 것처럼, x/y 방향 속도를 랜덤하게 주입하여 로봇의 외란 극복 능력을 훈련

# ---- 5. 보상 정의 ----
# 로봇이 목표하는 동작을 얼마나 잘 수행하고 있는지 수치화하여 학습을 유도하는 설정
@configclass
class SpotRewardsCfg:
    # -- task
    base_linear_velocity = RewardTermCfg(
        func=spot_mdp.base_linear_velocity_reward,
        weight=5.0,
        params={"std": 1.0, "ramp_rate": 0.5, "ramp_at_vel": 1.0, "asset_cfg": SceneEntityCfg("robot")},
    ) # -> 로봇이 사용자가 명령한 속도대로 잘 이동하고 있는지 확인하는 변수
    # -> 명령 속도와 실제 로봇의 속도가 일치할수록 더 높은 보상을 부여
    gait = RewardTermCfg(
        func=spot_mdp.GaitReward,
        weight=10.0,
        params={
            "std": 0.1,
            "synced_feet_pair_names": (("fl_foot", "hr_foot"), ("fr_foot", "hl_foot")),
            ...
        }, # -> 구체적인 보행 박자를 강제하여 로봇의 자연스러운 걸음걸이를 유도
    )# ("fl_foot", "hr_foot"), ("fr_foot", "hl_foot")와 같이 발을 짝지어 설정
        # => 이는 '왼쪽 앞발+오른쪽 뒷발', '오른쪽 앞발+왼쪽 뒷발'이 동시에 지면에 닿도록 대각선 발을 맞춰 걷게 하는 효과
    
    # -- penalties
    base_orientation = RewardTermCfg(
        func=spot_mdp.base_orientation_penalty, weight=-3.0, params={"asset_cfg": SceneEntityCfg("robot")}
    ) # -> 로봇의 몸통이 기울어질수록 더 큰 벌점을 부여
    # => 로봇이 걷는 동안에도 등판을 최대한 수평에 가깝게 유지하며 안정적으로 보행하도록 학습
    joint_torques = RewardTermCfg(
        func=spot_mdp.joint_torques_penalty,
        weight=-5.0e-4,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*")},
    ) # -> 관절 토크(힘)를 많이 사용할수록 벌점을 부여
    # => "필요 이상으로 힘을 쓰지 마라"라는 원칙을 학습시켜, 에너지 효율을 높이고 기계적으로 부드러운 움직임을 생성하도록 유도

# ----0. 환경 구성 ---
# 시뮬레이션의 물리 계산 주기와 제어 주기를 설정하여 학습된 정책이 실제 로봇에서도 동일하게 작동하도록 맞추는 과정
def __post_init__(self):
    # 부모 클래스 초기화 호출
    super().__post_init__()

    # 일반 설정 (제어 주기 관련)
    # 물리 법칙 계산을 설정된 횟수만큼 반복한 뒤, 제어 액션을 1번 업데이트하도록 설정하는 값
    self.decimation = 10  # 50 Hz (0.002s * 10 = 0.02s)
    
    # 시뮬레이션 설정 (물리 엔진 업데이트 주기)
    self.sim.dt = 0.002   # 500 Hz

    # terrain 설정
    self.scene.terrain = TerrainImporterCfg( # 시뮬레이션 환경의 바닥면(지형)을 설정하는 클래스
        prim_path="/World/ground",
        terrain_type="generator",
        terrain_generator=COBBLESTONE_ROAD_CFG, # 자갈길 생성기 사용 => 로봇이 평지 뿐 아니라 욽퉁불퉁한 노면에서도 적응하며 걸을 수 있도록 환경 조성
        ...
    )
    class SpotFlatEnvCfg_PLAY(SpotFlatEnvCfg):
    # 학습이 완료된 모델을 실제로 구동하거나 테스트할 때 사용하는 설정 클래스
    def __post_init__(self) -> None:
        ...
        # 병렬 환경 수를 50개로 제한 (데모 및 시각화용) 학습 시 보다 적은 수의 환경만 가볍게 띄워 시각화나 데모에 집중
        self.scene.num_envs = 50
        
        # 센서 잡음(Noise) 비활성화 (적용 여부 설정) => 실행 단계에서는 의도적인 잡음을 제거하여 모델의 순수한 추론 결과를 확인하기 위해 False로 설정
        self.observations.policy.enable_corruption = False
    