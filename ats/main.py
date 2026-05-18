# 모듈 조립 및 실행

def main(): # 앞선 코드의 최상위 코드, 실행의 시작과 끝을 담당
    cfg = load_cfg() # 경로 / 프레임 / 주파수 / 스케일 / 제어 파라미터 로드
    sim = SimulationApp({
        "headless": cfg["sim"]["headless"],
        "load_extensions": cfg["sim"]["load_extensions"],
    }) # Isaac 부팅
    sim.update() # 부팅 직후 1프레임 플러시 (타입/플러그인 등록 완료용)

    # 모듈 임포트 : Isaac 런타임 이후 임포트해야 초기화 누락/타입 미등록 문제 방지
    from app.world import SimWorld, ensure_lidar_prims
    from app.graph_builder import GraphBuilder
    from app.policy import PolicyRunner
    from app.observation import ObservationBuilder
    from app.controller import RobotController
    from app.input import TeleopInput
    from omni.isaac.core.utils.prims import define_prim
    import omni.timeline

    world = SimWorld( # 앞선 코드의 최상위 코드, 실행의 시작과 끝을 담당
        usd_path=cfg["assets"]["usd_path"],
        spot_prim=cfg["assets"]["spot_prim"],
        ats_prim=cfg["assets"]["ats_prim"],
        imu_dummy_prim=cfg["assets"]["imu_prim"],
        fixed_time_step=cfg["player"]["fixed_time_step"],
        play_every_frame=cfg["player"]["play_every_frame"],
        target_hz=cfg["player"]["target_hz"],
    ) # USD 로드, Spot / ATS → ArticulationView 래핑하는 역할
    # /World/odom 등 프레임 정리
    # fixed_time.step / target_hz / play_every_frame 설정
    # 시뮬레이션 타임라인과 메인 루프를 동일 주기로 고정

    import omni.usd, omni.kit.app
    from pxr import UsdGeom, Sdf
    import omni.graph.core as og

    app = omni.kit.app.get_app()
    stage = omni.usd.get_context().get_stage()

    gb = GraphBuilder(cfg["assets"], cfg["ros"])
    gb.build_camera_ros_graph() # RenderProduct 생성, 카메라 프림 바인딩, 카메라 정보 ROS 전달
    gb.build_ats_graph() # 프레임 마다 자동 평가, cmd_vel 구독, 조인트 상태 퍼블리시 등 관리
    gb.build_lidar_ros_graph(cfg, graph_path="/LidarGraph") # LiDAR 그래프 생성
    # OnPlaybackTick → RunOneSimulationFrame →  RenderProduct → LidarHelper 순서 고정 // 레이스 조건 없는 /scan / /point_cloud 출력 보장
    sim.update()

    from isaacsim.core.utils.render_product import get_camera_prim_path

    rp2d = og.Controller.get(og.Controller.attribute("/LidarGraph/RP_2D.outputs:renderProductPath"))
    print("[CHK] RP_2D:", rp2d)
    print("[CHK] RP_2D cameraPrim:", get_camera_prim_path(rp2d)) # 프림 바인딩 빠른 검증

    timeline = omni.timeline.get_timeline_interface()
    timeline.play() # 시뮬레이션 틱 & OnPlaybackTick 그래프 활성화 센서 / Odom / TF 퍼블리시 시작
    sim.update()


    policy = PolicyRunner(cfg["policy"]["path"], cfg["policy"]["device"]) # TorchScript 모델 로드 (.eval / no_grad)

    ### [ADVICE] obsb.build(cmd_vec)가 생성하는 차원(예: 48차원)과 policy.infer()가 기대하는 입력 차원이 
    ### isaaclab의 SpotObservationsCfg와 정확히 일치하는지 검증하는 로직을 추가하는 것을 권장합니다.
    default_pos = world.spot.get_joint_positions().squeeze(0) + 1.0
    obsb = ObservationBuilder(world.spot, default_pos) # Spot 기본자세 기반 48차원 관측 준비

    ctrl = RobotController( # Spot/ATS 아티큘레이션 제어기
        world.spot,
        world.ats,
        cfg["controls"]["spot_action_scale"],
        cfg["controls"]["ats_joint_step"],
    )

    teleop = TeleopInput() # 키 입력 폴링(teleop 제어)

    try:
        while sim.is_running(): # 매 프레임 루프를 반복
            world.step(render=True) # 물리, 렌더, 센서 상태를 갱신
            
            teleop_vec, ats_cmd = ctrl.teleop_from_keys(
                teleop.pressed,
                cfg["controls"]["teleop"]["lin_speed"],
                cfg["controls"]["teleop"]["ang_speed_yaw"],
                cfg["controls"]["teleop"]["ang_speed_pitch"],
            )
            
            vx_r, vy_r, vz_r = RobotController.read_twist_from_graph()
            cmd_vec = np.array( # 자율 제어를 하나의 명령 벡터로 합성
                [teleop_vec[0] + vx_r, teleop_vec[1] + vy_r, teleop_vec[2] + vz_r],
                dtype=np.float32,
            ) # 구독한 cmd_vel(vx_r, vy_r, vz_r$과 teleop_vec과 합성
            
            obs = obsb.build(cmd_vec) # 몸체 기준 선,각속도, 중력 방향, 명령, 관절 상태, 직전 액션을 모아 12차원으로 패킹
            action = policy.infer(obs) # 12차원 보행 액션을 즉시 반환
            ctrl.apply_actions(action, ats_cmd)
            obsb.update_prev_action(action) # 이전 액션의 정보를 다음에 위상 정보 전달 
            RobotController.trigger_graph_impulse() # 특정 이벤트나 초기화 트리거 역할

    finally:
        sim.close() # Isaac 프로세스 종료

if __name__ == "__main__":
    main()
