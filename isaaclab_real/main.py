import argparse
import os
import sys
import numpy as np
import torch

# 1. Isaac Sim 초기화
from omni.isaac.kit import SimulationApp

# 설정을 먼저 읽기 위한 임시 시뮬레이션 앱 (필요한 경우)
# 실제로는 AppLauncher를 쓰거나 SimulationApp을 바로 띄웁니다.
# 여기서는 일반적인 추론 스크립트 형식을 따릅니다.

def load_cfg(config_path=None):
    """설정 파일을 로드하는 유틸리티 함수"""
    import yaml
    if config_path is None:
        config_path = os.environ.get("ATS_CONFIG", "config/complete_asm.yaml")
    
    if not os.path.exists(config_path):
        print(f"[Warning] Config file not found at {config_path}. Using default empty dict.")
        return {}
        
    with open(config_path, "r") as f:
        return yaml.safe_load(f)

def main():
    # CLI 인자 처리
    parser = argparse.ArgumentParser(description="Inference script for Isaac Lab Real.")
    parser.add_argument("--config", type=str, default=None, help="Path to the configuration file.")
    parser.add_argument("--headless", action="store_true", default=False, help="Run in headless mode.")
    args = parser.parse_args()

    # -----1. config로딩과 시뮬레이션 asset 설정
    cfg = load_cfg(args.config)
    
    # Simulation App 실행
    sim_config = {
        "headless": args.headless or cfg.get("sim", {}).get("headless", False),
        "load_extensions": cfg.get("sim", {}).get("load_extensions", True),
    }
    simulation_app = SimulationApp(sim_config)

    # --- Isaac Sim 환경 구성 ---
    # 중요: SimulationApp 실행 후에만 아래 모듈들을 임포트할 수 있습니다.
    from ats.world import SimWorld
    from ats.controller import RobotController
    from ats.observation import ObservationBuilder
    from ats.policy import PolicyRunner
    import omni.timeline

    # 2. 월드 및 로봇 초기화
    world = SimWorld(
        usd_path=cfg["assets"]["usd_path"],
        spot_prim=cfg["assets"]["spot_prim"],
        ats_prim=cfg["assets"]["ats_prim"],
        imu_dummy_prim=cfg["assets"]["imu_prim"],
        fixed_time_step=cfg["player"]["fixed_time_step"],
        play_every_frame=cfg["player"]["play_every_frame"],
        target_hz=cfg["player"]["target_hz"],
    )

    # 3. 정책(Policy) 및 관측(Observation) 준비
    policy = PolicyRunner(cfg["policy"]["path"], cfg["policy"]["device"])
    
    # 초기 관절 위치 획득, 관측 벡터 구성
    default_pos = world.spot.get_joint_positions().squeeze(0)
    obsb = ObservationBuilder(world.spot, default_pos)
    
    # 4. 제어기(Controller) 설정
    ctrl = RobotController(
        world.spot,
        world.ats,
        cfg["controls"]["spot_action_scale"],
        cfg["controls"]["ats_joint_step"],
    )

    # 타임라인 시작
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    print("[Info] Starting inference loop...")

    # 5. 메인 추론 루프
    try:
        while simulation_app.is_running():
            # 시뮬레이션 스텝 진행
            world.step(render=True)
            
            # (예시) 명령 벡터: 실제로는 ROS2 cmd_vel이나 키보드 입력을 받아야 함
            # 여기서는 정지 상태(0, 0, 0)를 기본값으로 함
            cmd_vec = np.zeros(3, dtype=np.float32)
            
            # 관측 생성
            obs = obsb.build(cmd_vec)
            
            # 정책 추론 (Action 결정)
            action = policy.infer(obs)
            
            # 로봇에 액션 적용
            ctrl.apply_actions(action, ats_cmd={})
            
            # 다음 스텝을 위해 액션 저장
            obsb.update_prev_action(action)

    except KeyboardInterrupt:
        print("[Info] Interrupt received, shutting down...")
    finally:
        # 6. 종료 및 리소스 정리
        simulation_app.close()

if __name__ == "__main__":
    main()
