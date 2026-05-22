# isaac lab을 이용한 강화학습
# add argparse arguments # --task, --num_envs, --video 등 사용자 옵션 정의
parser = argparse.ArgumentParser(description="Train an RL agent with RSL-RL.")
parser.add_argument("--video", action="store_true", default=False, help="Record videos during training.")
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
...
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# --headless 같은 Isaac Sim 전용 옵션을 자동으로 추가
args_cli, hydra_args = parser.parse_known_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
# 터미널에서 입력한 모든 설정을 args_cli에 정리해서 저장
# CLI 설정을 기반으로 Isaac Sim 실행기 구성

simulation_app = app_launcher.app
# 실행되고 Isaac Sim 엔진이 메모리에 로드
	# GUI 모드면 창이 뜨고, --headless면 백그라운드에서만 동작


# --- 1. 시뮬레이션 구동 ----
# 요약: App(시뮬레이터)을 먼저 켜고 그 위에 라이브러리를 업로드하는 구조
"""Rest everything follows."""
import gymnasium as gym
import os
import torch
...
from isaaclab.envs import import ManagerBasedRLEnvCfg

# 중간 import 입력: Isaac Sim이 파이썬 환경 및 GPU 자원을 먼저 점유하고 있어야 torch, gym이 충돌 없이 초기화 가능합니다.
# 이 순서를 어길 경우 CUDA/환경 충돌 및 초기화 에러 발생 가능성이 높아집니다.

# ---2. 설정의 확정 ---
# 요약: 2단계는 "Hydra가 설정 파일을 배달하고, CLI가 그 위를 덮어써서 최종 학습 계획표를 완성하는 단계"
@hydra_task_config(args_cli.task, args_cli.agent)
# task 이름의 라이브러리 안에서 해당하는 환경 설정, 에이전트 설정 파일을 자동으로 찾아서 로드

def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg, agent_cfg: RslRlOnPolicyRunnerCfg):
# 이미 파싱된 설정 객체(env_cfg, agent_cfg)가 인자로 "주입(injection)"
	# 개발자는 open()/yaml.load() 같은 코드 없이도 태스크 이름만 바꿔서 다양한 실험 설정을 바로 불러올 수 있음
	
"""Train with RSL-RL agent."""
# override configurations with non-hydra CLI arguments
agent_cfg = cli_args.update_rsl_rl_cfg(agent_cfg, args_cli)
env_cfg.scene.num_envs = args_cli.num_envs if args_cli.num_envs is not None else env_cfg.scene.num_envs
agent_cfg.max_iterations = (
    args_cli.max_iterations if args_cli.max_iterations is not None else agent_cfg.max_iterations
) # 사용자의 명령이 파일 설정보다 우선하는 원칙 적용
			# 사용자 명령 예) 기본값: num_envs=4096 -> --num_envs 1024로 덮어쓰기(Overriding) 가능

# set the environment seed
	# note: certain randomizations occur in the environment initialization so we set the seed here
	env_cfg.seed = agent_cfg.seed # 환경과 에이전트가 같은 시드를 쓰도록 맞춰 재현성 확보
	env_cfg.sim.device = args_cli.device if args_cli.device is not None else env_cfg.sim.device

# ------3. 환경생성과 포장 -----
# 요약 : gym.make로 창조된 다양한 기능의 가상 세계와 인공지능 알고리즘 사이의 통역사 + 어댑터
# create isaac environment
env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)
	# OpenAI 표준 인터페이스, 설정된 task와 args_cli, env_cfg를 바탕으로 시뮬레이션 환경 인스턴스 생성
	# num_envs : 설정된 숫자 만큼의 메모리 안에 평행 우주 동시에 생성
	
# wrap for video recording
if args_cli.video:
    video_kwargs = { ... }
    env = gym.wrappers.RecordVideo(env, **video_kwargs)
# RecordVideo : --video 옵션이 켜져 있으면 래퍼(RecordVideo) 한 겹 더 씌움
	# 로봇/환경 자체는 그대로 두고, "방송용 카메라맨" 처럼 학습 과정을 주기적으로 촬영하여 파일로 저장

# wrap around environment for rsl-rl
env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
# RslRlVecEnvWrapper : Isaac Lab(환경)과 RSL-RL(알고리즘)이 쓰는 데이터 형식이 다르기에 중간 '통역사' 역할
	# 역할 1 : 환경에서 나온 복잡한 데이터를 알고리즘이 원하는 텐서 포맷으로 변환
	# 역할 2 : 알고리즘이 낸 액션을 시뮬레이터가 이해할 물리 신호로 다시 변환

# -----4. 학습 시작 -----
# 요약: OnPolicyRunner로 학습을 돌리고, 설정을 로그에 남기고, 학습이 끝나면 환경을 정리하며 깔끔하게 종료
# create runner from rsl-rl
runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=log_dir, device=agent_cfg.device)
# PPO 같은 on-policy 알고리즘을 실제로 돌리고 관리하는 코치 객체
	# 초기화 입력값: 래핑된 환경(env), 딕셔너리로 바꾼 에이전트 설정(agent_cfg.to_dict()), 로그 저장 경로(log_dir), 학습 디바이스(device)

# dump the configuration into log-directory
dump_yaml(os.path.join(log_dir, "params", "env.yaml"), env_cfg)
dump_yaml(os.path.join(log_dir, "params", "agent.yaml"), agent_cfg)
dump_pickle(os.path.join(log_dir, "params", "env.pkl"), env_cfg)
dump_pickle(os.path.join(log_dir, "params", "agent.pkl"), agent_cfg)
# env_cfg, agent_cfg를 YAML + Pickle로 로그 디렉토리에 저장
	# 실험 재현성 확보가 목적

# run training
runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)
# 호출 시 설정된 반복(max_iterations) 횟수만큼 데이터 수집 ↔ 정책 업데이트(Policy Update) 루프를 반복
	# 정책 네트워크 파라미터가 점점 개선되고, 콘솔/텐서보드 로그에 성능 곡선, 보상, 손실 값이 기록됨
# close the simulator
env.close()
# 시뮬레이터 프로세스 종료, GPU·메모리 리소스 정리