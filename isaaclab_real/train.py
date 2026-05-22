# isaac lab을 이용한 강화학습

import argparse
import os
import sys
from datetime import AppLauncher  # Isaac Sim/Lab 초기화 (torch 임포트보다 먼저 수행되어야 함)

# 1. CLI 인자 설정
# add argparse arguments # --task, --num_envs, --video 등 사용자 옵션 정의
parser = argparse.ArgumentParser(description="Train an RL agent with RSL-RL.")
parser.add_argument("--video", action="store_true", default=False, help="Record videos during training.")
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument("--agent", type=str, default="ppo", help="Name of the agent configuration.")
parser.add_argument("--max_iterations", type=int, default=None, help="Maximum iterations to train.")
parser.add_argument("--device", type=str, default="cuda:0", help="Device to use for training.")
parser.add_argument("--seed", type=int, default=None, help="Seed for the environment and the agent.")


# AppLauncher 전용 인자 추가(--headless 등)
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# --- 1. 시뮬레이션 구동 ----
import gymnasium as gym
import os
import torch

import config
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlVecEnvWrapper
from isaaclab_rl.utils.hydra import hydra_task_config
import isaaclab_rl.utils.cli_args as cli_args
from rsl_rl.runners import OnPolicyRunner
from isaaclab.utils.dict import dump_pickle, dump_yaml
from isaaclab.envs import ManagerBasedRLEnvCfg, DirectRLEnvCfg, DirectMARLEnvCfg

# ---2. 설정의 확정 ---
@hydra_task_config(args_cli.task, args_cli.agent)
# task 이름의 라이브러리 안에서 해당하는 환경 설정, 에이전트 설정 파일을 자동으로 찾아서 로드
def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg, agent_cfg: RslRlOnPolicyRunnerCfg):

	"""Train with RSL-RL agent."""
	 # 사용자의 명령이 파일 설정보다 우선하는 원칙 적용
	agent_cfg = cli_args.update_rsl_rl_cfg(agent_cfg, args_cli)
	env_cfg.scene.num_envs = args_cli.num_envs if args_cli.num_envs is not None else env_cfg.scene.num_envs
	agent_cfg.max_iterations = args_cli.max_iterations if args_cli.max_iterations is not None else agent_cfg.max_iterations
	env_cfg.seed = agent_cfg.seed # 환경과 에이전트가 같은 시드를 쓰도록 맞춰 재현성 확보
	env_cfg.sim.device = args_cli.device if args_cli.device is not None else env_cfg.sim.device

# ------3. 환경생성과 포장 -----
	# 로그 디렉토리 생성
	log_dir = os.path.join("logs", "rsl_rl", args_cli.task, datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))
	# 환경 생성
	env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)

	# 비디오 녹화 설정
	if args_cli.video:
		video_kwargs = {
			"video_folder": os.path.join(log_dir, "videos"),
			"step_trigger": lambda step: step % 1000 == 0,
			"video_length": 200,
		}
		env = gym.wrappers.RecordVideo(env, **video_kwargs)
		# RecordVideo : --video 옵션이 켜져 있으면 래퍼(RecordVideo) 한 겹 더 씌움

	# rsl-rl 래퍼 적용
	env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
	# RslRlVecEnvWrapper : Isaac Lab(환경)과 RSL-RL(알고리즘)이 쓰는 데이터 형식이 다르기에 중간 '통역사' 역할

# -----4. 학습 시작 -----
	# Runner 생성 및 학습
	runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=log_dir, device=agent_cfg.device)

	# 설정값 덤프 (재현성 확인용)
	os.makedirs(os.path.join(log_dir, "params"), exist_ok=True)
	dump_yaml(os.path.join(log_dir, "params", "env.yaml"), env_cfg)
	dump_yaml(os.path.join(log_dir, "params", "agent.yaml"), agent_cfg)
	dump_pickle(os.path.join(log_dir, "params", "env.pkl"), env_cfg)
	dump_pickle(os.path.join(log_dir, "params", "agent.pkl"), agent_cfg)

	runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)
	# 호출 시 설정된 반복(max_iterations) 횟수만큼 데이터 수집 ↔ 정책 업데이트(Policy Update) 루프를 반복
		# 정책 네트워크 파라미터가 점점 개선되고, 콘솔/텐서보드 로그에 성능 곡선, 보상, 손실 값이 기록됨

	env.close()