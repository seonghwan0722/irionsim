# Isaac Lab Training Module

Spot 로봇의 보행 및 작업 수행을 위한 강화 학습(Reinforcement Learning) 환경입니다. Isaac Lab 프레임워크를 활용하여 고속의 병렬 시뮬레이션 학습을 지원합니다.

## 🛠 학습 파이프라인

1. **AppLauncher**: Isaac Sim 엔진 구동 및 Gym/Torch 환경 초기화.
2. **Hydra Config**: 환경 및 에이전트 파라미터를 동적으로 로드 및 관리.
3. **Environment Creation**: `gym.make`를 통해 Spot 로봇과 지형이 포함된 학습 환경 생성.
4. **Learning Loop**: PPO(Proximal Policy Optimization) 알고리즘을 통한 가중치 최적화.

## 📝 주요 구성 요소

### 1. 환경 설정 (`spot_flat_env_cfg.py`)
- **Observation**: 몸체 속도, 자세, 관절 상태, 이전 액션 등을 포함하는 관측 벡터 정의.
- **Commands**: 목표 선속도 및 각속도 범위 설정.
- **Rewards**: 
  - `track_lin_vel_xy_exp`: 목표 속도 추종 보상
  - `track_ang_vel_z_exp`: 목표 회전 속도 추종 보상
  - `action_rate_l2`: 급격한 동작 변화 억제 페널티
  - `feet_air_time`: 안정적인 보행 패턴 유도

### 2. 학습 실행 (`train.py`)
- 다중 에이전트(Multi-environment) 병렬 학습 지원.
- Tensorboard/WandB 로그 출력 지원.
- 학습 완료 후 최적 모델(`model.pt`) 자동 저장.

## 🚀 학습 실행 방법

```bash
# 헤드리스 모드로 학습 시작
python3 isaaclab/train.py --headless --num_envs 4096
```

## 📈 학습 모니터링
학습 중 생성되는 `logs/` 폴더 내의 데이터를 Tensorboard를 통해 확인할 수 있습니다.
```bash
tensorboard --logdir logs/
```
