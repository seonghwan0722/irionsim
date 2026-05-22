# Isaac Sim Robotics Project

이 프로젝트는 NVIDIA Isaac Sim을 기반으로 한 로봇 제어, 시뮬레이션 및 강화 학습 통합 환경입니다. Spot 로봇과 ATS(Autonomous Transportation System)를 활용한 자율 주행 및 보행 제어 솔루션을 포함하고 있습니다.

## 📂 프로젝트 구조

```text
.
├── ats/                # 자율 제어 및 시뮬레이션 통합 모듈 (Runtime Control)
│   ├── main.py         # 시스템 메인 실행 스크립트
│   ├── controller.py   # 로봇 및 장치 제어 로직
│   ├── world.py        # 시뮬레이션 환경 및 USD 관리
│   ├── graph_builder.py # ROS2 통신을 위한 OmniGraph 구성
│   └── ...
├── isaaclab/           # Isaac Lab 기반 강화 학습 모듈 (RL Training)
│   ├── train.py        # RL 모델 학습 스크립트
│   ├── spot_flat_env_cfg.py # Spot 로봇 학습 환경 설정
│   └── ...
├── imu.py              # IMU 센서 유틸리티
├── lidar.py            # LiDAR 센서 유틸리티
└── radar.py            # Radar 센서 유틸리티
```

## 🚀 주요 모듈 상세

### 1. ATS (Autonomous Transportation System)
Isaac Sim 실시간 런타임에서 로봇을 제어하고 센서 데이터를 ROS2로 송출하는 통합 제어 모듈입니다. 학습된 정책(Policy) 모델을 사용하여 실시간 추론 및 보행 제어를 수행합니다.

### 2. Isaac Lab
강화 학습을 통해 로봇의 보행 정책을 학습시키는 환경입니다. 도메인 랜덤화(Domain Randomization)와 다양한 보상 체계(Reward Function)를 통해 강인한 제어 모델을 생성합니다.

## 🛠 시작하기

### 권장 환경
- **OS**: Ubuntu 22.04
- **Simulator**: NVIDIA Isaac Sim 2023.1.1+
- **Framework**: ROS2 Humble, PyTorch, Isaac Lab

### 설치 및 실행
각 하위 폴더의 `README.md` 파일을 참조하십시오.
- [ATS 실행 방법](./ats/README.md)
- [Isaac Lab 학습 방법](./isaaclab/README.md)
