# ATS (Autonomous Transportation System) Control Module

ATS 모듈은 Isaac Sim 환경에서 로봇의 실시간 제어, 시각화 및 ROS2 연동을 담당하는 핵심 런타임 엔진입니다.

## ✨ 주요 기능

- **실시간 정책 추론**: PyTorch(TorchScript) 기반으로 학습된 보행 모델을 실시간으로 실행.
- **ROS2 통합**: OmniGraph를 통해 LiDAR, Camera, IMU, Odometry 데이터를 ROS2 토픽으로 송출.
- **통합 제어 시스템**: 키보드 텔레오퍼레이션과 자율 주행 명령을 합성하여 로봇 구동.
- **안정적인 시뮬레이션**: 고정 타임스텝(Fixed Time Step) 및 타임라인 동기화를 통한 센서 데이터 신뢰성 확보.

## 🏗 시스템 아키텍처

1. **SimWorld (`world.py`)**: USD 스테이지 로드, 아티큘레이션 뷰(ArticulationView) 관리, 시뮬레이션 타임라인 설정.
2. **GraphBuilder (`graph_builder.py`)**: ROS2 통신용 OmniGraph 자동 생성 (Lidar, Camera, TF, Odom).
3. **ObservationBuilder (`observation.py`)**: 로봇의 상태 정보를 48차원 관측 벡터로 가공하여 모델 입력 준비.
4. **PolicyRunner (`policy.py`)**: 학습된 `.pt` 모델 로드 및 추론 수행.
5. **RobotController (`controller.py`)**: 추론된 액션 값을 로봇 관절에 적용하고 상위 명령 전달.

## 🚀 실행 방법

```bash
# Isaac Sim 환경에서 메인 제어 루프 실행
python3 ats/main.py
```

## 📡 ROS2 인터페이스

- **Subscriptions**:
  - `/cmd_vel`: 자율 주행 이동 명령 수신
- **Publications**:
  - `/scan`: 2D LiDAR 데이터
  - `/point_cloud`: 3D LiDAR 포인트 클라우드
  - `/odom`: 로봇 주행 오도메트리
  - `/tf`: 좌표계 변환 정보
  - `/camera/image_raw`: 카메라 영상 데이터

## ⚙️ 설정 (Configuration)

설정은 `config/default.yaml` 또는 환경 변수 `ATS_CONFIG`를 통해 관리됩니다.
- `assets`: USD 파일 경로 및 Prim 경로 설정
- `policy`: 모델 경로 및 실행 디바이스 설정
- `player`: 시뮬레이션 주파수(Hz) 및 타임스텝 설정
