# 시뮬레이션 실행, 그래프 구성, 로봇 제어 등 다양한 기능을 포함하는 스크립트

# -----1. config로딩과 시뮬레이션 asset 설정
from app.utils import load_cfg
cfg = load_cfg() # -> YAML을 포함하여 각종 설정 파일을 다음 우선순위에 맞게 탐색
# (1) 직접 전달한 인자 (2)환경변수 ATS_CONFIG (3) 프로젝트루트의 config/default.yaml
# assets:
#   usd_path: "../ATS_Enviroment/Spot_Ats2_with_Environment.usd" 로봇과 환경 정보가 포함된 USD 파일 경로
#   spot_prim: "/World/Spot" 해당 USD에서 로봇이 위치한 Prim 경로

# -----2. Policy 파일 로딩 -------
policy = PolicyRunner(cfg["policy"]["path"], cfg["policy"]["device"])
            # -> 학습된 정책 모델을 로딩 수행
# policy:
#   path: "../ATS_Enviroment/spot_policy.pt" 학습이 완료된 정책 모델 파일(`.pt`)의 경로를 지정
#   device: "auto"  모델을 실행할 장치(CPU 또는 GPU)를 자동으로 선택하도록 설정하는 옵션

# -------3. 관측 벡터 구성 -------
default_pos = world.spot.get_joint_positions().squeeze(0) * 1.0
                # -> Spot의 초기 관절값 획득
obsb = ObservationBuilder(world.spot, default_pos)
obs = obsb.build(cmd_vec) # 48차원의 관측 벡터 생성

# -------4. Policy 추론과 행동 적용 -----
action = policy.infer(obs) # 48차원의 관측 벡터를 정책에 전달

def infer(self, obs_np): # 관측 벡터 정보를 정책에 전달하기 위한 함수
    with torch.no_grad(): # PyTorch 텐서로 변환하여 적절한 디바이스 (CPU/GPU)로 이동
        t = torch.tensor(obs_np, dtype=torch.float32, device=self.device).unsqueeze(0)
        out = self.model(t).squeeze(0).cpu().numpy() # 정책 모델에 전달하고 행동(action)을 예측
    return out

ctrl.apply_actions(action, ats_cmd_from_keys) # 예측된 행동 값을 각 관절에 전달
obsb.update_prev_action(action) # 다음 스텝의 관측 생성을 위해 현재 수행한 행동을 저장
# 루프 구조: 관측 생성 → 정책 추론 → 행동 적용 → 시뮬레이션 업데이트 순서로 로봇이 동작