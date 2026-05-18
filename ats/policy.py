# 학습 모델 로드, 추론

class PolicyRunner: # 학습된 TorchScript 정책 모델을 로딩하고, 관측 벡터를 넣어 12차원 액션을 추론하는 전용 실행기
    def __init__(self, path: str, device: str = "auto"):
        if device == "auto": # GPU 연결 가능 여부에 따라 cuda:0 또는 cpu로 디바이스를 설정
            device = "cuda:0" if torch.cuda.is_available() else "cpu"
        self.device = torch.device(device)
        self.model = torch.jit.load(path, map_location=self.device).eval() # 모델을 추론 전용 모드로 고정하며, 이 과정에서 Dropout이나 BatchNorm 등이 비활성화

    def infer(self, obs_np): # 입력받은 관측(obs) 데이터를 (1, 48) 텐서 형태로 변환하여 모델에 입력
        with torch.no_grad(): # 그래디언트 트래킹을 비활성화하여 메모리 오버헤드를 줄임
            t = torch.tensor(obs_np, dtype=torch.float32, device=self.device).unsqueeze(0) 
            out = self.model(t).squeeze(0).cpu().numpy() # 모델 출력값 (1, 12)를 squeeze를 통해 (12,) 형태로 변환
        return out # 정책의 원시 액션($-1 \sim 1$ 범위)을 반환합니다. 반환된 값은 이후 기본자세와 스케일링이 적용되어 물리적으로 유효한 각도로 변환
    
    # eval() / no_grad(): 워밍업 실행 (1~2회 실행을 통해 JIT 내부 그래프 및 캐시를 예열하여 초반 지연 방지 입력을 도모)
    # dtype= float32: GPU와 CPU 간의 공통 호환성을 확보하고 연산 속도를 최적으로 유지
    # torch.set_float32_matmul_precision("high"): GPU 연산 시 정밀도와 속도 사이의 균형을 조정
    # 추론 시간이 주기 체크 (관측 생성/복사 비용도 함께 고려)
    # 멀티스레드 환경에서 일관성 보장 (prev_action업데이트가 관측빌드와 엇갈리지 않게 락(lock) 또는 상태 스냅샷 복사)
