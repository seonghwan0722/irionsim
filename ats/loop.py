# 실행 루프

class SimLoop: # 프레임 당 액션을 정의 하고, 순서를 정리하는 스케줄러 역할
    def __init__(self, sim_app, world, input_dev, policy, obs_builder, controller, speeds):
        self.app = sim_app
        self.world = world
        self.input = input_dev
        self.policy = policy
        self.obsb = obs_builder
        self.ctrl = controller
        self.speeds = speeds

    def run(self):
        import numpy as np
        while self.app.raw.is_running(): # 실행되고 하위 코드 반복 유지
            self.world.step(render=True) # 물리/센서/렌더 한 프레임 진행
            # => 물리 시뮬레이트 업데이트 및 센서(카메라/라이나)와 렌더 타깃(RenderProduct) 최신 상태 반영
            # => `render=True`는 뷰포트 갱신까지 포함 → 안정적인 센서 샘플링과 제어 적용 사이 순서 고정
            teleop_vec, ats_cmd = self.ctrl.teleop_cmd(self.input.pressed, self.speeds) # 키보드(방향키/WASD 등)로 만든 선속 벡터/ ATS(짐벌)용 증분 명령(예: yaw, pitch)
            vx_r, vy_r, vz_r = self.ctrl.read_twist_from_graph() # OmniGraph의 md_vel 구독 노드에서 읽은 자율 주행 Twist
            cmd_vec = np.array([teleop_vec[0] + vx_r, teleop_vec[1] + vy_r, teleop_vec[2] + vz_r], dtype=np.float32) # 텔레옵 + 자율 명령 합성
            # 최종 cmd_vec은 사람(teleop)+ 자율(twist) 여기에 가중치로 블렌딩 (예: α*자율 + (1-α)*텔레옵)을 권장

            obs = self.obsb.build(cmd_vec) # 현재 프레임 상태를 48차원의 관측 벡터로 구성 역할
            # 몸체 기준 선속도/각속도, 중력방향, 명령 벡터, 관절 상태, 직전 액션
            # 좌표계 변환(월드->바디) 및 기본자세 기준 편차 반영 정책
            action = self.policy.infer(obs) # TorchScript 정책으로부터 12차원 액션 획득 역할 [모델 입력 시 (1,48) 배치 차원 사용, 출력 시 squeeze()로 (12,) 변환, 결과는 넘파이 배열로 반환하여 후속 제어에 바로 사용]
            self.ctrl.apply_actions(action, ats_cmd) # spot, ats 아티큘레이션 갱신 역할 
            # SPOT(12DOF) 각도 설정식 q_target = default_pos + scale * action
            # ATS(2DOF) 증분 적용식: q += step * ats_cmd (프레임 시간 변동 시 step*= dt로 회전 속도 보정)
            
            self.obsb.update_prev_action(action) # 직전 액션 저장(다음 관측 벡터에 포함할 prev_action 갱신 (보행 위상 안정성 유지))
            self.ctrl.trigger_graph() # ATS 그래프 임펄스 트리거 (-> ATS  그래프 초기화 시 1 프레임 임펄스 신호 전달, 매 프레임 호출이 아니라 필요 시점에만 호출)