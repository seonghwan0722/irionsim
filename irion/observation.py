#  관측 벡터 구성 (관절 각/속도, 자세, 중력 등)

class ObservationBuilder: # Isaac Sim에서 얻은 로봇 상태를 몸체 좌표계 기준의 48차원 벡터로 정리
    def __init__(self, spot_view, default_pos):
        self.spot = spot_view
        self.prev_action = np.zeros(12, dtype=np.float32)
        self.default_pos = default_pos

    def build(self, cmd_vec):
        joint_pos = self.spot.get_joint_positions()[:, :12]
        joint_vel = self.spot.get_joint_velocities()[:, :12] # => Spot의 다리 관절 위치·속도를 12자유도까지만 사용
        lin_vel_I = self.spot.get_linear_velocities()[0]
        ang_vel_I = self.spot.get_angular_velocities()[0]
        _, quat_IB = self.spot.get_world_poses()
        R_IB = quat_to_rot_matrix(quat_IB[0]); R_BI = R_IB.T # Isaac 바디 프레임 좌표계와 정의 일치 (중력 벡터로 월드 z-down/y-down환경에 맞게 투영 필요)
        lin_vel_b = R_BI @ lin_vel_I
        ang_vel_b = R_BI @ ang_vel_I # => - 관성 좌표계 기준의 속도값
        # 자세에 따라 해당 값의 의미가 달라짐/ 쿼터니언 → 회전행렬 변환 → 몸체좌표계(R_BI)로 투영
        gravity_b = R_BI @ np.array([0.0, 0.0, -1.0], dtype=np.float32) # 중력 가속도 g = (0, 0, -1)에 몸체 측으로 투영한 값


        obs = np.zeros(48, dtype=np.float32) # 48차원의 관측 벡터
        # obs[0:12]: 상위 레벨에서 원하는 속도 벡터나 명령 벡터
        obs[0:3] = lin_vel_b  # obs[0:9]:  몸체 기준 선속·각속·중력 방향 벡터
        obs[3:6] = ang_vel_b
        obs[6:9] = gravity_b
        obs[9:12] = cmd_vec # 명령 벡터 cmd_vec
        # obs[12:48]: 관절의 위치 편차, 속도 벡트
        obs[12:24] = joint_pos - self.default_pos[:12] # 기본자세(default_pos)와 관절위치 차이 벡터 → 안정적인 학습
        obs[24:36] = joint_vel # 원래 관절 속도 값 벡터 → 보행 위상, 스윙/서포트 전환을 읽는 핵심 단서
        obs[36:48] = self.prev_action[:12] # 이전 액션을 저장하고, 보행 리듬을 외부 신호로 이어주는 역할 → 보행 안정성 향상
        return obs

    def update_prev_action(self, action):
        self.prev_action = action.copy()  #prev_action 갱신 필수: update_prev_action 호출
        # 갱신 안하면 0 벡터로 고정, 매 스텝 정책 출력 후 update_prev_action 반드시 호출