# 정책 출력 -> 관절 명령 변환
class RobotController: # 정책의 원시 액션 + 사용자 명령을 실제 관절 목표로 변환하여 시뮬레이터에 적용
    def __init__(self, spot_view, ats_view, spot_action_scale: float, ats_joint_step: float,
                 debug: bool = True, log_every_n: int = 120, dump_dir: str | None = None,
                 is_lock_assume_last_two_ats: bool = True):

        self.spot = spot_view # 12DOF 정책 출력(12차원)을 기본자세(default_pos) 기준으로 목표 관절각으로 변환
        self.ats = ats_view   # 2DOF 증분 명령(yaw/pitch)을 이용해 현재 각도를 한 스텝씩 변화   
        self.spot_scale = float(spot_action_scale) # 정책 출력(-1 ~ 1)을 실제 관절각(라디안)으로 변환하는 스케일
        self.ats_step = float(ats_joint_step) # ATS 회전을 증분 제어로 움직일 때의 한 프레임당 변화량
        self.default_pos = self.spot.get_joint_positions()[0].copy()

        # 디버그/로그
        self.DEBUG = bool(debug)
        self._log_every_n = int(log_every_n)
        self._apply_count = 0
        if dump_dir is None:
            dump_dir = Path(__file__).resolve().parents[1] / "logs"
        self.dump_dir = Path(dump_dir)
        self.dump_dir.mkdir(parents=True, exist_ok=True)

        # Spot/ATS DOF # DOF 자동 검출 기능을 수행 => DOF 자동 검출 결과를 각 변수에 저장하여 스테이지 변경에도 안전하게 대응
        try:
            self._spot_dof = int(len(self.spot.get_joint_positions()[0]))
        except Exception:
            self._spot_dof = 0
        try:
            self._ats_dof = int(len(self.ats.get_joint_positions()[0])) if self.ats else 0
        except Exception:
            self._ats_dof = 0

        # 제어 경로: 현 구조상 ATS는 별도 articulation으로 제어(고려)
        self._control_ats_via = "ats" if self.ats and self._ats_dof > 0 else None
        self._ats_ids_spot = [] # (...)=ats면 ats를 별도 아티큘레이션으로 제어/ (...) != ats 면 spot을 별도 아티큘레이션으로 제어
        self._ats_ids_ats = [0, 1] if self._ats_dof >= 2 else ([0] if self._ats_dof == 1 else [])
# ArticulationView를 통해 각 로봇의 현재 관절 상태에 접근
# default_pos는 정책 출력의 기준점으로 사용되며, 절대 위치 예측보다 안정적인 제어가 가능



        # spot 다리 인덱스 (앞 12개) --
        self._leg_idx = list(range(min(12, self._spot_dof)))

        if self.DEBUG:
            pring(f"[ATS DEBUG] control path = {self._control_ats_via}")
            pring(f"[ATS DEBUG] leg_idx={self._leg_idx}")
            pring(f"[ATS DEBUG] ats_idx_spot={self._ats_idx_spot}, ats_idx_ats={self._ats_idx_ats}")

    def read_twist_from_graph(): # 옴니그래프의 SubscribeTwist 출력에서 선속 및 각속 명령을 직접 로드
        ### [ADVICE] "/ATSActionGraph/SubscribeTwist"와 같은 경로는 cfg["ros"]["graph_path"] 등을 활용해 
        ### 외부 설정 파일에서 관리하도록 변경하면, 그래프 구조 변경 시 코드 수정을 최소화할 수 있습니다.
        lin = og.Controller.get(og.Controller.attribute("/ATSActionGraph/SubscribeTwist.outputs:linearVelocity"))
        ang = og.Controller.get(og.Controller.attribute("/ATSActionGraph/SubscribeTwist.outputs:angularVelocity"))
        # "/ATSActionGraph/SubscribeTwist.outputs:*"의 경로가 변경될 경우 해당 상수도 함께 변경
        if lin is None or ang is None:
            return 0.0, 0.0, 0.0
            
        vx, vy, _ = lin
        _, _, az = ang
        return float(vx), float(vy), float(az)

    def teleop_from_keys(pressed, lin_speed, yaw_speed, pitch_speed): # 키보드 입력(WASD/화살표)를 로봇의 선속 및 짐벌 회전 명령으로 변환
        vx = lin_speed if 'up' in pressed else (-lin_speed if 'down' in pressed else 0.0)
        vz = lin_speed if 'left' in pressed else (-lin_speed if 'right' in pressed else 0.0)
        qz = yaw_speed if 'a' in pressed else (-yaw_speed if 'd' in pressed else 0.0)
        qy = pitch_speed if 'w' in pressed else (-pitch_speed if 's' in pressed else 0.0)
        # => 반환값에 ATS 증분 명령을 더하여 로봇의 움직임을 미세하게 조정 가능
        return np.array([vx, 0.0, vz], dtype=np.float32), np.array([qz, qy], dtype=np.float32)

    def apply_actions(self, policy_action, ats_cmd):
        # -- Spot 다리 12축 제어 --
        q_spot = self.spot.get_joint_positions()[0].copy() # 현재 관절각 리드
        
        # policy_action 길이 방어: 부족하면 0으로 채움, 길면 12개만 사용 # 정책 출력 길이 보정, 12축에 맞게 패딩/ 슬라이싱
        pa = np.asarray(policy_action, dtype=np.float32).ravel()
        if len(self._leg_idx) > 0:
            if pa.size < len(self._leg_idx):
                pa = np.pad(pa, (0, len(self._leg_idx) - pa.size), mode="constant")
            pa_leg = pa[:len(self._leg_idx)]
            q_spot[self._leg_idx] = self.default_pos[self._leg_idx] + pa_leg * self.spot_scale # 계산 목표각 공식
            # 계산 목표각 = 기본자세 + 스케일 × 정책 => 결과를 Spot의 앞 12개 다리 조인트에 반영
        self.spot.set_joint_position_targets(q_spot) # 계산 목표각을 spot에 적용 -> PD/임피던스 컨트롤러가 목표각을 향해 움직임
        # 여기서 클램프(관절 리미트)를 한번 더 적용 [예) q_spot = np.clip(q_spot, q_min, q_max)]
        
        # -- ATS 2축 제어 (별도 articulation) --
        if self._control_ats_via == "ats" and self._ats_idx_ats: 
            q_ats = self.ats.get_joint_positions()[0].copy() #  현재 ATS 조인트각 리드
            cmd = np.asarray(ats_cmd, dtype=np.float32).ravel() # ats_cmd = [yaw, pitch] : 명령 벡터
            
            # ats_cmd 길이 방어: [qz, qy] 2축 기준, 부족시 패딩
            if cmd.size < len(self._ats_idx_ats):
                cmd = np.pad(cmd, (0, len(self._ats_idx_ats) - cmd.size), mode="constant")
                
            # 적용
            for i, idx in enumerate(self._ats_idx_ats):
                q_ats[idx] += self.ats_step * cmd[i] # 각 축에 증분 적용 (매틱 마다 조금씩 회전(조준제어에 적합))
            self.ats.set_joint_position_targets(q_ats) # 계산된 목표각을 ATS에 반영

            if self._apply_count % self._log_every_n == 0 and self.DEBUG:
                print(f"[ATS DEBUG] apply: path=ats, cmd={cmd[:len(self._ats_idx_ats)]}, ids={self._ats_idx_ats}")
        
        
        elif self._control_ats_via == "spot" and self._ats_idx_spot:
            # (현 구조에서는 사용 안 함)
            q = self.spot.get_joint_positions()[0].copy()
            cmd = np.asarray(ats_cmd, dtype=np.float32).ravel()
            if cmd.size < len(self._ats_idx_spot):
                cmd = np.pad(cmd, (0, len(self._ats_idx_spot) - cmd.size), mode="constant")
            for i, idx in enumerate(self._ats_idx_spot):
                q_ats[idx] += self.ats_step * cmd[i] 
            self.spot.set_joint_position_targets(q)
            if self._apply_count % self._log_every_n == 0 and self.DEBUG:
                print(f"[ATS DEBUG] apply: path=spot, cmd={cmd[:len(self._ats_idx_spot)]}, idx={self._ats_idx_spot}")

        else:
            if self._apply_count % self._log_every_n == 0 and self.DEBUG:
                print(f"[ATS DEBUG] apply: path=None, cmd={ats_cmd}")

        self._apply_count += 1

        @staticmethod
        def trigger_graph_impulse():
            og.Controller.set(og.Controller.attribute("/ATSActionGraph/OnImpulseEvent.state:enableImpulse"), True)
