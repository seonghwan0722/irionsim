# 환경 초기화: USD 스테이지 로드, Spot·ATS 초기화, 시뮬레이션 환경(물리·시간 스텝) 설정
from omni.isaac.core.utils.stage import open_stage
from omni.isaac.core import World
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils.prims import define_prim
from pxr import UsdGeom, Sdf

# app/world.py 등 공용 유틸에 넣어두면 좋음
import omni.usd, omni.kit.app, omni.kit.commands as kitcmd
from pxr import Gf, Sdf

class SimWorld:
    def __init__(self, usd_path: str, spot_prim: str, ats_prim: str, imu_dummy_prim: str,
                 fixed_time_step: bool, play_every_frame: bool, target_hz: int,
                 lidar_cfg: dict | None = None):

        import omni.usd
        import omni.timeline
        import carb.settings

        open_stage(usd_path) # 스테이지(USD파일)을 오픈
        self.stage = omni.usd.get_context().get_stage() # 로봇을 물리 시뮬레이션의 아티큘레이션(Articulation)으로 연결

        def _find_articulation_root(stage, base_path: str) -> str | None: # 아티큘레이션 루프 트림 탐색기
            from pxr import Sdf, Usd, UsdPhysics
            base = stage.GetPrimAtPath(base_path)
            if not base.IsValid():
                return None

            if UsdPhysics.ArticulationRootAPI.CanApply(base):
                if UsdPhysics.ArticulationRootAPI(base):
                    return base.GetPath().pathString

            it = Usd.PrimRange(base) # 경로의 하위 경로들을 순회하며 유효한 루트 탐색
            for prim in it:
                if prim.IsValid() and UsdPhysics.ArticulationRootAPI(prim):
                    return prim.GetPath().pathString
            return None



            settings = carb.settings.acquire_settings_interface()
            timeline = omni.timeline.get_timeline_interface() # 시간 및 타임라인 고정
            if fixed_time_step: # 목표 주파수를 설정하여 ROS2와 시간축을 통일
                settings.set("/app/player/useFixedTimeStepping", True)
            timeline.set_play_every_frame(play_every_frame)
            settings.set("/app/player/targetRunLoopFrequency", int(target_hz))
            timeline.play()

            define_prim("/World/odom", "Xform") # odom과 map을 구분하기 위해 스테이지 상에 Xform으로 고정점 생성

            spot_root = _find_articulation_root(self.stage, spot_prim)
            ats_root = _find_articulation_root(self.stage, ats_prim)

            self.world = World()
            self.world.reset()

            if not spot_root:
                raise RuntimeError(f"[SimWorld] Could not find ArticulationRoot under '{spot_prim}'")
            self.spot = ArticulationView(prim_paths_expr=spot_root, name="spot_view") # 관절 상태 조회 및 목표치 설정의 통로 역할
            self.world.scene.add(self.spot) # 
            self.world.play()

            self.ats = None
            # => self.spot / self.ats 활용: 서로 다른 제어 모드 적용이 용이하며, 카메라 TF 구조도 명확하게 유지
            if ats_root:
                self.ats = ArticulationView(prim_paths_expr=ats_root, name="ats_view")
                self.world.scene.add(self.ats)

            def step(self, render=True): # 한 줄의 코드로 물리 연산과 렌더링을 동시에 진행
                self.world.step(render=render)