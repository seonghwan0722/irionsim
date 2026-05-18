from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False}) # GUI 모드로 Isaac Sim 실행

# World 객체 생성 → 물리 공간 구성
assets_root_path = get_assets_root_path()
open_stage(assets_root_path + "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd")
my_world = World(stage_units_in_meters=1.0)

from omni.isaac.core.objects import DynamicCuboid
import omni.isaac.core.utils.numpy.rotations as rot_utils
import numpy as np

cube = my_world.scene.add(
    DynamicCuboid(
        prim_path="/World/cube",
        name="cube",
        position=np.array([0.0, 0.0, 1.0]), # 지면 위 1.0m에 배치
        orientation=rot_utils.euler_angles_to_quats(np.array([0, 0, 0]), degrees=True),# 회전 없음
        scale=np.array([0.5, 0.5, 0.5]), # 각 축의 크기를 0.5배로 조정 → 최종 크기 = $0.5\text{m}^3$
        size=1.0,
        color=np.array([0, 0, 255]), # 파란색 큐브
    )
)

import omni.kit.commands
from pxr import Gf
import omni.replicator.core as rep

lidar_config = "Example_Rotary"

_, sensor = omni.kit.commands.execute(
    "IsaacSensorCreateRtxLidar",
    path="/sensor", # 센서가 생성될 위치 (Prim 경로)
    parent=None,
    config=lidar_config, # Lidar 설정값 (회전식 예제 구성 사용) 
	    # 다양한 프리셋 가능: Example_Rotary, Example_FrontFacing 등
    translation=(1, 1, 0.5), # 센서 위치 (x, y, z)
    orientation=Gf.Quatd(1, 0, 0, 0), # 센서의 회전 방향 (단위 쿼터니언)
)

render_product = rep.create.render_product(sensor.GetPath(), [1, 1])
# 센서의 데이터 시각화를 위한 출력 포트(Viewport)를 생성
	# Para1: 센서의 위치 경로를 함수에 전달
	# Para2: 출력 해상도 설정

    annotator = rep.AnnotatorRegistry.get_annotator("RtxSensorCpuIsaacCreateRTXLidarScanBuffer")
# RTX Lidar로부터 스캔 데이터 추출/ CPU 메모리에 프레임 단위 버퍼로 저장
annotator.attach(render_product)

writer = rep.writers.get("RtxLidarDebugDrawPointCloudBuffer")
# 버퍼 데이터를 포인트 클라우드로 시각화/ 3D 점군 형태로 실시간 렌더링 가능
writer.attach(render_product)

while simulation_app.is_running(): # 시뮬레이션이 실행되고 있는 동안 반복하는 루프(시뮬레이션이 종료 될 때까지 실행)
    simulation_app.update() # 현재 프레임을 한 번 갱신(물리 연산, 센서 업데이트, 객체 상태 변화 등이 모두 반영)
    data = annotator.get_data() # 어노테이터로부터 최신 센서 데이터를 습득(Lidar나 Radar 등에서 현재 시점의 정보를 추출)
    print(data['distance'])# 수집된 데이터 중 'distance' 필드, 즉 센서와 각 포인트 사이의 거리 값들을 출력

simulation_app.close() # 루프 종료 후 호출 -> 자원을 정리하고 메모리 해제 포