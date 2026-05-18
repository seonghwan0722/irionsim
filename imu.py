from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})
# Isaac Sim을 GUI 모드로 실행, headless=True일 경우, 백그라운드에서만 실행됨

from omni.isaac.core import World
my_world = World(stage_units_in_meters=1.0) # 시뮬레이션에서 물리 엔진이 적용되는 공간 생성, 1단위 = 1미터

my_world.scene.add_default_ground_plane() # 기본 지면(Ground Plane) 추가, 동적 물체가 중력 및 충돌에 반응할 수 있도록 설정
my_world.reset() # 물리 엔진 초기화

from omni.isaac.core.objects import DynamicCuboid
import omni.isaac.core.utils.numpy.rotations as rot_utils
import numpy as np

cube = my_world.scene.add(
    DynamicCuboid( # 중력, 충돌 등 물리 엔진 적용 가능한 동적 직육면체 생성
        prim_path="/World/cube", # /World 경로 아래에 cube라는 이름으로 객체 생성
        name="cube",
        position=np.array([0.0, 0.0, 1.0]), # 지면 위 1m에 큐브 배치
        orientation=rot_utils.euler_angles_to_quats(np.array([0, 0, 0]), degrees=True),  # 회전 없음, 초기 상태
        scale=np.array([0.5, 0.5, 0.5]), # 상대 크기 조절값 (x, y, z 방향 각각 0.5배)
        size=1.0, # 기본 단위 크기 (1.0m 기준)
        color=np.array([255, 0, 0]),# RGB 기준 빨간색 큐브
    )
)

from omni.isaac.sensor import IMUSensor

sensor = my_world.scene.add(IMUSensor(
    prim_path="/World/cube/Imu", # 센서를 /World/cube 아래에 생성하여 큐브에 IMU 센서 부착
    name="imu", # 센서 객체의 참조 이름
    frequency=60, # 60Hz 주기로 센서 데이터 수집
    translation=np.array([0, 0, 0.5]), # 큐브 중심 기준 z축으로 0.5m 위에 센서 위치
    orientation=np.array([1, 0, 0, 0]), # 기본 회전 상태 (쿼터니언)
    linear_acceleration_filter_size=10, # 선형 가속도 필터 (노이즈 제거)
    angular_velocity_filter_size=10, # 각속도 필터 (회전 데이터 부드럽게 처리)
    orientation_filter_size=10, # 자세 필터(회전 방향 안정화)
))

my_world.reset()

window = ui.Window("IMU HUD", width=300, height=100) # 화면 상단에 IMU HUD 창 생성
with window.frame:
    with ui.VStack():
        imu_label = ui.Label("Waiting for IMU data...") # 실시간 IMU 가속도 텍스트 표시

while simulation_app.is_running():
    my_world.step(render=True) # 물리연산 + 렌더링 한 프레임씩 실행
    imu_data = sensor.get_current_frame(read_gravity=True)["lin_acc"] # 센서의 선형 가속도 실시간 추출
    imu_label.text = f"IMU Accel: {np.round(imu_data, 3)}" # 소수점 3째 자리까지 반올

simulation_app.close() # 시뮬레이션 종료 및 리소스 반환