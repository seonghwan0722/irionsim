from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False}) # GUI 모드로 Isaac Sim 실행

# World 객체 생성 → 물리 공간 구성
assets_root_path = get_assets_root_path()
open_stage(assets_root_path + "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd")
my_world = World(stage_units_in_meters=1.0)

radar_config = "Example"

_, sensor = omni.kit.commands.execute( # Isaac Sim 명령어 시스템을 통해 센서 생성 명령을 실행
    "IsaacSensorCreateRtxRadar",
    path="/sensor",
    parent=None,
    config=radar_config, # Example_Rotary, Example_FrontFacing 등 다양한 프리셋 설정이 가능
    translation=(0.0, 0, 0.25), # 센서 위치(x,y,z)
    orientation=Gf.Quatd(0.707, 0, 0, -0.707), # 센서의 회전 방향 (쿼터니언)
)

# ---- Render Product 생성 ----
render_product = rep.create.render_product(sensor.GetPath(), resolution=(512, 512))
# 센서 데이터 렌더링 결과를 출력할 뷰포트를 정의
# 시뮬레이션 내 센서 감지 결과를 화면에 표시하기 위한 필수 과정입니다.
#  해상도는 최소 **512x512** 이상을 권장하며, 이는 시각화 품질에 직접적인 영향을 미칩니다.

# ----Annotator 등록 ----
annotator = rep.AnnotatorRegistry.get_annotator("RtxSensorCpuIsaacComputeRTXRadarPointCloud")
annotator.attach(render_product) 
# 센서 데이터를 특정 포맷으로 변환해주는 어노테이터를 설정
# 예시: `RtxSensorCpuIsaacComputeRTXRadarPointCloud`는 수신된 Radar 신호를 포인트 클라우드 형태로 변환하는 역할을 합니다.

# ----Writer 등록----
    writer = rep.writers.get("RtxRadarDebugDrawPointCloud")
    writer.attach([render_product])
# 수집 및 가공된 데이터를 화면에 시각적으로 최종 표시
# 예시: RtxRadarDebugDrawPointCloud→ Radar 데이터
  # → 포인트 클라우드 형태로 렌더링 → 방향 벡터, 속도 정보 등도 함께 시각화 가능
while simulation_app.is_running(): # 시뮬레이션이 실행되고 있는 동안 반복하는 루프(시뮬레이션이 종료 될 때까지 실행)
    simulation_app.update() # 현재 프레임을 한 번 갱신(물리 연산, 센서 업데이트, 객체 상태 변화 등이 모두 반영)
    data = annotator.get_data() # 어노테이터로부터 최신 센서 데이터를 습득(Lidar나 Radar 등에서 현재 시점의 정보를 추출)
    print(data['distance'])# 수집된 데이터 중 'distance' 필드, 즉 센서와 각 포인트 사이의 거리 값들을 출력

simulation_app.close() # 루프 종료 후 호출 -> 자원을 정리하고 메모리 해제 포