# 센서 & ROS2 브릿지
    # 센서 및 제어 신호를 ROS2 토픽으로 노출하는 파이프라인을 구축하는 모듈
    # OmniGraph(노드 기반 실행 그래프) 코드로 구성하여 카메라, IMU, Odom, TF, LiDAR 데이터가 안정적으로 발행
    # TF 트리, 명령 구독을 그래프 형태로 일괄 구성 

class GraphBuilder:
    def __init__(self, assets_cfg: dict, ros_cfg: dict):
        import omni.usd
        import omni.timeline

        self.assets = assets_cfg
        self.ros = ros_cfg

        # 타임라인 재생
        self.timeline = omni.timeline.get_timeline_interface()
        self.timeline.play()

        # 스테이지 핸들
        self.stage = omni.usd.get_context().get_stage()

        # 필요한 익스텐션 활성화
        for ext in [
            "omni.anim.people",
            "omni.anim.graph.bundle",
            "omni.kit.scripting",
            "omni.anim.graph.ui",
            "omni.anim.graph.schema",
            "omni.anim.navigation.schema",
            "omni.isaac.ros2_bridge",
            "omni.isaac.sensor", # RTX LiDAR
        ]

        graph, _, _, _ = og.Controller.edit(
            {
                "graph_path": graph_path,
                "evaluator_name": "push",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
            }, # => 매 프레임 자동 평가가 아닌, 필요 시 직접 실행 방식 (every_frame: 매 프레임 자동 평가)
            {
                keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"), # 그래프 실행을 위한 트리거(시작 신호) 역할을 수행 -> OnTick.outputs:tick 출력이 IsaacCreateViewport 노드로 연결되어 프로세스를 시작
                    ("createViewport", "isaacsim.core.nodes.IsaacCreateViewport"), # 렌더링에 필요한 논리적 뷰(뷰포트)를 생성
                    ("getRenderProduct", "isaacsim.core.nodes.IsaacGetViewportRenderProduct"), # 생성된 뷰포트에 RenderProduct를 생성하고, 해당 객체의 경로(renderProductPath)를 출력
                    # RenderProduct: 카메라 장면을 AOV 버퍼를 통해 다운스트림 하기 위한 오프스크린 렌더 타깃
                    ("setCamera", "isaacsim.core.nodes.IsaacSetCameraOnRenderProduct"), # 생성된 RenderProduct 경로와 실제 카메라 프림(Prim)을 최종적으로 결속
                    ("cameraHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("cameraHelperInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                    ("cameraHelperDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ],

                keys.CONNECT: [
                    ("OnTick.outputs:tick", "createViewport.inputs:execIn"),
                    ("createViewport.outputs:execOut", "getRenderProduct.inputs:execIn"),
                    ("createViewport.outputs:viewport", "getRenderProduct.inputs:viewport"),
                ],
            }
        )

    # 카메라 퍼블리셔 그래프
    def build_camera_ros_graph(self, graph_path: str = "/ActionGraph"):
        """
        - Viewport 생성 -> RenderProduct 획득 -> 특정 Camera prim을 RenderProduct에 바인딩
        - ROS2CameraHelper / ROS2CameraInfoHelper로 이미지/캠정보 퍼블리시
        """
        import omni.graph.core as og
        keys = og.Controller.Keys

        (graph, _, _, _) = og.Controller.edit(
            {
                "graph_path": graph_path,
                "evaluator_name": "push",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
            },
            {
                keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("createViewport", "isaacsim.core.nodes.IsaacCreateViewport"),
                    ("getRenderProduct", "isaacsim.core.nodes.IsaacGetViewportRenderProduct"),
                    ("setCamera", "isaacsim.core.nodes.IsaacSetCameraOnRenderProduct"),
                    ("cameraHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("cameraHelperInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                    ("cameraHelperDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ],

                keys.CONNECT: [
                    ("OnTick.outputs:tick", "createViewport.inputs:execIn"),
                    ("createViewport.outputs:execOut", "getRenderProduct.inputs:execIn"),
                    ("createViewport.outputs:viewport", "getRenderProduct.inputs:viewport"),

                    ("getRenderProduct.outputs:execOut", "setCamera.inputs:execIn"),
                    ("getRenderProduct.outputs:renderProductPath", "setCamera.inputs:renderProductPath"),
                    ("setCamera.outputs:execOut", "cameraHelperRgb.inputs:execIn"),
                    ("setCamera.outputs:execOut", "cameraHelperInfo.inputs:execIn"),
                    #("setCamera.outputs:execOut", "cameraHelperDepth.inputs:execIn"),

                    ("getRenderProduct.outputs:renderProductPath", "cameraHelperRgb.inputs:renderProductPath"),
                    ("getRenderProduct.outputs:renderProductPath", "cameraHelperInfo.inputs:renderProductPath"),
                    #("getRenderProduct.outputs:renderProductPath", "cameraHelperDepth.inputs:renderProductPath"),
                    #("getRenderProduct.outputs:renderProductPath", "cameraHelperRgb.inputs:renderProductPath"),
                    #("getRenderProduct.outputs:renderProductPath", "cameraHelperInfo.inputs:renderProductPath"),
                    #("getRenderProduct.outputs:renderProductPath", "cameraHelperDepth.inputs:renderProductPath"),

                    # exec 트리거도 직접 연결
                    #("OnTick.outputs:tick", "cameraHelperRgb.inputs:execIn"),
                    #("OnTick.outputs:tick", "cameraHelperInfo.inputs:execIn"),
                    #("OnTick.outputs:tick", "cameraHelperDepth.inputs:execIn"),
                ],

                keys.SET_VALUES: [
                    ("createViewport.inputs:viewportId", 0),
                    ("cameraHelperRgb.inputs:frameId", "Camera"),
                    ("cameraHelperRgb.inputs:topicName", "yolo/image_raw"),
                    ("cameraHelperRgb.inputs:type", "rgb"),
                    ("cameraHelperInfo.inputs:frameId", "Camera"),
                    ("cameraHelperInfo.inputs:topicName", "camera_info"),
                    #("cameraHelperDepth.inputs:frameId", "Camera"),
                    #("cameraHelperDepth.inputs:frameId", "Camera"),
                    #("cameraHelperDepth.inputs:topicName", "depth"),
                    #("cameraHelperDepth.inputs:type", "depth"),
                    ("setCamera.inputs:cameraPrim", "/World/Spot/ATS/ATS/link2/Xform/Camera"), # renderproduct경로와 카메라 프림을 최종 결속

                    # [THERMAL CAMERA PLACEHOLDER] - 열화상 카메라 기능 추가 시 아래 패턴 참고
                    # ("thermalHelper.inputs:frameId", "thermal_frame"),
                    # ("thermalHelper.inputs:topicName", "/thermal/image_raw"),
                    # ("thermalHelper.inputs:type", "thermal"), # IsaacSim 2023+ 에서는 'thermal_linear' 또는 별도 Annotator 사용

                    #==================================================
                    #("createViewport.inputs:viewportId", 0),
                    #("cameraHelperRgb.inputs:frameId", "sim_camera"),
                    #("cameraHelperRgb.inputs:topicName", "yolo/image_raw"),
                    #("cameraHelperRgb.inputs:type", "rgb"),
                    #("cameraHelperInfo.inputs:frameId", "sim_camera"),
                    #("cameraHelperInfo.inputs:topicName", "camera_info"),
                    #("cameraHelperDepth.inputs:frameId", "sim_camera"),
                    #("cameraHelperDepth.inputs:frameId", "sim_camera"),
                    #("cameraHelperDepth.inputs:type", "depth"),
                    # CameraPrim은 풀 Path 직접 지정
                    #("getRenderProduct.inputs:cameraPrim", Sdf.Path("/World/Spot/ATS/ATS/link2/Xform/Camera")),
                    #==================================================
                ],
            },
        )
        og.Controller.evaluate_sync(graph)


    # 제어/상태(Spot 조인트·Twist), 시간(Clock), 관성(IMU), 위치추정(Odometry), 그리고 좌표계 동기화(TF)
    def build_ats_graph(self, graph_path: str ="/ATSActionGraph"):
        import omni.graph.core as og
        keys = og.Controller.Keys

        BODY_PRIM = Sdf.Path(self.assets["body_prim"])
        CAMERA_BASE_PRIM = Sdf.Path(self.assets["camera_base_prim"])
        SPOT_PRIM        = Sdf.Path(self.assets["spot_prim"])
        CAMERA_PRIM      = Sdf.Path(self.assets["camera_prim"])
        IMU_PRIM = Sdf.Path(self.assets["imu_prim"])
        ODOM_PRIM        = Sdf.Path("/World/odom")   # 프레임 "odom"과 연결

        (graph, _, _, _) = og.Controller.edit(
            {"graph_path":graph_path, "evaluator_name":"execution"}, # 시뮬레이션 플레이 중 매 프레임 자동 평가
            {   
                keys.CREATE_NODES: [ # 그래프 생명주기와 ROS 컨텍스트
                    # 한번 만 필요한 초기 세팅을 위한 1회 트리거
                    ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
                    # 매 프레임 반복 세팅을 위한 트리거
                    ("Tick",           "omni.graph.action.OnPlaybackTick"),
                    # ROS2 컨텍스트 모든 브리지 노드에 context 전달 핸들 ( 퍼블리/ 구동조건 )
                    ("Context",        "isaacsim.ros2.bridge.ROS2Context"),
                    # 제어/상태
                    ("SubscribeJointState",    "isaacsim.ros2.bridge.ROS2SubscribeJointState"), # 받은 jointNames를 컨트롤러에 연결
                    ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"), # Spot프림에 힘/속도/위치 명령 적용
                    ("SubscribeTwist",         "isaacsim.ros2.bridge.ROS2SubscribeTwist"), # /cmd_vel 구독(필요시 컨트롤러 입력/오도메트리 계산에 사용)
                    ("SpotPublishJointState",  "isaacsim.ros2.bridge.ROS2PublishJointState"), # 현재 관절 위치, 속도, 토크를 퍼블리시(디버그/상위제어용)
                    # Clock
                    ("ClockTime", "isaacsim.core.nodes.IsaacReadSimulationTime"), # 시뮬시간(/clock)을 ROS 전체에 배포 /clock 덕분에 use_sim_time 노드들이 같은 시간축으로 동작
                    ("ClockPub",  "isaacsim.ros2.bridge.ROS2PublishClock"),
                    # IMU
                    ("OnTickIMU",       "omni.graph.action.OnTick"), # framePeriod=0, onlyPlayback=True -> 플레이 중 매 프레임 IMU 평가
                    ("ImuComputeOdom",  "isaacsim.core.nodes.IsaacComputeOdometry"), # IMU 관측치(orientation, angular velocity, linear acceleration) 합성 [일종의 로봇의 바디 포즈 기반 가짜 IMU 값 합성]
                    ("ImuReadSimTime",  "isaacsim.core.nodes.IsaacReadSimulationTime"), # 같은 프레임의 Odometry/TF와 시간상 동기화
                    ("ImuPublish",      "isaacsim.ros2.bridge.ROS2PublishImu"), # 합성도니 IMU 데이터를 /imu로 퍼블리시 [frameid", self.ros["frames"]["imu_frame"]->frame_id는 TF 프레임 이름과 일치]
                    # Odom 매 프레임 계산
                    ("OdomCompute", "isaacsim.core.nodes.IsaacComputeOdometry"), # 바디의 포즈/속도를 가져와 오도메트리 항목(position, orientation, linear/angular velocity) 생성, 이를 활용해 odom -> base_link 정보 생성
                    ("OdomPublish", "isaacsim.ros2.bridge.ROS2PublishOdometry"), # /odom 메시지 발행 [odomFrameId= "odom" -> 누적 이동 기준(점프 없음)/ chassisFrameId = "base_link"(=body): 로봇 본체 기준]
                    # TF (cam/imu)
                    ("TFReadTime_camimu", "isaacsim.core.nodes.IsaacReadSimulationTime"), 
                    ("TFPubCam",          "isaacsim.ros2.bridge.ROS2PublishTransformTree"), # camera_base -> camera 고정변환 퍼블리시 (짐벌 움직임이 카메라 포즈에 자연스럽게 반영)
                    ("TFPubImu",          "isaacsim.ros2.bridge.ROS2PublishTransformTree"), # body-> imu_link 고정 변환 퍼블리시 (IMU 메시지 frame_id와 child 프레임 이름 반드시 일치)
                    # TF (odom)
                    ("TFReadTime_odom",   "isaacsim.core.nodes.IsaacReadSimulationTime"), 
                    ("TFPubOdom",         "isaacsim.ros2.bridge.ROS2PublishTransformTree"), # odom -> base_link(body) 변환 퍼블리시 (/odom 메시지와 RF의 위치 관계를 항상 동일하게 유지)
                ],
                # create_nodes = [
                #     ("Tick",   "omni.graph.action.OnPlaybackTick"),
                #     ("RunSim", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                #     #("RunSim", "isaacsim.core.nodes.IsaacRunOneSimulationFrame"),
                #     ("Ctx",    "isaacsim.ros2.bridge.ROS2Context"),
                # ]

                
                
                keys.CONNECT: [
                    # 초기화 계열(한 번)
                    ("OnImpulseEvent.outputs:execOut", "SubscribeJointState.inputs:execIn"),
                    ("OnImpulseEvent.outputs:execOut", "ArticulationController.inputs:execIn"),
                    ("OnImpulseEvent.outputs:execOut", "SpotPublishJointState.inputs:execIn"),
                    ("OnImpulseEvent.outputs:execOut", "SubscribeTwist.inputs:execIn"),
                    ("OnImpulseEvent.outputs:execOut", "ClockPub.inputs:execIn"),

                    # 컨텍스트 연결
                    ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                    ("Context.outputs:context", "SpotPublishJointState.inputs:context"),
                    ("Context.outputs:context", "ClockPub.inputs:context"),

                    # 조인트명 전달
                    ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),

                    # 시계
                    ("ClockTime.outputs:simulationTime", "ClockPub.inputs:timeStamp"),

                    # IMU(매 프레임)
                    ("OnTickIMU.outputs:tick", "ImuComputeOdom.inputs:execIn"),
                    ("OnTickIMU.outputs:tick", "ImuPublish.inputs:execIn"),
                    ("Context.outputs:context", "ImuPublish.inputs:context"),
                    ("ImuComputeOdom.outputs:orientation",       "ImuPublish.inputs:orientation"),
                    ("ImuComputeOdom.outputs:angularVelocity",    "ImuPublish.inputs:angularVelocity"),
                    ("ImuComputeOdom.outputs:linearAcceleration", "ImuPublish.inputs:linearAcceleration"),
                    ("ImuReadSimTime.outputs:simulationTime",     "ImuPublish.inputs:timeStamp"),

                    # Odom(매 프레임)
                    ("Tick.outputs:tick", "OdomCompute.inputs:execIn"),
                    ("Tick.outputs:tick", "OdomPublish.inputs:execIn"),
                    ("Context.outputs:context", "OdomPublish.inputs:context"),
                    ("OdomCompute.outputs:position",        "OdomPublish.inputs:position"),
                    ("OdomCompute.outputs:orientation",     "OdomPublish.inputs:orientation"),
                    ("OdomCompute.outputs:linearVelocity",  "OdomPublish.inputs:linearVelocity"),
                    ("OdomCompute.outputs:angularVelocity", "OdomPublish.inputs:angularVelocity"),
                    ("ImuReadSimTime.outputs:simulationTime", "OdomPublish.inputs:timeStamp"),

                    # TF(cam/imu/odom)
                    ("Tick.outputs:tick", "TFPubCam.inputs:execIn"),
                    ("Tick.outputs:tick", "TFPubImu.inputs:execIn"),
                    ("Tick.outputs:tick", "TFPubOdom.inputs:execIn"),
                    ("Context.outputs:context", "TFPubCam.inputs:context"),
                    ("Context.outputs:context", "TFPubImu.inputs:context"),
                    ("Context.outputs:context", "TFPubOdom.inputs:context"),
                    ("TFReadTime_camimu.outputs:simulationTime", "TFPubCam.inputs:timeStamp"),
                    ("TFReadTime_camimu.outputs:simulationTime", "TFPubImu.inputs:timeStamp"),
                    ("TFReadTime_odom.outputs:simulationTime",   "TFPubOdom.inputs:timeStamp"),
                ],

                keys.SET_VALUES: [
                    # 제어 타겟(Spot)
                    ("ArticulationController.inputs:robotPath", str(SPOT_PRIM)),
                    ("ArticulationController.inputs:targetPrim", str(SPOT_PRIM)),

                    ("SubscribeJointState.inputs:topicName", self.ros["topics"]["joint_cmd"]),
                    ("SubscribeTwist.inputs:topicName",      self.ros["topics"]["cmd_twist"]),
                    ("SpotPublishJointState.inputs:topicName", self.ros["topics"]["spot_joint_state_pub"]),
                    ("SpotPublishJointState.inputs:targetPrim", str(SPOT_PRIM)),

                    # IMU
                    ("ImuComputeOdom.inputs:chassisPrim", str(SPOT_PRIM)),
                    ("ImuPublish.inputs:frameId",         self.ros["frames"]["imu_frame"]),
                    ("ImuPublish.inputs:topicName",       self.ros["topics"]["imu"]),
                    ("ImuPublish.inputs:publishOrientation",         True),
                    ("ImuPublish.inputs:publishAngularVelocity",    True),
                    ("ImuPublish.inputs:publishLinearAcceleration", True),
                    ("OnTickIMU.inputs:framePeriod", 0),
                    ("OnTickIMU.inputs:onlyPlayback", True),

                    # Odom (odom -> base_link)
                    ("OdomCompute.inputs:chassisPrim", str(SPOT_PRIM)),
                    ("OdomPublish.inputs:topicName",   self.ros["topics"]["odom"]),
                    ("OdomPublish.inputs:odomFrameId", self.ros["frames"]["odom"]),
                    ("OdomPublish.inputs:chassisFrameId", "body"),

                    # TF - Cam / IMU (부모: base_link)
                    ("TFPubCam.inputs:parentPrim", str(BASE_LINK_PRIM)),
                    ("TFPubCam.inputs:parentPrim", str(CAMERA_BASE_PRIM)),
                    ("TFPubCam.inputs:targetPrims", [str(CAMERA_PRIM)]),
                    ("TFPubImu.inputs:parentPrim", str(BODY_PRIM)),
                    ("TFPubImu.inputs:targetPrims", [str(IMU_PRIM)]),

                    # TF - Odom 트리 (부모: /World/odom, 자식: base_link)
                    ("TFPubOdom.inputs:parentPrim", str(ODOM_PRIM)),
                    ("TFPubOdom.inputs:targetPrims", [str(BODY_PRIM)]),
                ],
            }
        )
        og.Controller.evaluate_sync(graph)

    def build_lidar_ros_graph(self, full_cfg: dict, graph_path: str = "/LidarGraph"):
        import omni.graph.core as og
        from pxr import UsdGeom, Gf, Sdf

        keys = og.Controller.Keys

        lidar  = full_cfg["sensors"]["lidar"]  # e.g., {"create_2d": True, "prim_2d": "/World/Spot/Lidar2D", ...}
        frames = full_cfg["ros"]["frames"]     # e.g., {"base_scan": "base_scan"}
        topics = full_cfg["ros"]["topics"]     # e.g., {"scan": "/scan", "point_cloud": "/point_cloud"}
        domain = full_cfg.get("ros", {}).get("domain_id", None)

        create_nodes = [
            ("Tick",    "omni.graph.action.OnPlaybackTick"),  # 매프레임마다 업데이트하는 트리거 역할 (틱(트리거 신호) → 프레임 실행 → 센서 샘플링 순서 보장/ RenderProduct가 시뮬 상태 미반영으로 빈 프레임을 뽑는 문제 방지)
            ("RunSim",  "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"), # RunSim.step → RenderProduct.execIn → RenderProduct.execOut → LiDAR Helper.execIn
            #("RunSim", "isaacsim.core.nodes.IsaacRunOneSimulationFrame"),      # "시뮬레이션 스텝 → RP 평가 → 헬퍼 퍼블리시" 순서를 강제/ 한 프레임 속 "장면 갱신 → 레이캐스트 → ROS 직렬화" 과정 연결
            ("Ctx",     "isaacsim.ros2.bridge.ROS2Context"), # 모든 브리지 노드의 공통 ROS 핸들 (domainId 주입 지원 -> 동일 머신 다중 실험 충돌 방지)
        ]

        connect, setvals = [], []

        # --- 공통: Tick -> RunSim ---
        connect += [
            ("Tick.outputs:tick", "RunSim.inputs:execIn"),
        ]

        # --- ROS2 Context 설정 ---
        # Domain ID가 있으면 넣어줌, 없으면 기본값 (환경변수 사용 여부는 False로 가정)
        setvals += [
            ("Ctx.inputs:useDomainIdEnvVar", False),
        ]

        if domain is not None:
            setvals += [("Ctx.inputs:domainId", int(domain))]

        # ==================== 2D LaserScan ==================== # LiDAR는 2D(LaserScan) / 3D(PointCloud2) 두 경로로 분리
        if lidar.get("create_2d", True):            # 각 경로마다 전용 RenderProduct(RP_2D, RP_3D) 생성
            create_nodes += [                       # RenderProduct의 cameraPrim에 LiDAR 프림을 바인딩
                ("RP_2D",      "isaacsim.core.nodes.IsaacCreateRenderProduct"), # ③
                ("TFReadTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("TFLidar2D",  "isaacsim.ros2.bridge.ROS2PublishTransformTree"), # body -> Lidar 고정변환을 매 프레임 퍼블리시
                ("Lidar2D",    "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
            ]
            ("Lidar2D.inputs:enabled",    True),
            ("Lidar2D.inputs:type",       "laser_scan"),
            # 흐름: RunSim.step -> RP_2D.execIn -> RP_2D.renderProductPath -> Lidar2D.renderProductPath
            #      RunSim.step -> Lidar2D.execIn (매 프레임 발행)
            connect += [
                ("RunSim.outputs:step",                 "RP_2D.inputs:execIn"),
                ("RP_2D.outputs:renderProductPath",     "Lidar2D.inputs:renderProductPath"),
                #("RunSim.outputs:step",                "Lidar2D.inputs:execIn"),
                ("RP_2D.outputs:execOut",               "Lidar2D.inputs:execIn"),
                ("Ctx.outputs:context",                 "Lidar2D.inputs:context"),

                # TF 갱신
                ("Tick.outputs:tick",                   "TFLidar2D.inputs:execIn"),
                ("Ctx.outputs:context",                 "TFLidar2D.inputs:context"),
                ("TFReadTime.outputs:simulationTime",   "TFLidar2D.inputs:timeStamp"),
            ]

            # 기본 RP(해상도는 필요시 조정)
            setvals += [
                ("RP_2D.inputs:enabled",     True),
                ("RP_2D.inputs:cameraPrim",  Sdf.Path(lidar["prim_2d"])),
                ("RP_2D.inputs:width",       640),
                ("RP_2D.inputs:height",      480),

                # Lidar Helper
                ("Lidar2D.inputs:enabled",         True),
                ("Lidar2D.inputs:type",            "laser_scan"),
                ("Lidar2D.inputs:topicName",       topics["scan"]), #  토픽: /scan sensor_msgs/LaserScan
                ("Lidar2D.inputs:frameId",         frames["base_scan"]), # frameId는 TF 트리 child 프레임 이름과 반드시 일치 2D → frames["base_scan"]
                ("Lidar2D.inputs:frameSkipCount",  0), # 메시지 퍼블리시 빈도 
                ("Lidar2D.inputs:nodeNamespace",   ""),                     
                # ("Lidar2D.inputs:publishFullScan", True), # 회전형 LIDAR에서 바퀴 음영 스캔 타임 발생해서 사용
                # (publishFullScan(회전형Lidar)을 활용해 특정 동작 옵션을 활성화 -> 실제 산출 레이트는 [Lidar 프림 스펙 * 프레임 스케줄링]에 의해 결정
                # RunOneSimulationFrame 호출 주기와 frameSkipCount를 함께 조정

                # TF 부모/자식
                ("TFLidar2D.inputs:parentPrim", str(self.assets["body_prim"])),
                ("TFLidar2D.inputs:targetPrims", [str(lidar["prim_2d"])]),
            ]

        # --- 그래프 생성/적용 ---
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {keys.CREATE_NODES: create_nodes, keys.CONNECT: connect, keys.SET_VALUES: setvals},
        )
        # 즉시 계산(그래프 활성화)
        og.Controller.evaluate_sync(graph_path)