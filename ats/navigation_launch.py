# 경로 계획에 필요한 네비게이션 엔진을 실행
    # planner_server: 전역 경로 생성
    # controller_server: 경로 → cmd_vel 속도 명령으로 변환
    # smoother_server: 경로 곡률/형태 보정
    # behavior_server: 회전/후진/복구 동작 처리
    # bt_navigator: 전체 흐름을 행동 트리(BT)로 조율 비용맵 구성
# /map → Static Layer, /scan → Obstacle Layer, inflation_radius → 장애물 주변 완충 영역 확보