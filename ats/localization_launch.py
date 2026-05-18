# 기존 지도를 로딩하고 AMCL을 통해 로봇 위치를 추정
  # 지도 : map_server가 저장된 지도 파일(spot_ats_map.yaml)을 로드하고 map 토픽으로 퍼블리시
  # 위치 추정 : AMCL이 /scan + /odom 데이터를 이용해 현재 로봇의 위치를 지도 좌표계에서 추정
    # 이때, 오도메트리가 odom → body 변환 / AMCL이 map → odom 이어주어 전체 좌표계(map → odom → body)가 닫힘