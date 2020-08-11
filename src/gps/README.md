### pure_pursuit

- Nodes
  - pure_pursuit : pure pursuit 알고리즘을 실행해주는 노드입니다.
  - coordinate2pos : 현재 차량의 UTM-K 좌표를 받아 Yaw 값을 구하고, Pose 데이터로 변환해 publish 해주는 노드입니다.
  
- Subscribed Topics
  - pure_pursuit
    - /current_pose : UTM-K 좌표계로 변환된 현재 위치와 차량의 Yaw 값
  
  - coordinate2pos
    - /utmk_coordinate : UTM-K 좌표계로 변환된 현재 위치

- Published Topics
  - pure_pursuit
    - /control_value : 차량의 제어값
    - /target_point : pure pursuit 알고리즘에서 목표점에 해당하는 좌표. 시각화 용도
  
  - coordinate2pos
    - /current_pose : UTM-K 좌표계로 변환된 현재 위치와 차량의 Yaw 값
  
  - Pure Pursuit을 포함한 Launch Files을 실행할 때
    - `roslaunch package_name launchfile_name path:=pathfile_name ld:=ld vel:=vel`
    
### utmk_coordinate

- Nodes
  - wgs84_to_utmk.py : 위도, 경도 데이터를 입력받아 UTM-K 좌표계로 변환해주는 노드입니다.
  - path_maker.py : gps를 돌아다니며 경로를 찍으면 pure_pursuit에서 사용할 수 있는 path 형식으로 만들어주는 노드입니다.

- Subscribed Topics
  - wgs84_to_utmk.py
    - /gps_front/fix : gps 위도, 경도 데이터. 이름 바꿔주면 좋을듯
  
  - path_maker.py
    - /utmk_coordinate : UTM-K 좌표계로 변환된 현재 위치
    
- Published Topics
  - wgs84_to_utmk.py
    - /utmk_coordinate : UTM-K 좌표계로 변환된 현재 위치
    
### ublox_gps

- Nodes
  - ublox_gps : gps 수신기로부터 raw 데이터를 받아 파싱해 publish 해주는 노드입니다.

- Published Topics
  - /gps_front/fix : gps 위도, 경도 데이터. 이름 바꿔주면 좋을듯
  
- Launch Files
  - pure_pursuit.launch : 시각화 없는 pure pursuit 알고리즘 실행
  - pure_pursuit_with_rviz.launch : 실시간 시각화되는 pure pursuit 알고리즘 실행
  - path_maker.launch : path 생성, utmk_coordinate/paths에 저장
  
### rviz_visualization
- 시각화 패키지, 실행 방법은 README에 적혀있습니다.
- bag 파일을 이용한 시각화와 실시간 시각화 모두 가능합니다.
  
