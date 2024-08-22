# Autonomous Drive with Scout-Mini

### 코드 다운(vscode 사용시)

1. 터미널에서 workspace 및 src 폴더 생성 (ex) mkdir -p scout_ws/src)
2. vscode 실행
3. 왼쪽 창의 세번쩨 항목 '소스 제어' 선택 후 '리포지토리 복제' 누르기
4. github 로그인(아마) -> erp-cml 선택
5. scout_ws/src 폴더 안에서 레포지토리 위치시키기
6. 터미널의 scout_ws폴더에서 catkin_make로 빌드 후 코드 실행하기

---
### Scout-Mini 초기설정
https://github.com/agilexrobotics/scout_ros 을 참고해서 패키지 설치 후 빌드 진행


처음 한번 실행\
`sudo modprobe gs_usb`\
`rosrun scout_bringup setup_can2usb.bash`\
`rosrun scout_bringup bringup_can2usb.bash`

`candump can0` → can 되는지 테스트하는 명령어\
`roslaunch scout_bringup scout_mini_robot_base.launch` → bringup 켜기\
`roslaunch scout_bringup scout_teleop_keyboard.launch` → keyboard로 teleop하기\
`roslaunch scout_description display_scout_mini.launch` → RViz에서 visualization

odometry publsih & 좌표 설정
1. scout_bringup/launch/scout_mini_robot_base.launch에서 `pub_tf:= true`로 설정
2. scout_base/src/scout_messenger.cpp에서 288, 301, 337, 350번 line을 `z = 0.183498` 로 수정
3. RViz에서 odom을 기준으로 좌표 보기


---
### 코드 실행

카메라 실행
- `roslaunch realsense2_camera rs_camera.launch` 
- 또는 `rosrun camera_data turn_on_camera.py`

카메라 이미지 처리
- `rosrun camera_data main_ROS_prs.py`

reference point를 기반으로 MPC 제어(solver: gurobi)
- `rosrun testdrive drive_gurobi.py`

Scout-Mini bringup하기 (CAN 통신?)
- `sudo ip link set can0 up type can bitrate 500000`
- `roslaunch scout_bringup scout_mini_robot_base.launch`

Rviz에서 확인하기(rviz 따로 켤 필요 없음)
- `roslaunch visualization visualization.launch`
