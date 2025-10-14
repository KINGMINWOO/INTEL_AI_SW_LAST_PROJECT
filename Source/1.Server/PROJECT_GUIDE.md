# 프로젝트 구조 및 실행 가이드

이 문서는 Intel Final Project 저장소를 처음 접하는 개발자가 빠르게 환경을 이해하고 주요 컴포넌트를 활용할 수 있도록 정리한 안내서입니다. 각 섹션은 현재 구성된 파일과 서비스, 실행 흐름에 따라 구성되어 있으며, 필요 시 워크플로우에 맞춰 확장하거나 수정할 수 있습니다.

## 1. 전체 개요
- **주요 기능**: YOLOv8 기반 객체 감지, TurtleBot Nav2 경로 제어, 스마트팜 환경/영상 데이터 수집 및 저장.
- **핵심 구성 요소**
  - `server_highres_output.py`: 클라이언트 영상 스트리밍 수신, YOLO 추론, Nav2 제어, 사용자 명령 처리.
  - `client_turtle.py`: TurtleBot용 스트리밍 클라이언트(ROS 의존 제거, 영상 전송 전용).
  - `nav2_bridge.py`: Nav2 `NavigateToPose` 액션을 다루는 비동기 브리지.
  - `farm_data_service.py`: 스마트팜 센서/CCTV 데이터를 수집해 MariaDB에 저장하는 Flask 서비스.
  - Gazebo + Nav2 시뮬레이션 환경(TurtleBot3) / 실제 TurtleBot과의 연동.

## 2. 개발 환경 준비
1. **필수 패키지 설치 (Ubuntu / ROS 2 Humble 기준)**
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop ros-humble-turtlebot3-gazebo \
       ros-humble-turtlebot3-navigation2 ros-humble-navigation2 ros-humble-nav2-bringup
   sudo apt install python3-sqlalchemy python3-pymysql python3-flask
   ```
2. **Python 가상환경 권장**
   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   pip install -r requirements.txt  # 필요 시 구성
   ```
3. **MariaDB 준비** (선택)
   ```bash
   sudo apt install mariadb-server
   sudo mysql_secure_installation
   mysql -u root -p
   > CREATE DATABASE farm DEFAULT CHARACTER SET utf8mb4;
   > CREATE USER 'farmuser'@'%' IDENTIFIED BY '비밀번호';
   > GRANT ALL PRIVILEGES ON farm.* TO 'farmuser'@'%';
   ```

## 3. 시뮬레이션 및 Nav2 구성
시뮬레이터 없이 실제 TurtleBot과 연동할 수도 있으나, 개발/테스트 단계에서는 Gazebo + Nav2 환경을 사용하는 것이 안전합니다.

### 3.1 Gazebo 시뮬레이터 실행
```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py headless:=True
```
- GUI가 필요하면 `headless:=True`를 제거합니다.

### 3.2 Nav2 (TurtleBot3 전용) 실행
```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/opt/ros/humble/share/turtlebot3_navigation2/map/map.yaml
```

### 3.3 RViz로 초기 포즈 설정
```bash
source /opt/ros/humble/setup.bash
rviz2 -d /opt/ros/humble/share/turtlebot3_navigation2/rviz/turtlebot3_nav2.rviz
```
- `/map` 토픽이 보이면 `2D Pose Estimate`로 로봇 위치 추정 입력 → `2D Goal Pose`로 경로 확인.

## 4. 서버 구성 요소
### 4.1 영상/감지 서버 (`server_highres_output.py`)
- 역할:
  - 클라이언트 CCTV/TurtleBot 영상 스트림 수집.
  - YOLOv8 추론, `ripe`/`rotten` 감지 좌표 파악.
  - Nav2 브리지와 연동해 `ripe@go`, `rotten@go`, `stop` 등 명령 처리.
  - 감지 조건(3초 내 5회) 충족 시 자동 STOP 및 로봇팔 명령 (`robot@high/middle/low`).
  - `robot@done` 수신 시 중단됐던 Nav2 이동 재개.

- 실행: Nav2 환경이 준비된 상태에서
  ```bash
  source /opt/ros/humble/setup.bash
  python3 server_highres_output.py
  ```
  - 시작 시 `Nav2 브리지를 초기화했습니다.` 로그가 나타나면 Nav2 액션 서버와 연결 성공.
  - 사용자 채널(예: `USER01` 클라이언트)에서 `ripe@go`, `ripe@done`, `rotten@go`, `rotten@done`, `dump@done`, `stop` 명령을 전송할 수 있습니다.

### 4.2 Nav2 브리지 (`nav2_bridge.py`)
- 역할:
  - ROS2 `rclpy`를 통해 Nav2 액션(`/navigate_to_pose`)을 비동기로 호출.
  - 목표 취소(`cancel_current_goal`) 기능 제공.
  - 서버 모듈에서 재사용 용이하도록 별도 파일로 분리.
- 별도 실행 없음. `server_highres_output.py`에서 자동 로드.

## 5. 클라이언트 구성 요소
### 5.1 TurtleBot 스트리밍 클라이언트 (`client_turtle.py`)
- 기능: 카메라 영상을 MJPG로 인코딩하여 서버에 전송. ROS 제어 코드는 제거되었습니다.
- 실행:
  ```bash
  source /opt/ros/humble/setup.bash
  python3 client_turtle.py
  ```
- 주의: 서버와 동일한 인증 정보(`CLIENT_ID`, `CLIENT_PASSWORD`)가 `idlist.txt`와 일치해야 합니다.

### 5.2 사용자 채널 클라이언트 (예시)
- 별도 스크립트나 텔넷을 사용해 `USERxx`로 인증 후 명령 문자열 전송.
- 예: `ripe@go` → 좌표 `(0.0, 1.5)` 이동 시작, `stop` → Nav2 취소 + TurtleBot 정지.

## 6. 스마트팜 데이터 수집 서비스
### 6.1 `farm_data_service.py`
- Flask 기반 REST API 서비스.
- 입력 형식: `POST /api/farm-data` with JSON `{ 
