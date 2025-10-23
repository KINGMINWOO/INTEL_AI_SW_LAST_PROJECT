# [Intel]AI+SW 아카데미 최종 종합 프로젝트

## SERVER

### 이 폴더에는 프로젝트 전체 데이터 흐름을 제어하는 서버 및 작물 상태 추론 AI 모델 관련 소스코드가 업로드됩니다.
# 프로젝트 구조 및 실행 가이드

이 문서는 Intel Final Project 저장소를 처음 접하는 개발자가 빠르게 환경을 이해하고 주요 컴포넌트를 활용할 수 있도록 정리한 안내서입니다. 각 섹션은 현재 구성된 파일과 서비스, 실행 흐름에 따라 구성되어 있으며, 필요 시 워크플로우에 맞춰 확장하거나 수정할 수 있습니다.

## 1. 전체 개요
- **주요 기능**: YOLOv8 기반 객체 감지, TurtleBot Nav2 경로 제어, 스마트팜 환경/영상 데이터 수집 및 저장.
- **핵심 구성 요소**
  - `server_highres_output.py`: 클라이언트 영상 스트리밍 수신, YOLO 추론, Nav2 제어, 사용자 명령 처리.
  - `client_turtle.py`: TurtleBot용 스트리밍 클라이언트(ROS 의존 제거, 영상 전송 전용).
  - `client_cctv.py`: CCTV 영상·센서 업링크 및 테스트용 명령 송신 스크립트.
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
   pip install -r requirements.txt
   ```
3. **MariaDB 준비** (선택)
   ```bash
   sudo apt install mariadb-server
   sudo mysql_secure_installation
   mysql -u root -p
   > CREATE DATABASE smart_farm DEFAULT CHARACTER SET utf8mb4;
   > CREATE USER 'user01'@'%' IDENTIFIED BY 'user1234';
   > GRANT ALL PRIVILEGES ON smart_farm.* TO 'user01'@'%';
   ```
   - 서버 기본 연결 문자열은 `mysql+pymysql://user01:user1234@192.168.0.4:3306/smart_farm`입니다. 다른 계정을 사용하려면 `FARM_DB_URL` 환경 변수를 덮어쓰세요.

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
  - YOLOv8 추론, `ripe`/`rotten` 감지 좌표 파악(탐지는 `TURTLE*` 스트림에만 적용).
  - Nav2 브리지와 연동해 `ripe@go`, `rotten@go`, `stop` 등 명령 처리.
  - 감지 조건(3초 내 5회) 충족 시 자동 STOP 및 로봇팔 명령 (`robot@high/middle/low`).
  - `robot@done` 수신 시 중단됐던 Nav2 이동 재개.
  - CCTV 센서 문자열(`air@...`, `land@...`)을 수신해 MariaDB(`farm_air_samples`, `farm_land_samples`)에 저장.
  - 사용자 채널에서 `[CCTV01]LED@HIGH`와 같은 메시지를 전달하면 대상 CCTV 소켓으로 그대로 포워딩.

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
- `[CCTV01]LED@LOW`와 같이 대괄호로 시작하면 해당 CCTV 클라이언트로 문자열이 그대로 전달됩니다.

### 5.3 CCTV 스트리밍 클라이언트 (`client_cctv.py`)
- `/dev/video0` 영상을 10FPS MJPG로 인코딩해 서버로 송신합니다.
- 표준 입력에서 `air@온도@습도@공기질@조도`, `land@온도@습도@EC@pH`를 입력하면 5초 간격으로 서버에 전송하여 DB에 기록됩니다.
- 서버에서 오는 모든 문자열을 터미널에 출력하고, `[CCTV1]` 접두사가 붙은 메시지는 UART3로 전달합니다.
- 실행 예:
  ```bash
  python3 client_cctv.py
  ```
  - `pyserial` 설치가 필요합니다. (`pip install pyserial`)
  - UART를 사용하지 않을 경우 포트 리스트를 비워두거나 오류 메시지만 무시하면 됩니다.

## 6. 스마트팜 데이터 및 데이터베이스
- `server_highres_output.py`는 센서/메시지 처리를 통합하며 아래 테이블을 MariaDB에 생성합니다.
  - `farm_air_samples`: `air@...` 문자열에서 추출한 공기 온도·습도·공기질·조도.
  - `farm_land_samples`: `land@...` 문자열에서 추출한 토양 온도·습도·EC·pH.
  - `farm_tomato_snapshots`: CCTV 프레임 분석 결과를 저장할 때 사용(현재 기본 흐름에서는 미사용이지만 스키마는 자동 생성됨).
- 기본 연결 문자열은 `mysql+pymysql://user01:user1234@192.168.0.4:3306/smart_farm`입니다. 다른 환경에서는 `FARM_DB_URL`로 덮어씌우세요.

### 6.1 데이터 확인 예시
```sql
USE smart_farm;
SHOW TABLES;
SELECT * FROM farm_air_samples ORDER BY id DESC LIMIT 5;
SELECT * FROM farm_land_samples ORDER BY id DESC LIMIT 5;
SELECT * FROM farm_tomato_snapshots ORDER BY id DESC LIMIT 5;
```
- `SQLAlchemy`, `PyMySQL` 패키지가 설치돼 있어야 하며, 필요 시 `sudo apt install python3-sqlalchemy python3-pymysql` 또는 `pip install SQLAlchemy pymysql`로 준비합니다.

### 6.2 추가 REST 수집기 (`farm_data_service.py`)
- 외부 장치가 HTTP로 데이터 업로드해야 할 때만 사용합니다.
  ```bash
  FARM_DB_URL='mysql+pymysql://user01:user1234@192.168.0.4:3306/smart_farm' \
  python3 farm_data_service.py
  ```
- JSON `{ "payload": "air@23.5@70.1@150@420" }` 또는 단순 텍스트 바디를 받아 위 테이블과 동일한 스키마로 저장합니다.
