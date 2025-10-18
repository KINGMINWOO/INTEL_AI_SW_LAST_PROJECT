# 프로젝트 구조 및 실행 가이드

이 문서는 Intel Final Project 저장소(1.Server 디렉터리)를 기준으로 현재 구현된 실시간 스트리밍 및 로봇 제어 스택을 빠르게 파악하고 실행하기 위한 가이드입니다. 각 섹션은 최신 코드 흐름과 환경 설정을 반영하며, 실제 장비/시뮬레이터에 맞춰 추가 조정할 수 있습니다.

## 1. 전체 개요
- **핵심 기능**: YOLOv8 기반 토마토 감지, TurtleBot Nav2 이동/라인트레이서 연계, CCTV·센서 데이터 수집, MJPEG 웹 스트리밍, MariaDB 로깅.
- **주요 모듈**
  - `server_highres_output.py`: 다중 소켓 수신, 인증, YOLO 추론, Nav2 연동, 자동 정지/로봇팔 명령, 센서 로깅, Flask 스트리밍.
  - `nav2_bridge.py`: ROS2 Nav2 NavigateToPose/Spin 액션을 동기 호출로 감싼 헬퍼.
  - `farm_storage.py`: SQLAlchemy 기반 MariaDB ORM 헬퍼.
  - `farm_data_service.py`: HTTP 수집 엔드포인트(`/api/farm-data`)로 외부 센서 데이터를 받아 저장.
  - `client_turtle.py`, `client_cctv.py`, `client_low_buff.py`, `client_jpeg.py`, `client_user.py`: 각각 TurtleBot, CCTV, 사용자 콘솔 전용 클라이언트.

## 2. 개발 환경 준비
### 2.1 Python/YOLO 환경
1. Python 3.10 이상 권장, 가상환경 생성:
   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   pip install --upgrade pip
   pip install -r requirements.txt
   ```
   - `requirements.txt`에는 `ultralytics`, `torch==2.5.1+cu121`, `opencv-python`, `Flask`, `SQLAlchemy`, `PyMySQL`, `pyserial`가 포함됩니다.
   - GPU가 없는 환경이라면 알맞은 Torch Wheel을 수동 설치 후 `pip install -r requirements.txt --no-deps`로 의존성 충돌을 피하세요.
2. YOLO 추론 가중치 `best.pt`를 프로젝트 루트(`1.Server`)에 보관합니다. 다른 모델을 사용할 경우 `server_highres_output.py`의 로드 경로를 수정하세요.

### 2.2 ROS 2 / Nav2 환경
TurtleBot3 (ROS 2 Humble) 기준 패키지:
```bash
sudo apt update
sudo apt install ros-humble-desktop \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-navigation2 \
    ros-humble-navigation2 ros-humble-nav2-bringup
```
- ROS 2를 사용하지 않는 워크스테이션에서도 `nav2_bridge` 임포트는 실패해도 전체 서버가 동작하도록 예외 처리가 되어 있습니다. 다만 Nav2 이동/정렬 기능은 비활성화됩니다.
- 라인트레이서 스크립트는 `../3.Robot/1.ROS/line_tracer_remote.py`를 호출하므로 해당 리포지터리 경로와 Python 의존성을 확인하세요.

### 2.3 MariaDB (선택)
센서/감지 로그를 DB에 저장하려면 MariaDB와 계정을 준비합니다.
```bash
sudo apt install mariadb-server
sudo mysql_secure_installation
mysql -u root -p
> CREATE DATABASE smart_farm DEFAULT CHARACTER SET utf8mb4;
> CREATE USER 'user01'@'%' IDENTIFIED BY 'user1234';
> GRANT ALL PRIVILEGES ON smart_farm.* TO 'user01'@'%';
```
- 기본 연결 문자열은 `mysql+pymysql://user01:user1234@10.10.16.29:3306/smart_farm`입니다.
- 다른 환경에서는 `FARM_DB_URL` 환경 변수를 설정해 덮어씁니다.

## 3. 시뮬레이션 및 Nav2 구성
실제 TurtleBot 대신 Gazebo를 사용한 검증 절차:
1. **Gazebo 시뮬레이터**
   ```bash
   source /opt/ros/humble/setup.bash
   export TURTLEBOT3_MODEL=burger
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py headless:=True
   ```
2. **Nav2 Bringup**
   ```bash
   source /opt/ros/humble/setup.bash
   export TURTLEBOT3_MODEL=burger
   ros2 launch turtlebot3_navigation2 navigation2.launch.py \
       use_sim_time:=True \
       map:=/opt/ros/humble/share/turtlebot3_navigation2/map/map.yaml
   ```
3. **RViz 초기 위치/목표 지정**
   ```bash
   source /opt/ros/humble/setup.bash
   rviz2 -d /opt/ros/humble/share/turtlebot3_navigation2/rviz/turtlebot3_nav2.rviz
   ```
   - `/map` 프레임이 활성화된 것을 확인하고 `2D Pose Estimate` → `2D Goal Pose` 순으로 초기화를 진행합니다.

`server_highres_output.py` 기동 시 Nav2 액션 서버에 접속 가능한 경우 `Nav2 브리지를 초기화했습니다.` 로그가 출력됩니다. 실패하더라도 감지/스트리밍 기능은 유지됩니다.

## 4. 서버 구성 요소
### 4.1 `server_highres_output.py`
- 9999 포트에서 TCP 소켓 서버를 열어 각 클라이언트를 `idlist.txt` 기반으로 인증 (`AUTH_REQUEST`/`AUTH_OK`).
- `TURTLE*` 클라이언트의 MJPEG 프레임에 YOLOv8 추론을 수행하고 `ripe`/`rotten` 박스를 검출합니다.
- 감지 중앙값이 화면 가운데에 3초 내 5회 이상 등장하면 자동으로 Nav2를 취소하고 `robot@high/middle/low` 명령을 전송한 뒤 정밀 정렬을 진행합니다.
- 사용자 채널(`USERxx`) 명령을 받아 `ripe@go`, `rotten@go`, `dump@done`, `turtle@go` 큐 등을 호출하며, Nav2와 수동 이동(`MOVE:...`) 명령을 모두 지원합니다.
- CCTV 스트림의 센서 문자열(`air@`, `land@`)은 `farm_storage.FarmDataLogger`를 통해 MariaDB 테이블(`farm_air_samples`, `farm_land_samples`)에 적재하거나, DB 연결이 없으면 로그만 남깁니다.
- `Flask`를 사용해 `http://<server>:5000/video_feed/<client_id>` 경로로 최근 프레임을 MJPEG로 제공하며, `/` 루트 페이지에서 접속 가능한 클라이언트 목록을 확인할 수 있습니다.
- 실행 예:
  ```bash
  source /opt/ros/humble/setup.bash  # Nav2가 필요한 경우
  export FARM_DB_URL='mysql+pymysql://user01:user1234@10.10.16.29:3306/smart_farm'
  python3 server_highres_output.py
  ```
- YOLO 가중치(`best.pt`)와 `idlist.txt`가 동일 디렉터리에 존재해야 하며, Nav2가 필요 없으면 ROS 환경 변수는 생략 가능합니다.

### 4.2 `nav2_bridge.py`
- ROS2 `NavigateToPose`와 `Spin` 액션을 동기 형태로 호출하는 래퍼입니다.
- 요청/응답을 별도 스레드에서 실행하며, 타임아웃 시 자동으로 목표를 취소합니다.
- 서버에서 자동 정렬(Spin) 또는 Nav2 이동이 필요할 때만 사용됩니다.

### 4.3 `farm_storage.py`
- SQLAlchemy가 설치되어 있고 `FARM_DB_URL`이 제공된 경우 MariaDB 엔진을 초기화합니다.
- `FarmDataLogger`는 공기/토양 센서와 토마토 스냅샷을 각각 `farm_air_samples`, `farm_land_samples`, `farm_tomato_snapshots` 테이블에 저장합니다.
- DB 연결이 없을 때는 로그만 남겨 흐름을 추적할 수 있도록 설계됐습니다.

### 4.4 `farm_data_service.py`
- 외부 장치가 HTTP POST `/api/farm-data`로 `{"payload": "air@24.3@68@120@350"}` 또는 순수 텍스트를 보내면 파싱 후 `farm_samples` 테이블에 적재합니다.
- 지표 이름은 `farm_metrics.json`에서 동적으로 로드하며, 없는 경우 기본 필드(`temperature_c`, `humidity_pct`, ...)를 사용합니다.
- 실행 예:
  ```bash
  FARM_DB_URL='mysql+pymysql://user01:user1234@10.10.16.29:3306/smart_farm' \
    python3 farm_data_service.py
  ```

## 5. 클라이언트 구성 요소
### 5.1 `client_turtle.py`
- TurtleBot 카메라 영상을 MJPEG로 전송하고, ROS2 오도메트리(`/odom`)를 구독해 현재 위치/방향을 서버에 주기적으로 보고합니다.
- `MotionController`가 `MOVE:` 명령을 해석해 상대 좌표로 주행하며, `TURN:` 명령(선속도, 각속도 순)을 받아 수동 속도 제어를 수행합니다.
- 기본 서버 주소는 `("127.0.0.1", 9999)`이므로 실제 서버 IP로 수정하고, `CLIENT_ID`, `CLIENT_PASSWORD`가 `idlist.txt`와 일치하는지 확인하세요.
- 실행 전 `source /opt/ros/humble/setup.bash`로 ROS 환경을 활성화해야 합니다.

### 5.2 `client_cctv.py`
- `/dev/video0`에서 프레임을 10 FPS MJPEG로 인코딩해 송신하며, 표준 입력에 입력된 `air@...`/`land@...` 데이터를 5초 간격으로 전송합니다.
- 서버에서 `[CCTV1]LED@HIGH` 등으로 전달된 메시지를 UART3(`pyserial`)로 포워딩해 외부 MCU와 연동합니다.
- 기본 서버 주소는 `10.10.16.29:9999`이므로 테스트 환경에서는 적절히 변경합니다.

### 5.3 `client_low_buff.py`
- 저지연 스트리밍을 위한 CCTV 변형입니다. 프레임 버퍼를 버리고 즉시 캡처하는 방식으로 지연을 줄입니다.
- `TARGET_FPS`, `FRAME_SIZE`, `JPEG_QUALITY`로 네트워크 대역폭을 조절할 수 있습니다.

### 5.4 `client_jpeg.py`
- 해상도를 2952x1944로 고정한 고해상도 테스트용 CCTV 클라이언트입니다.
- 서버 메시지를 별도 스레드로 출력하며, 인증 절차는 다른 클라이언트와 동일합니다.

### 5.5 `client_user.py`
- 영상 없이 명령만 전송하는 사용자 콘솔입니다. `USERxx` 계정으로 로그인한 뒤 `ripe@go`, `stop`, `[CCTV01]LED@LOW` 등 텍스트 명령을 전송합니다.
- 기본 서버 주소는 `127.0.0.1:9999`이므로 외부 접속 시 IP를 수정하세요.

## 6. 스마트팜 데이터 및 데이터베이스
- `server_highres_output.py`는 다음 테이블을 사용합니다.
  - `farm_air_samples`: `air@` 페이로드에서 온도, 습도, 공기질, 조도를 저장.
  - `farm_land_samples`: `land@` 페이로드에서 토양 온도, 습도, EC, pH를 저장.
  - `farm_tomato_snapshots`: `turtle@go` 실행 전 CCTV 스냅샷의 익은/썩은 토마토 개수를 기록.
- 디폴트 접속 문자열은 `mysql+pymysql://user01:user1234@10.10.16.29:3306/smart_farm`이며, 환경변수 `FARM_DB_URL`로 재정의할 수 있습니다.

### 6.1 데이터 확인 예시
```sql
USE smart_farm;
SHOW TABLES;
SELECT * FROM farm_air_samples ORDER BY id DESC LIMIT 5;
SELECT * FROM farm_land_samples ORDER BY id DESC LIMIT 5;
SELECT * FROM farm_tomato_snapshots ORDER BY id DESC LIMIT 5;
```
- CLI에서 `mysql -u user01 -p -h <db-host>`로 접속하거나, SQLAlchemy 기반 스크립트에서 동일한 쿼리를 수행해 검증합니다.

### 6.2 추가 수집 서비스
- `farm_data_service.py`는 REST 방식 수집이 필요할 때 선택적으로 사용합니다.
- POST 예시(기본 포트 8081, `FARM_SERVICE_PORT`로 변경 가능):
  ```bash
  curl -X POST http://<host>:8081/api/farm-data \
       -H "Content-Type: application/json" \
       -d '{"device_id": "CCTV01", "payload": "cctv01@24.1@70.3@120@350"}'
  ```
- 서비스 종료는 `Ctrl+C` 또는 `SIGTERM`으로 처리되며, 구성 변경 시 `farm_metrics.json`을 업데이트한 뒤 재시작합니다.

## 7. 운영 팁
- 인증 정보는 `idlist.txt`의 `id:password` 형식을 사용합니다. 전송 전후로 서버 로그에서 `클라이언트 <addr> 인증 성공` 여부를 확인하세요.
- 새로 학습한 모델을 적용할 때는 `best.pt` 교체와 함께 서버 재기동이 필요합니다.
- Nav2 연동 없이 테스트할 경우 TurtleBot 클라이언트는 `TURN:<linear>,<angular>` 및 `STOP` 명령만으로도 움직일 수 있으므로, `client_user.py`로 직접 명령을 전송해 동작을 확인합니다.
