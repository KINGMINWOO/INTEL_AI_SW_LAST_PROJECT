# 프로젝트 구조 및 실행 가이드

이 문서는 Intel Final Project 저장소의 `Source/1.Server` 디렉터리를 기준으로 실시간 영상 스트리밍·토마토 감지·TurtleBot 제어 스택을 신속하게 파악하고 실행하기 위한 안내서입니다. 최신 코드 흐름과 장비 구성을 반영했으며, 실제 배포 환경에 맞춰 값을 조정할 수 있도록 핵심 설정 지점을 함께 정리했습니다.

## 1. 전체 개요
- **핵심 기능**: YOLOv8 기반 토마토 감지, TurtleBot Nav2 경로 제어와 라인트레이서 연계, CCTV/센서 데이터 수집, MJPEG 웹 스트리밍, MariaDB 로깅 및 REST 수집.
- **주요 모듈**
  - `server.py` (구 `server_highres_output.py`): 멀티 소켓 수신, 인증, YOLO 추론, Nav2 제어, 라인트레이서 명령, 센서 로깅, Flask 스트리밍.
  - `nav2_bridge.py`: ROS2 Nav2 `NavigateToPose` 액션을 비동기 스레드로 감싼 헬퍼.
  - `farm_storage.py`: SQLAlchemy 기반 MariaDB 로거 (`farm_air_samples`, `farm_land_samples`, `farm_tomato_snapshots` 관리).
  - `farm_data_service.py`: HTTP POST(`/api/farm-data`)로 추가 센서 데이터를 받아 저장하는 Flask 서비스.
  - `client_cctv.py`: CCTV 영상 및 센서 업링크 + UART3 포워딩.
  - `client_turtle_with_sensor.py`: TurtleBot 영상 전송 + ROS2 오도메트리/라인센서 + Arduino 브리지(라인 트레이싱 로직 포함).
  - `client_user.py`: 사용자 명령 콘솔. (레거시 분기에서는 `client_jpeg.py`, `client_low_buff.py`, `client_turtle.py`가 존재할 수 있습니다.)

## 2. 개발 환경 준비
### 2.1 Python/YOLO 환경
1. Python 3.10 이상 권장, 가상환경 생성:
   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   pip install --upgrade pip
   pip install -r requirements.txt
   ```
   - `requirements.txt`에는 `ultralytics`, `torch==2.5.1+cu121`, `opencv-python`, `Flask`, `SQLAlchemy`, `PyMySQL`, `pyserial` 등이 포함됩니다.
   - GPU가 없는 환경이라면 알맞은 Torch wheel을 수동 설치한 뒤 `pip install -r requirements.txt --no-deps`로 의존성 충돌을 피하세요.
2. YOLO 추론 가중치 `best.pt`를 `Source/1.Server` 루트에 보관합니다. 다른 모델을 사용할 경우 `server.py` 상단의 로드 경로를 수정합니다.

### 2.2 ROS 2 / Nav2 환경
TurtleBot3 (ROS 2 Humble) 기준 패키지 설치:
```bash
sudo apt update
sudo apt install ros-humble-desktop \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-navigation2 \
    ros-humble-navigation2 ros-humble-nav2-bringup
```
- ROS 2가 없는 워크스테이션에서도 서버는 동작하지만 Nav2 이동/정렬이 비활성화됩니다.
- 라인트레이서 호출은 `../3.Robot/1.ROS/line_tracer_remote.py`(외부 저장소) 경로를 사용하므로 로컬 경로가 다르면 스크립트 내부를 수정하세요.

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
- 기본 연결 문자열은 `mysql+pymysql://user01:user1234@192.168.0.4:3306/smart_farm`.
- 다른 환경에서는 `FARM_DB_URL` 환경 변수를 사용하거나 `server.py`의 기본값을 수정하세요.

### 2.4 하드웨어 의존 패키지
- Raspberry Pi에서 UART/GPIO를 사용하려면 `sudo apt install python3-serial python3-rpi.gpio`를 수행하고, `dtoverlay=uart3` 설정 후 재부팅합니다.
- Arduino와 USB 직렬 통신을 활용하려면 `pip install pyserial`.
- ROS2가 동작하지 않는 장비에서는 `client_turtle_with_sensor.py`가 자동으로 ROS 기능을 비활성화합니다.

## 3. 빠른 실행 절차
1. 저장소 루트에서 가상환경을 활성화하고 필수 패키지를 설치합니다. 배포 전 간단한 구문 검사를 위해:
   ```bash
   python3 -m compileall server.py client_cctv.py client_turtle_with_sensor.py client_user.py
   ```
2. (선택) DB, Nav2, 외부 서비스 주소를 환경 변수로 지정합니다.
   ```bash
   export FARM_DB_URL="mysql+pymysql://user01:user1234@<db-host>:3306/smart_farm"
   export FARM_SERVICE_HOST="0.0.0.0"
   export FARM_SERVICE_PORT="8081"
   ```
3. 서버 실행:
   ```bash
   python3 server.py
   ```
   - 파일 상단 주석은 `server_highres_output.py`로 되어 있으나, 현재 저장소에서는 `server.py`가 메인 엔트리입니다.
   - 9999/TCP로 소켓을 열고, 5000/TCP Flask 앱을 실행합니다. 콘솔에 `Nav2 브리지를 초기화했습니다.` 로그가 나오면 Nav2 연결 성공입니다.
4. CCTV 클라이언트(영상 + 센서) 예시:
   ```bash
   python3 client_cctv.py
   ```
   - 스크립트 상단의 `CLIENT_ID`, `CLIENT_PASSWORD`, `server_ip`를 현재 서버 환경에 맞게 수정하세요.
   - 터미널에서 `air@24.2@68@115@360`처럼 입력하면 5초 간격으로 센서 데이터가 전송됩니다.
5. TurtleBot 클라이언트:
   ```bash
   python3 client_turtle_with_sensor.py
   ```
   - `SERVER_ADDR`, `CLIENT_ID`, `CLIENT_PASSWORD`를 수정하고, 실행 전 `source /opt/ros/humble/setup.bash`로 ROS 환경을 활성화합니다.
6. 사용자 콘솔:
   ```bash
   python3 client_user.py
   ```
   - `USERxx` 계정으로 로그인한 뒤 `ripe@go`, `stop`, `[CCTV01]LED@LOW` 등 명령을 테스트합니다.
7. 웹 스트림 확인: 브라우저에서 `http://<server-ip>:5000/video_feed/<client_id>` 경로를 열어 각 클라이언트의 MJPEG 스트림을 확인합니다.

## 4. 시뮬레이션 및 Nav2 구성
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

`server.py` 실행 시 Nav2 액션 서버와 연결되면 `Nav2 브리지를 초기화했습니다.` 로그가 출력됩니다. 실패하더라도 감지·스트리밍 기능은 정상 동작합니다.

## 5. 서버 구성 요소
### 5.1 `server.py` (Flask + 소켓 허브)
- 9999 포트에서 TCP 소켓 서버를 열어 각 클라이언트를 `idlist.txt` 기반으로 인증합니다 (`AUTH_REQUEST` → `id:password` → `AUTH_OK`).
- `TURTLE*` 계열 스트림에 YOLOv8 추론을 적용하고 `ripe`/`rotten` 감지 박스를 계산합니다.
- 감지 중앙값이 화면 가운데에 지정 시간 이상 머무르면 Nav2 목표를 취소하고 `robot@high/middle/low` 명령을 송신한 뒤 정렬 루틴을 실행합니다.
- 사용자 채널(`USERxx`)에서 수신한 명령(`ripe@go`, `dump@done`, `turtle@go`, `MOVE:0.1,0`)을 큐로 관리해 Nav2/라인트레이서 호출을 스케줄링합니다.
- `air@`, `land@`와 같은 센서 문자열을 `farm_storage.FarmDataLogger`를 통해 MariaDB에 저장하며, DB 연결이 없을 때는 콘솔 로그로 대체합니다.
- Flask 앱은 `/video_feed/<client_id>`로 MJPEG 스트림을 제공하고 `/status` JSON 엔드포인트를 통해 연결 현황을 노출합니다.

### 5.2 Nav2 브리지 (`nav2_bridge.py`)
- ROS2 `rclpy`를 사용해 Nav2 액션(`NavigateToPose`)을 비동기로 호출하고, 목표 취소와 스핀 동작을 제공합니다.
- `create_nav2_bridge()`가 성공하면 별도의 스레드에서 목표 큐를 소비하며, 실패 시 서버는 Nav2 기능 없이 실행됩니다.

### 5.3 농가 데이터 로거 (`farm_storage.py`)
- `FarmDataLogger`가 공기·토양 센서와 토마토 스냅샷을 각각 `farm_air_samples`, `farm_land_samples`, `farm_tomato_snapshots` 테이블에 저장합니다.
- DB 연결이 실패하면 예외를 무시하고 콘솔 로그만 남기도록 설계되었습니다. 배포 환경에서는 반드시 DB 로그를 확인하세요.

### 5.4 REST 수집 서비스 (`farm_data_service.py`)
- 외부 장치가 HTTP POST `/api/farm-data`로 `{"payload":"air@24.3@68@120@350"}` 또는 단순 텍스트를 전송하면 파싱 후 `farm_samples` 테이블에 적재합니다.
- 지표 이름은 `farm_metrics.json`에서 동적으로 로드하며, 파일이 없으면 기본 필드(`temperature_c`, `humidity_pct`, ...)를 사용합니다.
- 예시:
  ```bash
  FARM_DB_URL='mysql+pymysql://user01:user1234@192.168.0.4:3306/smart_farm' \
    python3 farm_data_service.py
  ```
  `curl -X POST http://localhost:8081/api/farm-data -H "Content-Type: application/json" -d '{"payload":"cctv01@24.1@70.3@120@350"}'`

## 6. 클라이언트 구성 요소
### 6.1 `client_turtle_with_sensor.py`
- TurtleBot 영상(MJPG) 전송, ROS2 오도메트리 구독, 라인 센서 GPIO 읽기, Arduino 직렬 통신을 통합한 클라이언트입니다. 라인트레이싱 알고리즘도 이 스크립트에 포함되어 `TRACE_LINE` 명령만 수신하면 센서 기반 주행을 완료합니다.
- `SERVER_ADDR`, `CLIENT_ID`, `CLIENT_PASSWORD`, `TARGET_FPS`, `FRAME_SIZE` 등을 수정해 배포 환경에 맞춥니다.
- ROS2가 준비되지 않은 경우 자동으로 메시지를 출력하고 ROS 기능 없이 동작합니다.
- Arduino와 연결되면 `🔌 Arduino 브리지 시작` 로그가 출력되며, 서버에서 전달된 명령은 `write()`를 통해 Arduino로 전송됩니다.

### 6.2 `client_cctv.py`
- `/dev/video0` 영상을 10FPS MJPG로 인코딩해 송신하고, 표준 입력에서 `air@...`/`land@...` 데이터를 입력하면 5초 간격으로 서버에 전송합니다.
- 서버 메시지 중 `[CCTV1]` 접두사는 UART3로 전달되어 외부 MCU와 연동됩니다. UART가 없으면 경고만 출력되고 계속 동작합니다.
- 실행 전 스크립트 상단의 서버 주소와 인증 정보를 실제 값으로 수정하세요.

### 6.3 `client_user.py`
- 영상 없이 명령만 전송하는 사용자 콘솔입니다. `USERxx` 계정으로 로그인해 `ripe@go`, `stop`, `[CCTV01]LED@LOW` 등을 실시간으로 전송하며 서버 로그를 확인합니다.
- `server_ip`, `server_port`, `CLIENT_ID`, `CLIENT_PASSWORD`가 소스 내에 하드코딩되어 있으므로 배포 시 반드시 값이 일치하도록 관리하세요.

### 6.4 레거시 클라이언트
- 과거 브랜치에는 `client_turtle.py`, `client_low_buff.py`, `client_jpeg.py`가 존재하며, 동일한 인증 프로토콜을 공유합니다.
- 해당 파일을 사용하는 경우 현재 가이드의 설정(포트, 인증, 명령 흐름)을 동일하게 적용하면 호환됩니다.
- 레거시 워크플로우에서 사용되던 ROS 전용 라인트레이서(`../3.Robot/1.ROS/line_tracer_remote.py`)는 현재 파이프라인에서 필수가 아니며, `client_turtle_with_sensor.py`가 라인트레이싱을 내장하고 있습니다. 별도의 ROS 노드로 라인트레이싱을 실험해야 할 때만 활용하세요.

## 7. 스마트팜 데이터 및 데이터베이스
- `server.py`는 다음 테이블을 사용합니다.
  - `farm_air_samples`: `air@` 페이로드에서 온도, 습도, 공기질, 조도를 저장.
  - `farm_land_samples`: `land@` 페이로드에서 토양 온도, 습도, EC, pH를 저장.
  - `farm_tomato_snapshots`: `turtle@go` 실행 전 CCTV 스냅샷의 익은/썩은 토마토 개수를 기록.
- `farm_data_service.py`는 `farm_samples` 테이블을 사용하며, metric 이름은 `farm_metrics.json`으로 관리할 수 있습니다.
- 기본 접속 문자열은 `mysql+pymysql://user01:user1234@192.168.0.4:3306/smart_farm`이며, 환경 변수 `FARM_DB_URL`로 재정의 가능합니다.

### 7.1 데이터 확인 예시
```sql
USE smart_farm;
SHOW TABLES;
SELECT * FROM farm_air_samples ORDER BY id DESC LIMIT 5;
SELECT * FROM farm_land_samples ORDER BY id DESC LIMIT 5;
SELECT * FROM farm_tomato_snapshots ORDER BY id DESC LIMIT 5;
SELECT * FROM farm_samples ORDER BY id DESC LIMIT 5;
```
- CLI에서 `mysql -u user01 -p -h <db-host>`로 접속하거나, SQLAlchemy 세션을 이용해 동일한 쿼리를 수행해 검증합니다.

### 7.2 추가 수집 서비스
- `farm_data_service.py`는 REST 방식 수집이 필요할 때 선택적으로 사용합니다.
- 서비스 종료는 `Ctrl+C` 또는 `SIGTERM`으로 처리되며, 구성 변경 시 `farm_metrics.json`을 업데이트한 뒤 `/api/farm-metrics/reload` 호출로 즉시 반영할 수 있습니다.

## 8. 모델 학습 및 업데이트
- 학습용 스크립트: `train_yolov8.py`
  ```bash
  python3 train_yolov8.py --data tomato_yolo_prepared.yaml --epochs 100 --img 640
  ```
  - 데이터셋은 `classification_dataset_prepared/`, `tomato_geti_dataset/`, `yolo_dataset_prepared/` 디렉터리 구조(ultralytics 표준)를 따릅니다.
  - 학습 결과는 기본적으로 `runs/detect/exp*`에 저장됩니다.
- 새 모델을 적용하려면 `best.pt`를 교체하고 서버를 재기동합니다. 모델 경로가 달라지면 `server.py` 상단의 `YOLO('best.pt')` 부분을 수정하세요.
- YOLOv5 학습이 필요하면 하위 디렉터리의 `yolov5/` 스크립트를 활용하고, 벤더 가이드를 참고해 동일한 데이터셋 구조를 사용합니다.

## 9. 테스트 & 검증 체크리스트
- 구문 검증: `python3 -m compileall server.py client_cctv.py client_turtle_with_sensor.py client_user.py`
- 서버 기동 후 각 클라이언트를 연결해 다음을 확인합니다.
  - 인증 성공 로그: `클라이언트 <addr> 인증 성공`.
  - `/video_feed/<client_id>`에서 MJPEG 스트림 재생.
  - `ripe`/`rotten` 감지 시 Nav2 취소 및 로봇 명령 전송.
  - `air@`/`land@` 페이로드가 DB 테이블에 적재되는지 확인 (`SELECT * FROM farm_air_samples ...`).
- Nav2 미연결 환경에서는 `client_user.py`로 `TURN:0.1,-0.2`, `STOP` 명령을 테스트해 안전 장치가 동작하는지 확인합니다.
- `farm_data_service.py` 사용 시 `curl`로 샘플 데이터를 전송하고 200 응답·DB 적재 여부를 검증합니다.

## 10. 트러블슈팅
- **Nav2 브리지 초기화 실패**: ROS 환경 설정(`source /opt/ros/humble/setup.bash`)이 누락되었거나 액션 서버가 열려 있지 않습니다. Gazebo/실기에서 `ros2 action list`로 상태를 확인하세요.
- **Torch CUDA 오류**: GPU 드라이버와 CUDA 버전을 점검하거나, `export ULTRALYTICS_PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:256` 같은 메모리 설정을 적용합니다.
- **DB 연결 실패**: `FARM_DB_URL`을 확인하고, 방화벽/포트 허용 여부를 점검합니다. SQLAlchemy가 설치되지 않은 경우 로컬 로그 모드로 동작합니다.
- **UART3 미연결**: Raspberry Pi에서 `dtoverlay=uart3` 설정과 권한을 확인한 뒤 `/dev/ttyAMA*` 존재 여부를 점검하세요.
- **Flask 스트림이 끊김**: 네트워크 지연이 큰 경우 `client_*` 스크립트의 `TARGET_FPS`와 `JPEG_QUALITY`를 조정해 대역폭을 낮춥니다.

## 11. 운영 체크리스트 & 팁
- 인증 정보는 `idlist.txt`(`id:password`)로 관리하며, 변경 시 서버·클라이언트 파일을 동시에 업데이트합니다. 정상 로그인 시 서버 로그에 `AUTH_OK` 메시지가 출력됩니다.
- 새 모델 또는 설정 파일을 배포한 뒤에는 서버를 재기동하고 `/status` 엔드포인트로 연결 상태를 확인하세요.
- Nav2 없이 테스트할 때는 TurtleBot 클라이언트의 `TURN:<linear>,<angular>` 및 `STOP` 명령만으로도 기본 동작을 검증할 수 있습니다.
- 실장비 배포 전에는 Gazebo 시뮬레이션에서 전체 명령 시퀀스(`turtle@go`)를 재현해 자동 정렬과 DB 로깅이 함께 동작하는지 확인하세요.
- 외부 공개 전에는 `idlist.txt`, DB 자격 증명, `best.pt`를 모두 교체하고, 서버 로그에 남는 민감 정보를 점검한 뒤 배포하세요.
