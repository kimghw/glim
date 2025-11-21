# Ouster 라이다 설정 및 사용 가이드

이 폴더는 Ouster OS-1-32 라이다의 네트워크 설정, ROS2 드라이버 실행, 데이터 녹화를 위한 모든 스크립트와 설정 파일을 포함합니다.

## 📁 폴더 구조

```
ouster_setup/
├── scripts/           # 실행 스크립트
│   ├── set_ouster_static_ip.sh    # 센서 Static IP 설정 스크립트
│   ├── record_ouster.sh           # Rosbag 녹화 스크립트
│   └── install_ros2.sh            # ROS2 Jazzy 설치 스크립트
├── config/            # 설정 파일
│   └── ouster_params.yaml         # 센서 파라미터 설정
├── launch/            # ROS2 Launch 파일
│   └── ouster_driver.launch.py   # 드라이버 실행 Launch 파일
├── data/              # 데이터 저장 폴더 (rosbag 등)
└── README.md          # 이 파일
```

## 🚀 Quick Start

### 1. 센서 네트워크 설정

센서를 Static IP로 설정합니다 (최초 1회):

```bash
cd /home/kimghw/glim/ouster_setup/scripts
sudo ./set_ouster_static_ip.sh
```

**기본 설정:**
- 센서 IP: `192.168.10.10/24`
- 호스트 IP: `192.168.10.1/24`

**커스텀 IP 설정:**
```bash
sudo ./set_ouster_static_ip.sh 192.168.20.10 192.168.20.1
```

### 2. ROS2 환경 설정

터미널을 열 때마다 실행:

```bash
source /opt/ros/jazzy/setup.bash
```

또는 `.bashrc`에 추가하여 자동화:

```bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
```

### 3. 센서 드라이버 실행

#### 방법 1: ROS2 공식 드라이버 사용

```bash
cd /home/kimghw/glim/ouster_setup
source /opt/ros/jazzy/setup.bash
ros2 launch ouster_ros driver.launch.py \
    params_file:=$(pwd)/config/ouster_params.yaml \
    viz:=True
```

#### 방법 2: 커스텀 Launch 파일 사용

```bash
cd /home/kimghw/glim/ouster_setup
source /opt/ros/jazzy/setup.bash
ros2 launch launch/ouster_driver.launch.py viz:=True
```

### 4. 데이터 녹화

#### 기본 녹화 (Raw packets)

```bash
cd /home/kimghw/glim/ouster_setup/scripts
./record_ouster.sh
```

#### 파일명 및 시간 지정

```bash
# my_data.db3로 60초 녹화
./record_ouster.sh my_data 60

# 모든 토픽 녹화 (포인트클라우드 포함)
./record_ouster.sh my_data 60 --all

# RViz와 함께 녹화
./record_ouster.sh my_data --with-viz
```

**녹화된 데이터 위치:**
`/home/kimghw/glim/rosbag_data/` 또는 `./data/`

### 5. 데이터 재생

#### Raw packets 재생

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch ouster_ros replay.launch.xml \
    bag_file:=/home/kimghw/glim/rosbag_data/my_data \
    metadata:=/home/kimghw/glim/rosbag_data/my_data/metadata.json \
    viz:=true
```

#### 전체 토픽 재생

```bash
ros2 bag play /home/kimghw/glim/rosbag_data/my_data
```

## ⚙️ 센서 설정

### 센서 정보
- **모델**: Ouster OS-1-32-U2-SR
- **Serial Number**: 122413001532
- **펌웨어**: v3.1.0
- **기본 IP**: 192.168.10.10 (Static)

### 파라미터 설정 ([config/ouster_params.yaml](config/ouster_params.yaml))

주요 파라미터:
- `sensor_hostname`: 센서 IP 주소
- `lidar_mode`: 해상도 및 속도 (2048x10, 1024x20 등)
- `timestamp_mode`: 타임스탬프 모드
- `lidar_port`: 7502
- `imu_port`: 7503

수정 후 드라이버 재시작 필요.

## 📊 ROS2 토픽

드라이버 실행 시 발행되는 토픽:

### Raw Data (권장 녹화)
- `/ouster/imu_packets` - IMU raw packets
- `/ouster/lidar_packets` - Lidar raw packets
- `/ouster/metadata` - 센서 메타데이터

### Processed Data
- `/ouster/points` - PointCloud2 (포인트클라우드)
- `/ouster/imu` - IMU 데이터
- `/ouster/scan` - LaserScan
- `/ouster/range_image` - 거리 이미지
- `/ouster/reflec_image` - 반사율 이미지
- `/ouster/signal_image` - 신호 강도 이미지
- `/ouster/nearir_image` - 근적외선 이미지

### 토픽 확인

```bash
# 모든 토픽 리스트
ros2 topic list

# 특정 토픽 확인
ros2 topic echo /ouster/points

# 토픽 발행 주파수 확인
ros2 topic hz /ouster/points
```

## 🛠️ 문제 해결

### 센서 연결 확인

```bash
ping 192.168.10.10
curl http://192.168.10.10/api/v1/sensor/metadata
```

### Static IP 초기화

센서를 DHCP/Link-local로 되돌리기:

```bash
# 센서의 현재 호스트명 또는 IP로
curl -X DELETE http://os-122413001532.local/api/v1/system/network/ipv4/override
```

### 호스트 IP 확인

```bash
ip addr show enxf8e43b49701e | grep "inet "
```

### ROS2 환경 확인

```bash
ros2 pkg list | grep ouster
ros2 doctor --report
```

## 📚 참고 자료

- [Ouster ROS2 공식 문서](https://github.com/ouster-lidar/ouster-ros)
- [Ouster 센서 문서](https://static.ouster.dev/sensor-docs/)
- [ROS2 Jazzy 문서](https://docs.ros.org/en/jazzy/)

## 🔧 스크립트 상세 설명

### set_ouster_static_ip.sh
- Link-local IP로 센서 자동 검색
- Static IP 자동 설정
- 호스트 네트워크 자동 구성
- sudo 권한 필요

### record_ouster.sh
- Rosbag 녹화 자동화
- Raw packets 또는 전체 토픽 선택
- 시간 제한 녹화 지원
- 자동 파일명 생성

### install_ros2.sh
- ROS2 Jazzy 자동 설치 (Ubuntu 24.04)
- Ouster 드라이버 의존성 포함
- 개발 도구 자동 설치

## 📝 라이선스

각 스크립트는 Ouster 공식 라이선스를 따릅니다.
