# 🚀 Ouster 라이다 빠른 시작 가이드

## 1️⃣ 센서 네트워크 설정 (최초 1회)

```bash
cd /home/kimghw/glim/ouster_setup/scripts
sudo ./set_ouster_static_ip.sh
```

센서 IP: `192.168.10.10`, 호스트 IP: `192.168.10.1`

---

## 2️⃣ 드라이버 실행

```bash
cd /home/kimghw/glim/ouster_setup/scripts
./run_driver.sh
```

RViz와 함께 센서 드라이버가 실행됩니다.

---

## 3️⃣ 데이터 녹화

### 기본 녹화 (무제한)
```bash
cd /home/kimghw/glim/ouster_setup/scripts
./record_ouster.sh
```

### 60초 녹화
```bash
./record_ouster.sh my_data 60
```

### 녹화 중지
`Ctrl + C`

---

## 4️⃣ 데이터 재생

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch ouster_ros replay.launch.xml \
    bag_file:=/home/kimghw/glim/rosbag_data/my_data \
    viz:=true
```

---

## 📁 주요 파일

- **네트워크 설정**: `scripts/set_ouster_static_ip.sh`
- **드라이버 실행**: `scripts/run_driver.sh`
- **데이터 녹화**: `scripts/record_ouster.sh`
- **센서 설정**: `config/ouster_params.yaml`
- **자세한 설명**: `README.md`

---

## 🔍 문제 해결

### 센서 연결 확인
```bash
ping 192.168.10.10
```

### ROS2 토픽 확인
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
```

자세한 내용은 [README.md](README.md)를 참조하세요.
