#!/bin/bash

# ROSBAG 포인트클라우드 전용 플레이어
# 포인트클라우드 토픽만 필터링하여 재생

set -e

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# rosbag 디렉토리 설정 (여러 경로 검색)
ROSBAG_DIRS=(
    "/home/kimghw/rosbag"
    "/home/kimghw/lidarslamlecture/data"
    "/home/kimghw"
)

echo -e "${BLUE}=== ROSBAG 포인트클라우드 플레이어 ===${NC}"
echo ""

# rosbag 파일 찾기
echo -e "${YELLOW}rosbag 파일 검색 중...${NC}"
rosbags=()
for dir in "${ROSBAG_DIRS[@]}"; do
    if [ -d "$dir" ]; then
        while IFS= read -r file; do
            rosbags+=("$file")
        done < <(find "$dir" -maxdepth 4 \( -name "*.db3" -o -name "*.mcap" \) ! -path "*/.*" 2>/dev/null)
    fi
done

# 중복 제거 및 정렬
mapfile -t rosbags < <(printf '%s\n' "${rosbags[@]}" | sort -u)

if [ ${#rosbags[@]} -eq 0 ]; then
    echo -e "${RED}rosbag 파일을 찾을 수 없습니다.${NC}"
    exit 1
fi

# rosbag 선택 메뉴
echo -e "${GREEN}사용 가능한 rosbag 파일:${NC}"
for i in "${!rosbags[@]}"; do
    filepath="${rosbags[$i]}"
    filename=$(basename "$filepath")
    dirname=$(dirname "$filepath")
    size=$(du -h "$filepath" 2>/dev/null | cut -f1)
    echo -e "  ${GREEN}$((i+1))${NC}) $filename (${size}) - ${dirname}"
done

echo ""
read -p "재생할 rosbag 번호를 선택하세요 (1-${#rosbags[@]}): " selection

if ! [[ "$selection" =~ ^[0-9]+$ ]] || [ "$selection" -lt 1 ] || [ "$selection" -gt ${#rosbags[@]} ]; then
    echo -e "${RED}잘못된 선택입니다.${NC}"
    exit 1
fi

SELECTED_BAG="${rosbags[$((selection-1))]}"
echo ""
echo -e "${GREEN}선택된 파일: $(basename "$SELECTED_BAG")${NC}"

# rosbag 정보에서 포인트클라우드 토픽 찾기
echo ""
echo -e "${YELLOW}포인트클라우드 토픽 검색 중...${NC}"

# 포인트클라우드 관련 토픽 찾기
POINTCLOUD_TOPICS=$(ros2 bag info "$SELECTED_BAG" 2>/dev/null | grep "sensor_msgs/msg/PointCloud2" | awk -F'Topic: ' '{print $2}' | awk '{print $1}' | sort -u)

if [ -z "$POINTCLOUD_TOPICS" ]; then
    echo -e "${RED}포인트클라우드 토픽을 찾을 수 없습니다.${NC}"
    echo "전체 토픽 목록을 확인하시겠습니까? (y/n)"
    read -r show_all
    if [ "$show_all" = "y" ]; then
        ros2 bag info "$SELECTED_BAG"
    fi
    exit 1
fi

echo -e "${GREEN}발견된 포인트클라우드 토픽:${NC}"
echo "$POINTCLOUD_TOPICS" | nl -w2 -s') '

# 토픽 선택
TOPIC_ARRAY=($POINTCLOUD_TOPICS)
if [ ${#TOPIC_ARRAY[@]} -eq 1 ]; then
    SELECTED_TOPIC="${TOPIC_ARRAY[0]}"
    TOPIC_FILTER="--topics $SELECTED_TOPIC /tf /tf_static"
    echo -e "${GREEN}자동 선택: $SELECTED_TOPIC${NC}"
else
    echo ""
    read -p "재생할 토픽 번호를 선택하세요 (1-${#TOPIC_ARRAY[@]}, 0=전체): " topic_selection

    if [ "$topic_selection" = "0" ]; then
        TOPIC_FILTER=""
        echo -e "${GREEN}모든 포인트클라우드 토픽 재생${NC}"
    elif [[ "$topic_selection" =~ ^[0-9]+$ ]] && [ "$topic_selection" -ge 1 ] && [ "$topic_selection" -le ${#TOPIC_ARRAY[@]} ]; then
        SELECTED_TOPIC="${TOPIC_ARRAY[$((topic_selection-1))]}"
        TOPIC_FILTER="--topics $SELECTED_TOPIC /tf /tf_static"
        echo -e "${GREEN}선택된 토픽: $SELECTED_TOPIC${NC}"
    else
        echo -e "${RED}잘못된 선택입니다.${NC}"
        exit 1
    fi
fi

# 재생 속도 설정
echo ""
echo -e "${YELLOW}재생 속도 설정:${NC}"
echo "  1) 정상 속도 (1.0x)"
echo "  2) 느린 재생 (0.5x)"
echo "  3) 빠른 재생 (2.0x)"
echo "  4) 사용자 정의"
read -p "선택 (1-4): " speed_choice

case $speed_choice in
    1) RATE="1.0" ;;
    2) RATE="0.5" ;;
    3) RATE="2.0" ;;
    4)
        read -p "재생 속도 입력 (예: 0.1, 1.5, 3.0): " RATE
        if ! [[ "$RATE" =~ ^[0-9]+(\.[0-9]+)?$ ]]; then
            echo -e "${RED}잘못된 속도 값입니다.${NC}"
            exit 1
        fi
        ;;
    *) RATE="1.0" ;;
esac

# RViz2 설정 파일 생성
RVIZ_CONFIG="/tmp/pointcloud_viewer.rviz"
cat > "$RVIZ_CONFIG" << 'EOF'
Panels:
  - Class: rviz_common/Displays
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /PointCloud21
      Splitter Ratio: 0.5
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Name: Tool Properties
  - Class: rviz_common/Views
    Name: Views
    Splitter Ratio: 0.5
  - Class: rviz_common/Time
    Name: Time

Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Value: true
      Cell Size: 1
      Line Style:
        Line Width: 0.03
        Value: Lines
      Color: 160; 160; 164
      Enabled: true
      Plane: XY
      Plane Cell Count: 100
      Offset:
        X: 0
        Y: 0
        Z: 0

    - Class: rviz_default_plugins/PointCloud2
      Name: PointCloud2
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
EOF

# 선택된 토픽으로 RViz 설정 업데이트
if [ -n "$SELECTED_TOPIC" ]; then
    echo "        Value: $SELECTED_TOPIC" >> "$RVIZ_CONFIG"
else
    echo "        Value: /ouster/points" >> "$RVIZ_CONFIG"
fi

cat >> "$RVIZ_CONFIG" << 'EOF'
      Enabled: true
      Size (Pixels): 3
      Size (m): 0.05
      Style: Flat Squares
      Use Fixed Frame: true
      Use rainbow: true
      Min Color: 0; 0; 0
      Max Color: 255; 255; 255
      Min Intensity: 0
      Max Intensity: 65535
      Position Transformer: XYZ
      Color Transformer: Intensity
      Queue Size: 100
      Selectable: true
      Alpha: 1
      Autocompute Intensity Bounds: true
      Invert Rainbow: false
      Decay Time: 0

    - Class: rviz_default_plugins/Axes
      Name: Axes
      Reference Frame: ""
      Value: true
      Length: 5
      Radius: 0.1
      Enabled: true

  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: os_lidar
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 30
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.06
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: false
      Focal Shape Size: 0.05
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.01
      Pitch: 0.5
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz_default_plugins)
      Yaw: 0.785
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 1000
  Hide Left Dock: false
  Hide Right Dock: false
  QMainWindow State: 000000ff00000000fd00000004000000000000016a000003a2fc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d000003a2000000c900fffffffb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073000000003d000002c4000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000100000100000003a2fc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073000000003d000003a2000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b20000000000000000000000020000073800000022fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000004420000003efc0100000002fb0000000800540069006d00650100000000000004420000000000000000fb0000000800540069006d00650100000000000004500000000000000000000005ce000003a200000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Selection:
    collapsed: false
  Time:
    collapsed: false
  Tool Properties:
    collapsed: false
  Views:
    collapsed: false
  Width: 1920
  X: 0
  Y: 0
EOF

# 재생 명령 구성
PLAY_CMD="ros2 bag play \"$SELECTED_BAG\" --rate $RATE"
if [ -n "$TOPIC_FILTER" ]; then
    PLAY_CMD="$PLAY_CMD $TOPIC_FILTER"
fi

# 반복 재생 옵션
echo ""
echo -e "${YELLOW}반복 재생하시겠습니까? (y/n):${NC}"
read -r loop_choice
if [ "$loop_choice" = "y" ]; then
    PLAY_CMD="$PLAY_CMD --loop"
fi

# 실행
echo ""
echo -e "${BLUE}================================${NC}"
echo -e "${GREEN}RViz2 실행 및 rosbag 재생 시작${NC}"
echo -e "${BLUE}================================${NC}"
echo ""
echo "재생 명령: $PLAY_CMD"
echo ""
echo -e "${YELLOW}종료하려면 Ctrl+C를 누르세요${NC}"
echo ""

# RViz2를 백그라운드에서 실행
rviz2 -d "$RVIZ_CONFIG" &
RVIZ_PID=$!

# 잠시 대기하여 RViz2가 완전히 시작되도록 함
sleep 3

# rosbag 재생
eval $PLAY_CMD

# RViz2 종료
kill $RVIZ_PID 2>/dev/null || true

# 임시 설정 파일 삭제
rm -f "$RVIZ_CONFIG"

echo ""
echo -e "${GREEN}재생이 완료되었습니다.${NC}"