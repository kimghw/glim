#!/bin/bash

# Ouster 호스트 IP 설정 스크립트
# 호스트 네트워크 인터페이스에 고정 IP를 설정합니다.

# sudo 권한 확인
if [ "$EUID" -ne 0 ]; then
    echo "이 스크립트는 sudo 권한이 필요합니다."
    echo "다음 명령으로 실행하세요:"
    echo "  sudo $0 $@"
    exit 1
fi

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}╔════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║      Ouster 호스트 IP 설정 스크립트           ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════╝${NC}"
echo ""

# 설정 변수
HOST_IP="192.168.10.1"
SUBNET_MASK="24"
NETWORK_INTERFACE="enxf8e43b49701e"
SENSOR_IP="192.168.10.10"

echo -e "${GREEN}설정 정보:${NC}"
echo "  호스트 IP      : $HOST_IP/$SUBNET_MASK"
echo "  네트워크 IF    : $NETWORK_INTERFACE"
echo "  센서 IP        : $SENSOR_IP"
echo ""

# ========================================
# STEP 1: Netplan 설정 파일 생성
# ========================================
echo -e "${YELLOW}[1/4] Netplan 설정 파일 생성 중...${NC}"

NETPLAN_FILE="/etc/netplan/99-ouster.yaml"

cat > "$NETPLAN_FILE" << EOF
network:
  version: 2
  ethernets:
    $NETWORK_INTERFACE:
      addresses:
        - $HOST_IP/$SUBNET_MASK
      optional: true
EOF

if [ -f "$NETPLAN_FILE" ]; then
    echo -e "${GREEN}✓ Netplan 설정 파일 생성 완료: $NETPLAN_FILE${NC}"
    chmod 600 "$NETPLAN_FILE"
else
    echo -e "${RED}✗ Netplan 설정 파일 생성 실패${NC}"
    exit 1
fi
echo ""

# ========================================
# STEP 2: Netplan 적용
# ========================================
echo -e "${YELLOW}[2/4] Netplan 설정 적용 중...${NC}"

if netplan apply 2>/dev/null; then
    echo -e "${GREEN}✓ Netplan 설정 적용 완료${NC}"
else
    echo -e "${YELLOW}⚠ Netplan 적용 실패, IP를 직접 추가합니다...${NC}"

    # 기존 IP 확인
    if ip addr show "$NETWORK_INTERFACE" | grep -q "$HOST_IP"; then
        echo -e "${YELLOW}  → IP가 이미 설정되어 있습니다.${NC}"
    else
        if ip addr add "$HOST_IP/$SUBNET_MASK" dev "$NETWORK_INTERFACE" 2>&1; then
            echo -e "${GREEN}✓ 호스트 IP 추가 성공${NC}"
        else
            echo -e "${RED}✗ IP 주소 추가 실패${NC}"
            exit 1
        fi
    fi
fi
echo ""

# ========================================
# STEP 3: 네트워크 인터페이스 상태 확인
# ========================================
echo -e "${YELLOW}[3/4] 네트워크 인터페이스 상태 확인...${NC}"

if ip addr show "$NETWORK_INTERFACE" | grep -q "$HOST_IP"; then
    echo -e "${GREEN}✓ 호스트 IP 설정 확인: $HOST_IP/$SUBNET_MASK${NC}"
    ip addr show "$NETWORK_INTERFACE" | grep "inet "
else
    echo -e "${RED}✗ IP 설정을 확인할 수 없습니다${NC}"
    exit 1
fi
echo ""

# ========================================
# STEP 4: 센서 연결 확인
# ========================================
echo -e "${YELLOW}[4/4] 센서 연결 확인 중...${NC}"

echo "  센서 IP: $SENSOR_IP"
if ping -c 3 -W 2 $SENSOR_IP > /dev/null 2>&1; then
    echo -e "${GREEN}✓ 센서 연결 성공!${NC}"

    # 센서 메타데이터 확인
    SENSOR_INFO=$(curl -s --connect-timeout 2 http://$SENSOR_IP/api/v1/sensor/metadata 2>/dev/null)
    if [ $? -eq 0 ] && [ ! -z "$SENSOR_INFO" ]; then
        MODEL=$(echo "$SENSOR_INFO" | grep -oP '"prod_line":"[^"]*"' | cut -d'"' -f4)
        SERIAL=$(echo "$SENSOR_INFO" | grep -oP '"prod_sn":"[^"]*"' | cut -d'"' -f4)
        STATUS=$(echo "$SENSOR_INFO" | grep -oP '"status":"[^"]*"' | cut -d'"' -f4)

        echo ""
        echo -e "${BLUE}센서 정보:${NC}"
        echo "  모델      : $MODEL"
        echo "  시리얼    : $SERIAL"
        echo "  상태      : $STATUS"
    fi
else
    echo -e "${RED}✗ 센서 연결 실패${NC}"
    echo -e "${YELLOW}  센서가 아직 재시작 중일 수 있습니다.${NC}"
    echo -e "${YELLOW}  잠시 후 다시 시도해주세요: ping $SENSOR_IP${NC}"
fi
echo ""

# ========================================
# 완료
# ========================================
echo -e "${GREEN}╔════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║            설정 완료!                          ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${BLUE}다음 단계:${NC}"
echo "  1. 센서 연결 확인: ping $SENSOR_IP"
echo "  2. 상태 모니터: ./check_status.sh"
echo "  3. 드라이버 실행: ros2 launch ouster_setup/launch/ouster_driver.launch.py"
echo ""
