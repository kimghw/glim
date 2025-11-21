#!/bin/bash

# Ouster 라이다 고정 IP 설정 스크립트
# 자동으로 센서를 찾아 static IP를 설정하고 호스트 네트워크도 구성합니다.

# sudo 권한 확인
if [ "$EUID" -ne 0 ]; then
    echo "이 스크립트는 sudo 권한이 필요합니다."
    echo "다음 명령으로 실행하세요:"
    echo "  sudo $0 $@"
    exit 1
fi

set -e

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 도움말 출력
show_help() {
    echo "Ouster 라이다 고정 IP 설정 스크립트"
    echo ""
    echo "사용법:"
    echo "  $0 [센서_static_IP] [호스트_IP] [네트워크_인터페이스]"
    echo "  $0 --reset                            # IP 설정 초기화"
    echo ""
    echo "예제:"
    echo "  $0                                    # 자동 감지 및 기본값 사용"
    echo "  $0 192.168.10.10 192.168.10.1         # IP 지정"
    echo "  $0 192.168.10.10 192.168.10.1 enp0s1  # 인터페이스까지 지정"
    echo "  $0 --reset                            # 센서를 DHCP로 복원"
    echo ""
    echo "기본값:"
    echo "  센서 IP: 192.168.10.10/24"
    echo "  호스트 IP: 192.168.10.1/24"
    echo ""
}

# IP 형식 검증
validate_ip() {
    local ip=$1
    if [[ $ip =~ ^[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}$ ]]; then
        IFS='.' read -ra ADDR <<< "$ip"
        for i in "${ADDR[@]}"; do
            if [ $i -gt 255 ]; then
                return 1
            fi
        done
        return 0
    else
        return 1
    fi
}

# 서브넷 계산
get_subnet() {
    local ip=$1
    echo "$ip" | cut -d'.' -f1-3
}

# Reset 옵션 확인
if [ "$1" == "--reset" ]; then
    echo -e "${YELLOW}센서 IP 설정을 초기화합니다...${NC}"

    # 센서 찾기
    SENSOR_DATA=$(avahi-browse -t -p -r _ouster-lidar._tcp 2>/dev/null | grep "^=" | grep IPv4 | head -1)
    if [ -z "$SENSOR_DATA" ]; then
        echo -e "${RED}오류: Ouster 센서를 찾을 수 없습니다.${NC}"
        exit 1
    fi

    SENSOR_LINKLOCAL=$(echo "$SENSOR_DATA" | cut -d';' -f8)
    echo -e "${BLUE}센서 Link-local IP: $SENSOR_LINKLOCAL${NC}"

    # Static IP 제거
    RESPONSE=$(curl -i -X DELETE "http://$SENSOR_LINKLOCAL/api/v1/system/network/ipv4/override" \
        -w "\nHTTP_STATUS:%{http_code}" 2>/dev/null)

    HTTP_STATUS=$(echo "$RESPONSE" | grep "HTTP_STATUS:" | cut -d':' -f2)

    if [ "$HTTP_STATUS" == "200" ] || [ "$HTTP_STATUS" == "204" ]; then
        echo -e "${GREEN}✓ 센서 Static IP 설정 제거 완료${NC}"
        echo -e "${GREEN}  센서가 DHCP 모드로 복원되었습니다.${NC}"

        # Netplan 설정 제거
        if [ -f "/etc/netplan/99-ouster.yaml" ]; then
            rm -f "/etc/netplan/99-ouster.yaml"
            netplan apply 2>/dev/null
            echo -e "${GREEN}✓ 호스트 Netplan 설정 제거 완료${NC}"
        fi

        # systemd 서비스 제거
        if [ -f "/etc/systemd/system/ouster-network.service" ]; then
            systemctl disable ouster-network.service 2>/dev/null
            rm -f "/etc/systemd/system/ouster-network.service"
            echo -e "${GREEN}✓ 호스트 systemd 서비스 제거 완료${NC}"
        fi
    else
        echo -e "${RED}오류: Static IP 제거 실패 (HTTP 상태: $HTTP_STATUS)${NC}"
        exit 1
    fi

    echo -e "${GREEN}초기화 완료!${NC}"
    exit 0
fi

# 기본값 설정
SENSOR_STATIC_IP=${1:-"192.168.10.10"}
HOST_STATIC_IP=${2:-"192.168.10.1"}
NETWORK_INTERFACE=${3:-""}
SUBNET_MASK=24

echo -e "${BLUE}╔════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║   Ouster 라이다 Static IP 설정 스크립트      ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════╝${NC}"
echo ""

# IP 검증
if ! validate_ip "$SENSOR_STATIC_IP"; then
    echo -e "${RED}오류: 유효하지 않은 센서 IP 주소: $SENSOR_STATIC_IP${NC}"
    exit 1
fi

if ! validate_ip "$HOST_STATIC_IP"; then
    echo -e "${RED}오류: 유효하지 않은 호스트 IP 주소: $HOST_STATIC_IP${NC}"
    exit 1
fi

# 서브넷 확인
SENSOR_SUBNET=$(get_subnet "$SENSOR_STATIC_IP")
HOST_SUBNET=$(get_subnet "$HOST_STATIC_IP")

if [ "$SENSOR_SUBNET" != "$HOST_SUBNET" ]; then
    echo -e "${RED}오류: 센서와 호스트가 다른 서브넷에 있습니다!${NC}"
    echo "  센서: $SENSOR_STATIC_IP (서브넷: $SENSOR_SUBNET.0)"
    echo "  호스트: $HOST_STATIC_IP (서브넷: $HOST_SUBNET.0)"
    exit 1
fi

echo -e "${GREEN}설정 정보:${NC}"
echo "  센서 Static IP : $SENSOR_STATIC_IP/$SUBNET_MASK"
echo "  호스트 IP      : $HOST_STATIC_IP/$SUBNET_MASK"
echo "  서브넷         : $SENSOR_SUBNET.0/24"
echo ""

# ========================================
# STEP 1: Link-local IP로 센서 검색
# ========================================
echo -e "${YELLOW}[1/4] Link-local 주소로 Ouster 센서 검색 중...${NC}"

# avahi로 센서 검색
SENSOR_DATA=$(avahi-browse -t -p -r _ouster-lidar._tcp 2>/dev/null | grep "^=" | grep IPv4 | head -1)

if [ -z "$SENSOR_DATA" ]; then
    echo -e "${RED}오류: Ouster 센서를 찾을 수 없습니다.${NC}"
    echo "다음 사항을 확인하세요:"
    echo "  - 센서 전원이 켜져 있는지"
    echo "  - 네트워크 케이블이 연결되어 있는지"
    exit 1
fi

# 센서 정보 파싱
SENSOR_MDNS=$(echo "$SENSOR_DATA" | cut -d';' -f7)
SENSOR_LINKLOCAL=$(echo "$SENSOR_DATA" | cut -d';' -f8)
SENSOR_SN=$(echo "$SENSOR_DATA" | grep -oP 'sn=\K[0-9]{12}')

if [ -z "$SENSOR_SN" ]; then
    SENSOR_SN="unknown"
fi

echo -e "${GREEN}✓ 센서 발견${NC}"
echo "  mDNS 이름: $SENSOR_MDNS"
echo "  S/N: $SENSOR_SN"

if [ -z "$SENSOR_LINKLOCAL" ]; then
    echo -e "${RED}오류: 센서의 Link-local IP를 확인할 수 없습니다.${NC}"
    exit 1
fi

echo -e "${GREEN}✓ 센서 Link-local IP: $SENSOR_LINKLOCAL${NC}"
echo ""

# 센서 정보 확인
SENSOR_INFO=$(curl -s --connect-timeout 5 "http://$SENSOR_LINKLOCAL/api/v1/sensor/metadata" 2>/dev/null)
if [ $? -eq 0 ]; then
    SENSOR_MODEL=$(echo "$SENSOR_INFO" | grep -oP '"prod_line":"[^"]*"' | cut -d'"' -f4)
    SENSOR_STATUS=$(echo "$SENSOR_INFO" | grep -oP '"status":"[^"]*"' | cut -d'"' -f4)
    echo -e "${BLUE}센서 모델: $SENSOR_MODEL${NC}"
    echo -e "${BLUE}센서 상태: $SENSOR_STATUS${NC}"
    echo ""
fi

# ========================================
# STEP 2: 센서에 Static IP 설정
# ========================================
echo -e "${YELLOW}[2/4] 센서에 Static IP 설정 중...${NC}"
echo "  현재 주소: $SENSOR_LINKLOCAL"
echo "  설정할 IP: $SENSOR_STATIC_IP/$SUBNET_MASK"

RESPONSE=$(curl -i -X PUT "http://$SENSOR_LINKLOCAL/api/v1/system/network/ipv4/override" \
    -H 'Content-Type: application/json' \
    --data-raw "\"$SENSOR_STATIC_IP/$SUBNET_MASK\"" \
    -w "\nHTTP_STATUS:%{http_code}" 2>/dev/null)

HTTP_STATUS=$(echo "$RESPONSE" | grep "HTTP_STATUS:" | cut -d':' -f2)

if [ "$HTTP_STATUS" == "200" ] || [ "$HTTP_STATUS" == "204" ]; then
    echo -e "${GREEN}✓ 센서 Static IP 설정 성공${NC}"
    echo -e "${YELLOW}  → 센서가 네트워크를 재설정합니다 (약 5초 소요)${NC}"
    sleep 5
else
    echo -e "${RED}오류: Static IP 설정 실패 (HTTP 상태: $HTTP_STATUS)${NC}"
    exit 1
fi
echo ""

# ========================================
# STEP 3: 호스트 네트워크 인터페이스에 Static IP 추가
# ========================================
echo -e "${YELLOW}[3/4] 호스트 네트워크 인터페이스 설정 중...${NC}"

# 네트워크 인터페이스 자동 감지 (Link-local 주소가 있는 인터페이스)
if [ -z "$NETWORK_INTERFACE" ]; then
    NETWORK_INTERFACE=$(ip route get "$SENSOR_LINKLOCAL" 2>/dev/null | grep -oP 'dev \K\S+' | head -1)

    if [ -z "$NETWORK_INTERFACE" ]; then
        echo -e "${RED}오류: 네트워크 인터페이스를 자동으로 찾을 수 없습니다.${NC}"
        echo "사용 가능한 인터페이스:"
        ip link show | grep "^[0-9]" | awk -F': ' '{print "  - "$2}'
        exit 1
    fi
fi

echo "  인터페이스: $NETWORK_INTERFACE"
echo "  추가할 IP: $HOST_STATIC_IP/$SUBNET_MASK"

# 기존 IP 확인
if ip addr show "$NETWORK_INTERFACE" | grep -q "$HOST_STATIC_IP"; then
    echo -e "${YELLOW}  → IP가 이미 설정되어 있습니다.${NC}"
else
    # IP 추가 (sudo 권한으로 실행 중이므로 바로 추가)
    echo -e "${YELLOW}  → 호스트 IP를 추가합니다...${NC}"

    if ip addr add "$HOST_STATIC_IP/$SUBNET_MASK" dev "$NETWORK_INTERFACE" 2>&1; then
        echo -e "${GREEN}✓ 호스트 IP 추가 성공${NC}"
    else
        echo -e "${RED}오류: IP 주소 추가 실패${NC}"
        echo "다음 명령을 수동으로 실행하세요:"
        echo "  sudo ip addr add $HOST_STATIC_IP/$SUBNET_MASK dev $NETWORK_INTERFACE"
        exit 1
    fi
fi
echo ""

# ========================================
# STEP 4: 새 Static IP로 연결 확인
# ========================================
echo -e "${YELLOW}[4/4] 새 Static IP로 연결 확인 중...${NC}"
echo "  연결 시도: $SENSOR_STATIC_IP"

MAX_RETRIES=10
RETRY_COUNT=0
SUCCESS=false

while [ $RETRY_COUNT -lt $MAX_RETRIES ]; do
    if curl -s --connect-timeout 3 "http://$SENSOR_STATIC_IP/api/v1/sensor/metadata" > /dev/null 2>&1; then
        SUCCESS=true
        break
    fi
    RETRY_COUNT=$((RETRY_COUNT + 1))
    echo "  시도 $RETRY_COUNT/$MAX_RETRIES..."
    sleep 2
done

echo ""

if [ "$SUCCESS" = true ]; then
    echo -e "${GREEN}╔════════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║            설정 완료! ✓                        ║${NC}"
    echo -e "${GREEN}╚════════════════════════════════════════════════╝${NC}"
    echo ""
    echo -e "${GREEN}센서 정보:${NC}"
    echo "  Serial Number : $SENSOR_SN"
    echo "  Model         : $SENSOR_MODEL"
    echo "  Static IP     : $SENSOR_STATIC_IP"
    echo ""
    echo -e "${GREEN}호스트 정보:${NC}"
    echo "  Interface     : $NETWORK_INTERFACE"
    echo "  IP Address    : $HOST_STATIC_IP/$SUBNET_MASK"
    echo ""
    echo -e "${BLUE}사용 예시:${NC}"
    echo "  # 센서 메타데이터 확인"
    echo "  curl http://$SENSOR_STATIC_IP/api/v1/sensor/metadata"
    echo ""
    echo "  # ROS2에서 사용"
    echo "  ros2 launch ouster_ros sensor.launch.py sensor_hostname:=$SENSOR_STATIC_IP"
    echo ""
    echo -e "${YELLOW}호스트 IP 영구 설정 중...${NC}"

    # Netplan 설정 파일 생성 (Ubuntu 18.04+)
    NETPLAN_FILE="/etc/netplan/99-ouster.yaml"
    echo -e "${YELLOW}  → Netplan 설정 파일 생성: $NETPLAN_FILE${NC}"

    cat > "$NETPLAN_FILE" << EOF
network:
  version: 2
  ethernets:
    $NETWORK_INTERFACE:
      addresses:
        - $HOST_STATIC_IP/$SUBNET_MASK
      optional: true
EOF

    # Netplan 적용
    netplan apply 2>/dev/null

    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ 호스트 IP 영구 설정 완료 (Netplan)${NC}"
    else
        echo -e "${YELLOW}⚠ Netplan 설정 실패, systemd 서비스로 설정 시도...${NC}"

        # 대체 방법: systemd 서비스 생성
        SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
        SERVICE_FILE="/etc/systemd/system/ouster-network.service"

        # 자동 실행 스크립트 생성
        cat > "$SCRIPT_DIR/auto_setup_network.sh" << EOF2
#!/bin/bash
# Ouster 네트워크 자동 설정
if ! ip addr show $NETWORK_INTERFACE | grep -q "$HOST_STATIC_IP"; then
    ip addr add $HOST_STATIC_IP/$SUBNET_MASK dev $NETWORK_INTERFACE
fi
EOF2
        chmod +x "$SCRIPT_DIR/auto_setup_network.sh"

        # systemd 서비스 생성
        cat > "$SERVICE_FILE" << EOF3
[Unit]
Description=Ouster Network Configuration
After=network.target

[Service]
Type=oneshot
ExecStart=$SCRIPT_DIR/auto_setup_network.sh
RemainAfterExit=true

[Install]
WantedBy=multi-user.target
EOF3

        systemctl enable ouster-network.service 2>/dev/null
        systemctl start ouster-network.service 2>/dev/null

        if [ $? -eq 0 ]; then
            echo -e "${GREEN}✓ 호스트 IP 영구 설정 완료 (systemd)${NC}"
        else
            echo -e "${YELLOW}⚠ 자동 설정 실패, 수동 설정 필요${NC}"
        fi
    fi

    echo -e "${GREEN}✓ 재부팅 후에도 설정이 유지됩니다.${NC}"
else
    echo -e "${RED}╔════════════════════════════════════════════════╗${NC}"
    echo -e "${RED}║            연결 실패!                          ║${NC}"
    echo -e "${RED}╚════════════════════════════════════════════════╝${NC}"
    echo ""
    echo "센서는 Static IP로 설정되었지만 연결을 확인할 수 없습니다."
    echo ""
    echo "확인 사항:"
    echo "  1. 호스트 IP가 제대로 설정되었는지:"
    echo "     ip addr show $NETWORK_INTERFACE"
    echo ""
    echo "  2. 핑 테스트:"
    echo "     ping -c 3 $SENSOR_STATIC_IP"
    echo ""
    echo "  3. Static IP 제거 (필요시):"
    echo "     curl -X DELETE http://$SENSOR_LINKLOCAL/api/v1/system/network/ipv4/override"
fi
