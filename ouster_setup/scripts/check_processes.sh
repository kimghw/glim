#!/bin/bash

echo "======================================"
echo "Web and ROS Process Status"
echo "======================================"
echo ""

# ROS 관련 프로세스 검색
echo "[ ROS Processes ]"
echo "--------------------------------------"
ros_processes=$(ps aux | grep -E "ros2|ouster" | grep -v grep)
if [ -z "$ros_processes" ]; then
    echo "No ROS processes running"
else
    echo "$ros_processes" | awk '{printf "%-10s %-8s %-6s %s\n", $1, $2, $3, substr($0, index($0,$11))}'
fi
echo ""

# 웹 서버 관련 프로세스 검색
echo "[ Web Server Processes ]"
echo "--------------------------------------"
web_processes=$(ps aux | grep -E "python.*web|start_web|flask|http.server|SimpleHTTPServer" | grep -v grep)
if [ -z "$web_processes" ]; then
    echo "No web server processes running"
else
    echo "$web_processes" | awk '{printf "%-10s %-8s %-6s %s\n", $1, $2, $3, substr($0, index($0,$11))}'
fi
echo ""

# 포트 사용 현황
echo "[ Port Usage ]"
echo "--------------------------------------"
echo "Port 7502 (Ouster):"
netstat -tlnp 2>/dev/null | grep 7502 || echo "  Not in use"
echo ""
echo "Port 8000 (Web Server):"
netstat -tlnp 2>/dev/null | grep 8000 || echo "  Not in use"
echo ""

# ROS 노드 확인
echo "[ Active ROS2 Nodes ]"
echo "--------------------------------------"
timeout 2 ros2 node list 2>/dev/null || echo "No ROS2 nodes detected"
echo ""

echo "======================================"
