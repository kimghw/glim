#!/bin/bash

# 웹 대시보드 의존성 설치 스크립트

echo "Flask 및 의존성 설치 중..."

sudo apt update
sudo apt install -y python3-flask python3-flask-cors

echo ""
echo "✓ 설치 완료!"
echo ""
echo "웹 서버를 시작하려면:"
echo "  ./start_web.sh"
