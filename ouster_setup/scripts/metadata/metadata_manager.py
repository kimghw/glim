#!/usr/bin/env python3
"""
ROS bag 메타데이터 관리 시스템
- 위치, 시간, 날씨, 환경 정보 등 저장
- LLM/RAG 검색을 위한 구조화된 메타데이터
- JSON 및 YAML 형식 지원
"""

import json
import yaml
import os
import sys
from datetime import datetime
from pathlib import Path
import subprocess
import socket
import requests
from typing import Dict, Any, Optional, List
import hashlib

class RosbagMetadata:
    """Rosbag 메타데이터 관리 클래스"""

    def __init__(self, bag_path: str = None):
        self.bag_path = Path(bag_path) if bag_path else None
        self.metadata = self._create_base_metadata()

    def _create_base_metadata(self) -> Dict[str, Any]:
        """기본 메타데이터 구조 생성"""
        return {
            # 시간 정보
            "temporal": {
                "created_at": datetime.now().isoformat(),
                "date": datetime.now().strftime("%Y-%m-%d"),
                "time": datetime.now().strftime("%H:%M:%S"),
                "timezone": "KST",
                "unix_timestamp": int(datetime.now().timestamp()),
                "day_of_week": datetime.now().strftime("%A"),
                "week_number": datetime.now().isocalendar()[1],
            },

            # 위치 정보
            "location": {
                "coordinates": {
                    "latitude": None,
                    "longitude": None,
                    "altitude": None
                },
                "address": None,
                "building": None,
                "floor": None,
                "room": None,
                "area_name": None,
                "description": None,
                "indoor_outdoor": None,  # "indoor" or "outdoor"
            },

            # 환경 정보
            "environment": {
                "weather": {
                    "condition": None,  # sunny, cloudy, rainy, snowy, foggy
                    "temperature": None,
                    "humidity": None,
                    "wind_speed": None,
                    "visibility": None,
                },
                "lighting": None,  # bright, dim, dark, mixed
                "surface_type": None,  # concrete, asphalt, grass, carpet, tile
                "obstacles": [],  # list of obstacle types
                "traffic_density": None,  # none, low, medium, high
                "crowd_density": None,  # none, low, medium, high
            },

            # 센서 정보
            "sensor": {
                "type": "Ouster OS1-64",
                "serial_number": None,
                "firmware_version": None,
                "ip_address": "192.168.10.10",
                "configuration": {
                    "lidar_mode": None,
                    "azimuth_window": None,
                    "operating_mode": None,
                },
                "calibration_date": None,
                "status": "normal",  # normal, degraded, maintenance
            },

            # 데이터 특성
            "data_characteristics": {
                "duration_seconds": None,
                "total_messages": None,
                "topics": [],
                "file_size_mb": None,
                "point_cloud_density": None,  # sparse, normal, dense
                "quality": None,  # low, medium, high
                "completeness": None,  # percentage
                "special_events": [],  # list of notable events
            },

            # 실험/프로젝트 정보
            "experiment": {
                "project_name": None,
                "experiment_id": None,
                "session_id": None,
                "operator": None,
                "purpose": None,
                "test_scenario": None,
                "notes": None,
                "tags": [],  # searchable tags
                "related_bags": [],  # IDs of related recordings
            },

            # 시스템 정보
            "system": {
                "hostname": socket.gethostname(),
                "platform": sys.platform,
                "ros_version": "ROS2 Jazzy",
                "recording_tool": "ouster_setup",
                "tool_version": "1.0.0",
            },

            # 검색/인덱싱 정보
            "search_metadata": {
                "keywords": [],  # 자동 생성된 키워드
                "summary": None,  # LLM이 생성할 수 있는 요약
                "embedding": None,  # 벡터 임베딩 (향후 사용)
                "index_version": "1.0",
                "searchable_text": None,  # 모든 텍스트 정보 결합
            },

            # 파일 정보
            "file_info": {
                "filename": None,
                "path": None,
                "checksum": None,  # MD5 or SHA256
                "compression": None,
                "archived": False,
                "backup_location": None,
            }
        }

    def add_location(self, **kwargs):
        """위치 정보 추가"""
        for key, value in kwargs.items():
            if key in self.metadata["location"]:
                self.metadata["location"][key] = value
            elif key in self.metadata["location"]["coordinates"]:
                self.metadata["location"]["coordinates"][key] = value

    def add_weather(self, **kwargs):
        """날씨 정보 추가"""
        for key, value in kwargs.items():
            if key in self.metadata["environment"]["weather"]:
                self.metadata["environment"]["weather"][key] = value

    def add_experiment_info(self, **kwargs):
        """실험 정보 추가"""
        for key, value in kwargs.items():
            if key in self.metadata["experiment"]:
                self.metadata["experiment"][key] = value

    def add_tags(self, tags: List[str]):
        """검색 태그 추가"""
        if isinstance(tags, list):
            self.metadata["experiment"]["tags"].extend(tags)
            self.metadata["experiment"]["tags"] = list(set(self.metadata["experiment"]["tags"]))

    def add_notes(self, notes: str):
        """메모 추가"""
        if self.metadata["experiment"]["notes"]:
            self.metadata["experiment"]["notes"] += f"\n{notes}"
        else:
            self.metadata["experiment"]["notes"] = notes

    def auto_generate_keywords(self):
        """메타데이터에서 자동으로 키워드 생성"""
        keywords = []

        # 날짜/시간 키워드
        keywords.append(self.metadata["temporal"]["date"])
        keywords.append(self.metadata["temporal"]["day_of_week"])

        # 위치 키워드
        if self.metadata["location"]["area_name"]:
            keywords.append(self.metadata["location"]["area_name"])
        if self.metadata["location"]["building"]:
            keywords.append(self.metadata["location"]["building"])

        # 환경 키워드
        if self.metadata["environment"]["weather"]["condition"]:
            keywords.append(self.metadata["environment"]["weather"]["condition"])
        if self.metadata["environment"]["lighting"]:
            keywords.append(self.metadata["environment"]["lighting"])

        # 태그 추가
        keywords.extend(self.metadata["experiment"]["tags"])

        # 중복 제거
        self.metadata["search_metadata"]["keywords"] = list(set(keywords))

    def generate_searchable_text(self):
        """모든 텍스트 정보를 결합하여 검색 가능한 텍스트 생성"""
        text_parts = []

        # 시간 정보
        text_parts.append(f"Date: {self.metadata['temporal']['date']}")
        text_parts.append(f"Time: {self.metadata['temporal']['time']}")
        text_parts.append(f"Day: {self.metadata['temporal']['day_of_week']}")

        # 위치 정보
        if self.metadata["location"]["description"]:
            text_parts.append(f"Location: {self.metadata['location']['description']}")
        if self.metadata["location"]["area_name"]:
            text_parts.append(f"Area: {self.metadata['location']['area_name']}")

        # 환경 정보
        if self.metadata["environment"]["weather"]["condition"]:
            text_parts.append(f"Weather: {self.metadata['environment']['weather']['condition']}")

        # 실험 정보
        if self.metadata["experiment"]["purpose"]:
            text_parts.append(f"Purpose: {self.metadata['experiment']['purpose']}")
        if self.metadata["experiment"]["notes"]:
            text_parts.append(f"Notes: {self.metadata['experiment']['notes']}")

        # 태그
        if self.metadata["experiment"]["tags"]:
            text_parts.append(f"Tags: {', '.join(self.metadata['experiment']['tags'])}")

        self.metadata["search_metadata"]["searchable_text"] = "\n".join(text_parts)

    def analyze_bag_file(self):
        """bag 파일 분석하여 메타데이터 추가"""
        if not self.bag_path or not self.bag_path.exists():
            return

        try:
            # 파일 정보
            stat = self.bag_path.stat()
            self.metadata["file_info"]["filename"] = self.bag_path.name
            self.metadata["file_info"]["path"] = str(self.bag_path.absolute())

            # 파일 크기 계산
            if self.bag_path.is_dir():
                total_size = sum(f.stat().st_size for f in self.bag_path.rglob('*') if f.is_file())
                self.metadata["data_characteristics"]["file_size_mb"] = round(total_size / 1024 / 1024, 2)

            # ROS2 bag info 실행
            result = subprocess.run(
                f"source /opt/ros/jazzy/setup.bash && ros2 bag info {self.bag_path}",
                shell=True,
                capture_output=True,
                text=True,
                timeout=10
            )

            if result.returncode == 0:
                info = result.stdout
                lines = info.split('\n')

                for line in lines:
                    if 'Duration:' in line:
                        duration_str = line.split('Duration:')[1].strip()
                        # Parse duration (e.g., "10.5s" -> 10.5)
                        if 's' in duration_str:
                            self.metadata["data_characteristics"]["duration_seconds"] = float(duration_str.replace('s', ''))
                    elif 'Messages:' in line:
                        self.metadata["data_characteristics"]["total_messages"] = int(line.split('Messages:')[1].strip())

        except Exception as e:
            print(f"Error analyzing bag file: {e}")

    def calculate_checksum(self):
        """파일 체크섬 계산"""
        if not self.bag_path or not self.bag_path.exists():
            return

        try:
            if self.bag_path.is_file():
                # 단일 파일 체크섬
                with open(self.bag_path, 'rb') as f:
                    checksum = hashlib.md5(f.read()).hexdigest()
            else:
                # 디렉토리의 경우 주요 파일들의 체크섬
                checksums = []
                for file in sorted(self.bag_path.rglob('*.mcap')):
                    with open(file, 'rb') as f:
                        checksums.append(hashlib.md5(f.read()).hexdigest())
                checksum = hashlib.md5(''.join(checksums).encode()).hexdigest()

            self.metadata["file_info"]["checksum"] = checksum
        except Exception as e:
            print(f"Error calculating checksum: {e}")

    def save_metadata(self, output_path: str = None):
        """메타데이터를 파일로 저장"""
        if not output_path and self.bag_path:
            # bag 파일과 같은 디렉토리에 저장
            if self.bag_path.is_dir():
                output_path = self.bag_path / "rich_metadata.json"
            else:
                output_path = self.bag_path.parent / f"{self.bag_path.stem}_rich_metadata.json"

        output_path = Path(output_path)

        # 자동 생성 필드 업데이트
        self.auto_generate_keywords()
        self.generate_searchable_text()

        # JSON으로 저장
        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(self.metadata, f, indent=2, ensure_ascii=False)

        # YAML 버전도 저장
        yaml_path = output_path.with_suffix('.yaml')
        with open(yaml_path, 'w', encoding='utf-8') as f:
            yaml.dump(self.metadata, f, default_flow_style=False, allow_unicode=True)

        print(f"Metadata saved to: {output_path}")
        print(f"YAML version saved to: {yaml_path}")

        return output_path

    def load_metadata(self, metadata_path: str):
        """저장된 메타데이터 로드"""
        metadata_path = Path(metadata_path)

        if metadata_path.suffix == '.json':
            with open(metadata_path, 'r', encoding='utf-8') as f:
                self.metadata = json.load(f)
        elif metadata_path.suffix in ['.yaml', '.yml']:
            with open(metadata_path, 'r', encoding='utf-8') as f:
                self.metadata = yaml.safe_load(f)
        else:
            raise ValueError(f"Unsupported file format: {metadata_path.suffix}")

    def search(self, query: str) -> bool:
        """메타데이터에서 쿼리 검색"""
        query_lower = query.lower()

        # 검색 가능한 텍스트에서 검색
        if self.metadata["search_metadata"]["searchable_text"]:
            if query_lower in self.metadata["search_metadata"]["searchable_text"].lower():
                return True

        # 키워드에서 검색
        for keyword in self.metadata["search_metadata"]["keywords"]:
            if query_lower in keyword.lower():
                return True

        # 태그에서 검색
        for tag in self.metadata["experiment"]["tags"]:
            if query_lower in tag.lower():
                return True

        return False


def interactive_metadata_collection(bag_path: str = None):
    """대화형으로 메타데이터 수집"""
    metadata = RosbagMetadata(bag_path)

    print("\n=== ROS Bag 메타데이터 수집 ===\n")

    # 위치 정보
    print("📍 위치 정보:")
    area = input("  지역 이름 (예: 연구실, 주차장): ").strip()
    if area:
        metadata.add_location(area_name=area)

    building = input("  건물 이름: ").strip()
    if building:
        metadata.add_location(building=building)

    floor = input("  층수: ").strip()
    if floor:
        metadata.add_location(floor=floor)

    indoor = input("  실내/실외 (indoor/outdoor): ").strip()
    if indoor:
        metadata.add_location(indoor_outdoor=indoor)

    location_desc = input("  위치 설명: ").strip()
    if location_desc:
        metadata.add_location(description=location_desc)

    # 환경 정보
    print("\n🌤️ 환경 정보:")
    weather = input("  날씨 (sunny/cloudy/rainy/snowy/foggy): ").strip()
    if weather:
        metadata.add_weather(condition=weather)

    temp = input("  온도 (°C): ").strip()
    if temp:
        try:
            metadata.add_weather(temperature=float(temp))
        except:
            pass

    lighting = input("  조명 상태 (bright/dim/dark/mixed): ").strip()
    if lighting:
        metadata.metadata["environment"]["lighting"] = lighting

    surface = input("  바닥 재질 (concrete/asphalt/grass/carpet/tile): ").strip()
    if surface:
        metadata.metadata["environment"]["surface_type"] = surface

    # 실험 정보
    print("\n🔬 실험 정보:")
    purpose = input("  녹화 목적: ").strip()
    if purpose:
        metadata.add_experiment_info(purpose=purpose)

    scenario = input("  테스트 시나리오: ").strip()
    if scenario:
        metadata.add_experiment_info(test_scenario=scenario)

    operator = input("  작업자 이름: ").strip()
    if operator:
        metadata.add_experiment_info(operator=operator)

    notes = input("  추가 메모: ").strip()
    if notes:
        metadata.add_notes(notes)

    # 태그
    print("\n🏷️ 검색 태그:")
    tags_input = input("  태그 (쉼표로 구분): ").strip()
    if tags_input:
        tags = [tag.strip() for tag in tags_input.split(',')]
        metadata.add_tags(tags)

    # bag 파일 분석
    if bag_path:
        print("\n📊 Bag 파일 분석 중...")
        metadata.analyze_bag_file()
        metadata.calculate_checksum()

    # 메타데이터 저장
    metadata.save_metadata()

    return metadata


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="ROS Bag 메타데이터 관리")
    parser.add_argument("bag_path", nargs='?', help="ROS bag 파일 경로")
    parser.add_argument("--interactive", "-i", action="store_true", help="대화형 메타데이터 수집")
    parser.add_argument("--search", "-s", help="메타데이터 검색 쿼리")
    parser.add_argument("--load", "-l", help="기존 메타데이터 로드")

    args = parser.parse_args()

    if args.interactive:
        interactive_metadata_collection(args.bag_path)
    elif args.search and args.load:
        metadata = RosbagMetadata()
        metadata.load_metadata(args.load)
        if metadata.search(args.search):
            print(f"✅ Found match for '{args.search}'")
            print(json.dumps(metadata.metadata, indent=2))
        else:
            print(f"❌ No match for '{args.search}'")
    elif args.bag_path:
        # 기본 메타데이터 생성
        metadata = RosbagMetadata(args.bag_path)
        metadata.analyze_bag_file()
        metadata.save_metadata()
    else:
        parser.print_help()