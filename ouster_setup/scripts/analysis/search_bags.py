#!/usr/bin/env python3
"""
ROS bag 메타데이터 검색 도구
- 자연어 쿼리로 bag 파일 검색
- LLM과 통합 가능한 인터페이스
"""

import json
import yaml
import os
import sys
from pathlib import Path
from datetime import datetime
from typing import List, Dict, Any, Optional
import argparse
from tabulate import tabulate

# metadata_manager에서 클래스 import
sys.path.append(str(Path(__file__).parent.parent / 'metadata'))
from metadata_manager import RosbagMetadata


class BagSearchEngine:
    """ROS bag 검색 엔진"""

    def __init__(self, data_dir: str = "/home/kimghw/glim/rosbag_data"):
        self.data_dir = Path(data_dir)
        self.bags_metadata = []
        self.load_all_metadata()

    def load_all_metadata(self):
        """모든 bag 파일의 메타데이터 로드"""
        if not self.data_dir.exists():
            print(f"데이터 디렉토리가 없습니다: {self.data_dir}")
            return

        for bag_dir in self.data_dir.iterdir():
            if bag_dir.is_dir():
                # 메타데이터 파일 찾기
                json_meta = bag_dir / "rich_metadata.json"
                yaml_meta = bag_dir / "rich_metadata.yaml"

                metadata_file = None
                if json_meta.exists():
                    metadata_file = json_meta
                elif yaml_meta.exists():
                    metadata_file = yaml_meta

                if metadata_file:
                    try:
                        metadata = RosbagMetadata()
                        metadata.load_metadata(str(metadata_file))
                        self.bags_metadata.append({
                            'path': str(bag_dir),
                            'name': bag_dir.name,
                            'metadata': metadata
                        })
                    except Exception as e:
                        print(f"메타데이터 로드 실패 ({bag_dir.name}): {e}")

        print(f"총 {len(self.bags_metadata)}개의 bag 파일을 로드했습니다.")

    def search(self, query: str, filters: Dict[str, Any] = None) -> List[Dict]:
        """쿼리로 bag 파일 검색"""
        results = []

        for bag_info in self.bags_metadata:
            metadata = bag_info['metadata']
            score = 0
            matched_fields = []

            # 텍스트 검색
            if metadata.search(query):
                score += 10
                matched_fields.append("text_search")

            # 필터 적용
            if filters:
                if self._apply_filters(metadata.metadata, filters):
                    score += 5
                    matched_fields.append("filters")

            # 날짜 검색
            if self._search_date(metadata.metadata, query):
                score += 8
                matched_fields.append("date")

            # 위치 검색
            if self._search_location(metadata.metadata, query):
                score += 7
                matched_fields.append("location")

            # 환경 검색
            if self._search_environment(metadata.metadata, query):
                score += 6
                matched_fields.append("environment")

            if score > 0:
                results.append({
                    'path': bag_info['path'],
                    'name': bag_info['name'],
                    'score': score,
                    'matched_fields': matched_fields,
                    'metadata': metadata.metadata
                })

        # 점수 기준 정렬
        results.sort(key=lambda x: x['score'], reverse=True)
        return results

    def _apply_filters(self, metadata: Dict, filters: Dict) -> bool:
        """필터 적용"""
        for key, value in filters.items():
            if key == "date_from":
                bag_date = datetime.fromisoformat(metadata["temporal"]["created_at"])
                if bag_date < datetime.fromisoformat(value):
                    return False
            elif key == "date_to":
                bag_date = datetime.fromisoformat(metadata["temporal"]["created_at"])
                if bag_date > datetime.fromisoformat(value):
                    return False
            elif key == "location":
                if value.lower() not in str(metadata["location"]).lower():
                    return False
            elif key == "weather":
                if metadata["environment"]["weather"]["condition"] != value:
                    return False
            elif key == "indoor":
                if metadata["location"]["indoor_outdoor"] != ("indoor" if value else "outdoor"):
                    return False

        return True

    def _search_date(self, metadata: Dict, query: str) -> bool:
        """날짜 관련 검색"""
        query_lower = query.lower()

        # 요일 검색
        if metadata["temporal"]["day_of_week"].lower() in query_lower:
            return True

        # 날짜 검색
        if metadata["temporal"]["date"] in query:
            return True

        # 상대적 시간 검색
        if "today" in query_lower:
            today = datetime.now().strftime("%Y-%m-%d")
            if metadata["temporal"]["date"] == today:
                return True

        if "yesterday" in query_lower:
            from datetime import timedelta
            yesterday = (datetime.now() - timedelta(days=1)).strftime("%Y-%m-%d")
            if metadata["temporal"]["date"] == yesterday:
                return True

        return False

    def _search_location(self, metadata: Dict, query: str) -> bool:
        """위치 관련 검색"""
        query_lower = query.lower()
        location = metadata["location"]

        # 각 위치 필드 검색
        for field in ["area_name", "building", "room", "description"]:
            if location.get(field) and location[field].lower() in query_lower:
                return True

        # 실내/실외 검색
        if "indoor" in query_lower and location.get("indoor_outdoor") == "indoor":
            return True
        if "outdoor" in query_lower and location.get("indoor_outdoor") == "outdoor":
            return True

        return False

    def _search_environment(self, metadata: Dict, query: str) -> bool:
        """환경 관련 검색"""
        query_lower = query.lower()
        env = metadata["environment"]

        # 날씨 검색
        weather = env["weather"].get("condition")
        if weather and weather.lower() in query_lower:
            return True

        # 조명 검색
        lighting = env.get("lighting")
        if lighting and lighting.lower() in query_lower:
            return True

        # 바닥 재질 검색
        surface = env.get("surface_type")
        if surface and surface.lower() in query_lower:
            return True

        return False

    def advanced_search(self, nlp_query: str) -> List[Dict]:
        """자연어 쿼리 처리 (LLM 통합 가능)"""
        # 간단한 키워드 추출 (향후 LLM으로 대체 가능)
        filters = {}

        # 날씨 키워드
        for weather in ["sunny", "cloudy", "rainy", "snowy", "foggy"]:
            if weather in nlp_query.lower():
                filters["weather"] = weather

        # 실내/실외
        if "indoor" in nlp_query.lower():
            filters["indoor"] = True
        elif "outdoor" in nlp_query.lower():
            filters["indoor"] = False

        return self.search(nlp_query, filters)

    def get_summary(self, bag_path: str) -> str:
        """bag 파일의 요약 정보 반환"""
        for bag_info in self.bags_metadata:
            if bag_info['path'] == bag_path:
                metadata = bag_info['metadata'].metadata

                summary = []
                summary.append(f"📁 Bag: {bag_info['name']}")
                summary.append(f"📅 Date: {metadata['temporal']['date']} {metadata['temporal']['time']}")

                if metadata['location']['area_name']:
                    summary.append(f"📍 Location: {metadata['location']['area_name']}")

                if metadata['environment']['weather']['condition']:
                    summary.append(f"🌤️ Weather: {metadata['environment']['weather']['condition']}")

                if metadata['experiment']['purpose']:
                    summary.append(f"🎯 Purpose: {metadata['experiment']['purpose']}")

                if metadata['data_characteristics']['duration_seconds']:
                    duration = metadata['data_characteristics']['duration_seconds']
                    summary.append(f"⏱️ Duration: {duration:.1f} seconds")

                if metadata['experiment']['tags']:
                    summary.append(f"🏷️ Tags: {', '.join(metadata['experiment']['tags'])}")

                return "\n".join(summary)

        return "No metadata found"

    def export_for_rag(self, output_file: str = "bags_index.json"):
        """RAG/LLM을 위한 인덱스 내보내기"""
        index = []

        for bag_info in self.bags_metadata:
            metadata = bag_info['metadata'].metadata

            # RAG를 위한 구조화된 문서
            doc = {
                "id": bag_info['name'],
                "path": bag_info['path'],
                "content": metadata['search_metadata'].get('searchable_text', ''),
                "metadata": {
                    "date": metadata['temporal']['date'],
                    "time": metadata['temporal']['time'],
                    "location": metadata['location'].get('area_name', ''),
                    "weather": metadata['environment']['weather'].get('condition', ''),
                    "purpose": metadata['experiment'].get('purpose', ''),
                    "tags": metadata['experiment'].get('tags', []),
                    "duration": metadata['data_characteristics'].get('duration_seconds', 0)
                },
                "embedding": metadata['search_metadata'].get('embedding', None)
            }
            index.append(doc)

        # JSON으로 저장
        with open(output_file, 'w', encoding='utf-8') as f:
            json.dump(index, f, indent=2, ensure_ascii=False)

        print(f"RAG 인덱스를 {output_file}에 저장했습니다.")
        return index


def interactive_search():
    """대화형 검색 인터페이스"""
    engine = BagSearchEngine()

    print("\n🔍 ROS Bag 검색 엔진")
    print("=" * 50)
    print("명령어:")
    print("  search <query>  - 검색")
    print("  list            - 모든 bag 파일 표시")
    print("  info <name>     - 상세 정보")
    print("  export          - RAG 인덱스 내보내기")
    print("  quit            - 종료")
    print("=" * 50)

    while True:
        try:
            command = input("\n> ").strip()

            if command.startswith("search "):
                query = command[7:]
                results = engine.advanced_search(query)

                if results:
                    print(f"\n검색 결과 ({len(results)}개):")
                    table = []
                    for r in results[:10]:  # 상위 10개만 표시
                        table.append([
                            r['name'],
                            r['score'],
                            ', '.join(r['matched_fields']),
                            r['metadata']['temporal']['date']
                        ])
                    print(tabulate(table, headers=['Bag 이름', '점수', '매칭 필드', '날짜']))
                else:
                    print("검색 결과가 없습니다.")

            elif command == "list":
                print(f"\n전체 bag 파일 ({len(engine.bags_metadata)}개):")
                for bag in engine.bags_metadata:
                    print(f"  - {bag['name']}")

            elif command.startswith("info "):
                bag_name = command[5:]
                found = False
                for bag in engine.bags_metadata:
                    if bag['name'] == bag_name:
                        print(f"\n{engine.get_summary(bag['path'])}")
                        found = True
                        break
                if not found:
                    print(f"'{bag_name}'을 찾을 수 없습니다.")

            elif command == "export":
                engine.export_for_rag()

            elif command in ["quit", "exit", "q"]:
                break

            else:
                print("알 수 없는 명령어입니다.")

        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"오류: {e}")

    print("\n검색 엔진을 종료합니다.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ROS Bag 메타데이터 검색")
    parser.add_argument("query", nargs='?', help="검색 쿼리")
    parser.add_argument("--interactive", "-i", action="store_true", help="대화형 모드")
    parser.add_argument("--export", "-e", action="store_true", help="RAG 인덱스 내보내기")
    parser.add_argument("--data-dir", "-d", default="/home/kimghw/glim/rosbag_data", help="bag 파일 디렉토리")

    args = parser.parse_args()

    if args.interactive:
        interactive_search()
    elif args.export:
        engine = BagSearchEngine(args.data_dir)
        engine.export_for_rag()
    elif args.query:
        engine = BagSearchEngine(args.data_dir)
        results = engine.advanced_search(args.query)

        if results:
            print(f"\n검색 결과 ({len(results)}개):")
            for r in results[:5]:
                print(f"\n{engine.get_summary(r['path'])}")
                print(f"  점수: {r['score']}")
                print(f"  매칭: {', '.join(r['matched_fields'])}")
        else:
            print("검색 결과가 없습니다.")
    else:
        parser.print_help()