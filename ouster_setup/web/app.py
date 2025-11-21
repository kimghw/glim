#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Ouster 라이다 웹 제어 대시보드
- 센서 상태 확인
- 데이터 녹화 시작/중지
- 녹화된 데이터 재생
"""

import os
import sys
import json
import subprocess
import signal
import time
import threading
from datetime import datetime
from pathlib import Path
from flask import Flask, render_template, jsonify, request
from flask_cors import CORS
import psutil

# 메타데이터 매니저 import
sys.path.append(str(Path(__file__).parent.parent / 'scripts' / 'metadata'))
from metadata_manager import RosbagMetadata

app = Flask(__name__)
CORS(app)

# 설정
BASE_DIR = Path(__file__).parent.parent
SCRIPTS_DIR = BASE_DIR / 'scripts'
DATA_DIR = Path('/home/kimghw/glim/rosbag_data')
SENSOR_IP = '192.168.10.10'
ROS_SETUP = '/opt/ros/jazzy/setup.bash'
ROS_PYTHON = '/opt/ros/jazzy/bin/python3'

# 전역 프로세스 관리
recording_process = None
driver_process = None
replay_process = None
CAPTURE_JSON_PATH = Path('/tmp/latest_capture.json')
CAPTURE_LOCK = threading.Lock()
RUNNING_STATUSES = {
    psutil.STATUS_RUNNING,
    psutil.STATUS_SLEEPING,
    psutil.STATUS_DISK_SLEEP,
    getattr(psutil, "STATUS_WAKING", "waking"),
    getattr(psutil, "STATUS_IDLE", "idle"),
}


def run_command(cmd, background=False, timeout=10, clear_venv=False):
    """명령 실행"""
    try:
        # 가상환경 제거 옵션
        env = os.environ.copy()
        if clear_venv:
            # 가상환경 관련 환경변수 제거
            env.pop('VIRTUAL_ENV', None)
            env.pop('VIRTUAL_ENV_PROMPT', None)
            # PATH에서 venv 경로 제거
            if 'PATH' in env:
                paths = env['PATH'].split(':')
                paths = [p for p in paths if 'venv' not in p]
                env['PATH'] = ':'.join(paths)

        if background:
            process = subprocess.Popen(
                cmd,
                shell=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                executable='/bin/bash',
                preexec_fn=os.setsid,
                env=env
            )
            return process
        else:
            result = subprocess.run(
                cmd,
                shell=True,
                capture_output=True,
                text=True,
                timeout=timeout,
                executable='/bin/bash',
                env=env
            )
            return result
    except Exception as e:
        return None


def list_active_pids(patterns):
    """주어진 패턴과 일치하는 실행 중인 PID 목록 반환"""
    matched = []
    for proc in psutil.process_iter(['pid', 'cmdline', 'status']):
        try:
            if proc.info['status'] not in RUNNING_STATUSES:
                continue
            cmdline = ' '.join(proc.info.get('cmdline') or [])
            if any(pattern in cmdline for pattern in patterns):
                matched.append(proc.pid)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    return matched


def terminate_process_tree(pid, sig=signal.SIGINT, timeout=5):
    """프로세스 트리를 종료"""
    try:
        proc = psutil.Process(pid)
    except psutil.NoSuchProcess:
        return False

    procs = [proc] + proc.children(recursive=True)

    for p in procs:
        try:
            p.send_signal(sig)
        except psutil.NoSuchProcess:
            continue

    gone, alive = psutil.wait_procs(procs, timeout=timeout)

    for p in alive:
        try:
            p.kill()
        except psutil.NoSuchProcess:
            continue

    return True


def check_sensor_status():
    """센서 상태 확인"""
    try:
        # Ping 확인
        ping_cmd = f'ping -c 1 -W 1 {SENSOR_IP}'
        ping_result = run_command(ping_cmd)
        ping_ok = ping_result.returncode == 0 if ping_result else False

        # API 확인
        api_cmd = f'curl -s --max-time 2 http://{SENSOR_IP}/api/v1/sensor/metadata'
        api_result = run_command(api_cmd)

        if api_result and api_result.returncode == 0:
            try:
                metadata = json.loads(api_result.stdout)
                sensor_info = metadata.get('sensor_info', {})

                return {
                    'status': 'online',
                    'ping': ping_ok,
                    'model': sensor_info.get('prod_line', 'Unknown'),
                    'serial': sensor_info.get('prod_sn', 'Unknown'),
                    'firmware': sensor_info.get('build_rev', 'Unknown'),
                    'sensor_status': sensor_info.get('status', 'Unknown'),
                    'ip': SENSOR_IP
                }
            except:
                pass

        return {
            'status': 'offline' if not ping_ok else 'degraded',
            'ping': ping_ok,
            'model': 'N/A',
            'serial': 'N/A',
            'firmware': 'N/A',
            'sensor_status': 'N/A',
            'ip': SENSOR_IP
        }
    except Exception as e:
        return {
            'status': 'error',
            'error': str(e),
            'ip': SENSOR_IP
        }


def check_ros_topics():
    """ROS2 토픽 확인"""
    try:
        # 모든 토픽 확인 (ouster와 os_cloud_node 둘 다)
        cmd = f'source {ROS_SETUP} && ros2 topic list | grep -E "(ouster|os_cloud_node)"'
        result = run_command(cmd)

        if result and result.returncode == 0:
            topics = result.stdout.strip().split('\n')
            return {
                'available': True,
                'topics': topics,
                'count': len(topics)
            }
        return {'available': False, 'topics': [], 'count': 0}
    except:
        return {'available': False, 'topics': [], 'count': 0}


def get_recording_files():
    """녹화된 파일 목록"""
    try:
        DATA_DIR.mkdir(parents=True, exist_ok=True)
        bags = []

        for bag_dir in DATA_DIR.iterdir():
            if bag_dir.is_dir() and not bag_dir.name.startswith('.'):
                metadata_file = bag_dir / 'metadata.yaml'
                # ROS2 Jazzy uses .mcap format, older versions use .db3
                bag_file = list(bag_dir.glob('*.mcap')) + list(bag_dir.glob('*.db3'))

                if bag_file:
                    stat = bag_dir.stat()
                    size = sum(f.stat().st_size for f in bag_dir.rglob('*') if f.is_file())

                    # bag 파일의 duration 정보 가져오기
                    duration = 'N/A'
                    try:
                        cmd = f'source {ROS_SETUP} && ros2 bag info {bag_dir} 2>/dev/null | grep Duration'
                        result = run_command(cmd, timeout=5)
                        if result and result.returncode == 0:
                            # Duration: 123.45s 형태에서 숫자 추출
                            import re
                            match = re.search(r'Duration:\s+([\d.]+)s', result.stdout)
                            if match:
                                dur_sec = float(match.group(1))
                                # 초를 MM:SS 형식으로 변환
                                minutes = int(dur_sec // 60)
                                seconds = int(dur_sec % 60)
                                duration = f'{minutes:02d}:{seconds:02d}'
                    except:
                        pass

                    bags.append({
                        'name': bag_dir.name,
                        'path': str(bag_dir),
                        'size': f'{size / 1024 / 1024:.1f} MB',
                        'created': datetime.fromtimestamp(stat.st_ctime).strftime('%Y-%m-%d %H:%M:%S'),
                        'duration': duration
                    })

        return sorted(bags, key=lambda x: x['created'], reverse=True)
    except Exception as e:
        return []


@app.route('/')
def index():
    """메인 페이지"""
    return render_template('index.html')


@app.route('/api/status')
def api_status():
    """전체 상태 확인"""
    sensor = check_sensor_status()
    topics = check_ros_topics()
    files = get_recording_files()

    global recording_process, driver_process, replay_process

    # 실제 프로세스 확인으로 드라이버 상태 감지
    driver_pids = list_active_pids(['os_driver', 'ouster_ros', 'driver.launch.py'])
    driver_pid = driver_pids[0] if driver_pids else None
    driver_active = bool(driver_pids)

    # 내부 변수와 실제 프로세스 둘 다 확인
    if not driver_active and driver_process is not None and driver_process.poll() is None:
        driver_active = True
        driver_pid = driver_process.pid

    # 실제 프로세스 확인으로 녹화 상태 감지
    recording_pids = list_active_pids(['ros2 bag record'])
    recording_pid = recording_pids[0] if recording_pids else None
    recording_active = bool(recording_pids)

    # 내부 변수와 실제 프로세스 둘 다 확인
    if not recording_active and recording_process is not None and recording_process.poll() is None:
        recording_active = True
        recording_pid = recording_process.pid

    # 재생 프로세스 확인
    replay_pids = list_active_pids(['ros2 bag play'])
    replay_active = bool(replay_pids)
    replay_pid = replay_pids[0] if replay_pids else None
    if not replay_active and replay_process is not None and replay_process.poll() is None:
        replay_active = True
        replay_pid = replay_process.pid

    return jsonify({
        'sensor': sensor,
        'ros_topics': topics,
        'recording': {
            'active': recording_active,
            'pid': recording_pid
        },
        'driver': {
            'active': driver_active,
            'pid': driver_pid
        },
        'replay': {
            'active': replay_active,
            'pid': replay_pid
        },
        'files': files,
        'timestamp': datetime.now().isoformat()
    })


@app.route('/api/driver/start', methods=['POST'])
def start_driver():
    """드라이버 시작"""
    global driver_process

    active_driver_pids = list_active_pids(['os_driver', 'ouster_ros', 'driver.launch.py'])
    if active_driver_pids:
        driver_process = None  # 이미 실행 중인 프로세스를 재사용
        return jsonify({
            'success': True,
            'message': f'드라이버가 이미 실행 중입니다. (PID: {active_driver_pids[0]})',
            'pid': active_driver_pids[0],
            'reused': True
        })

    try:
        # run_driver.sh 스크립트 사용 (드라이버 재사용 로직 포함)
        script_path = SCRIPTS_DIR / 'run_driver.sh'
        cmd = f'{script_path}'

        driver_process = run_command(cmd, background=True)
        time.sleep(3)  # 드라이버 초기화 대기

        return jsonify({
            'success': True,
            'message': '드라이버를 시작했습니다.',
            'pid': driver_process.pid if driver_process else None
        })
    except Exception as e:
        return jsonify({'success': False, 'message': f'오류: {str(e)}'})


@app.route('/api/driver/stop', methods=['POST'])
def stop_driver():
    """드라이버 중지"""
    global driver_process

    driver_pids = list_active_pids(['os_driver', 'ouster_ros', 'driver.launch.py'])
    messages = []

    # 내부 변수도 확인
    if driver_process and driver_process.poll() is None:
        try:
            terminate_process_tree(driver_process.pid)
            messages.append(f'웹에서 시작한 드라이버 종료 (PID: {driver_process.pid})')
        except Exception:
            pass
        driver_process = None

    # 실제 프로세스들 종료
    if driver_pids:
        for pid in driver_pids:
            if driver_process and driver_process.pid == pid:
                continue
            if terminate_process_tree(pid):
                messages.append(f'드라이버 프로세스 종료 (PID: {pid})')

    if messages:
        time.sleep(1)
        return jsonify({'success': True, 'message': '드라이버를 중지했습니다.', 'details': messages})

    return jsonify({'success': False, 'message': '실행 중인 드라이버가 없습니다.'})


@app.route('/api/record/start', methods=['POST'])
def start_recording():
    """녹화 시작"""
    global recording_process

    # 이미 녹화 중인지 확인 (프로세스로 직접 확인)
    active_recording = list_active_pids(['ros2 bag record', 'record_ouster.sh'])
    if active_recording:
        return jsonify({
            'success': False,
            'message': f'이미 녹화 중입니다. (PID: {active_recording[0]})',
            'pid': active_recording[0]
        })

    try:
        data = request.json or {}
        filename = data.get('filename', '')
        duration = data.get('duration', '')
        purpose = data.get('purpose', '')
        record_all = data.get('record_all', False)
        with_viz = data.get('with_viz', False)

        # 파일명이 없으면 자동생성 (타임스탬프)
        if not filename:
            filename = f"ouster_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

        # 녹화 스크립트 사용
        script_path = SCRIPTS_DIR / 'record_ouster.sh'

        # 스크립트에 인수 전달
        args = []

        if filename:
            args.append(filename)

        if duration:
            # 파일명이 없으면 빈 문자열로 처리
            if not filename:
                args.append('""')
            args.append(str(duration))

        if record_all:
            args.append('--all')

        if with_viz:
            args.append('--with-viz')

        # 명령어 구성
        if args:
            cmd = f'{script_path} {" ".join(args)}'
        else:
            cmd = f'{script_path}'

        recording_process = run_command(cmd, background=True)
        time.sleep(2)

        if recording_process and recording_process.poll() is None:
            # 녹화가 시작되면 목적(purpose)을 포함한 메타데이터를 즉시 생성
            if purpose:
                try:
                    bag_path = DATA_DIR / filename

                    # 메타데이터 객체 생성
                    metadata = RosbagMetadata(str(bag_path))
                    metadata.add_experiment_info(purpose=purpose)
                    metadata.add_experiment_info(operator="웹 인터페이스")

                    # 시간 정보는 자동으로 설정됨
                    # 파일이 아직 생성되지 않았을 수 있으므로 기본 정보만 저장
                    metadata.save_metadata()

                except Exception as e:
                    print(f"메타데이터 저장 실패: {e}")

            return jsonify({
                'success': True,
                'message': f'녹화를 시작했습니다',
                'filename': filename,
                'purpose': purpose,
                'pid': recording_process.pid
            })
        else:
            recording_process = None
            return jsonify({'success': False, 'message': '녹화 프로세스 시작 실패'})

    except Exception as e:
        recording_process = None
        return jsonify({'success': False, 'message': f'오류: {str(e)}'})


@app.route('/api/record/stop', methods=['POST'])
def stop_recording():
    """녹화 중지"""
    global recording_process

    # 모든 녹화 관련 프로세스 찾기 (ros2 bag record 및 record_ouster.sh)
    killed_any = False
    messages = []

    try:
        target_pids = set(list_active_pids(['record_ouster.sh', 'ros2 bag record']))

        # 웹서버 내부 변수도 확인
        if recording_process and recording_process.poll() is None:
            target_pids.add(recording_process.pid)

        for pid in target_pids:
            if terminate_process_tree(pid):
                killed_any = True
                messages.append(f'녹화 프로세스 {pid} 종료')

        recording_process = None

        if killed_any:
            time.sleep(1)
            return jsonify({
                'success': True,
                'message': '녹화를 중지했습니다.',
                'details': messages
            })
        else:
            return jsonify({
                'success': False,
                'message': '실행 중인 녹화가 없습니다.'
            })

    except Exception as e:
        recording_process = None
        return jsonify({'success': False, 'message': f'오류: {str(e)}'})


@app.route('/api/replay/start', methods=['POST'])
def start_replay():
    """재생 시작"""
    global replay_process

    active_replay = list_active_pids(['ros2 bag play'])
    if active_replay:
        replay_process = None
        return jsonify({'success': False, 'message': f'이미 재생 중입니다. (PID: {active_replay[0]})', 'pid': active_replay[0]})

    try:
        data = request.json or {}
        bag_path = data.get('bag_path')
        use_rviz = data.get('use_rviz', True)  # 기본값은 RViz 사용
        loop = data.get('loop', False)  # 반복 재생 옵션

        if not bag_path or not Path(bag_path).exists():
            return jsonify({'success': False, 'message': '유효하지 않은 파일 경로입니다.'})

        # replay_ouster.sh 스크립트 사용
        script_path = SCRIPTS_DIR / 'replay_ouster.sh'

        # 명령어 구성
        cmd = f'{script_path} {bag_path}'

        # RViz 옵션
        if not use_rviz:
            cmd += ' --no-viz'

        # 반복 재생 옵션
        if loop:
            cmd += ' --loop'

        replay_process = run_command(cmd, background=True)
        time.sleep(2)

        return jsonify({
            'success': True,
            'message': f'재생을 시작했습니다: {Path(bag_path).name}',
            'pid': replay_process.pid,
            'options': {
                'rviz': use_rviz,
                'loop': loop
            }
        })
    except Exception as e:
        return jsonify({'success': False, 'message': f'오류: {str(e)}'})


@app.route('/api/capture', methods=['GET'])
def capture_pointcloud():
    """Near-IR 이미지 캡처"""
    if not CAPTURE_LOCK.acquire(blocking=False):
        return jsonify({'success': False, 'message': '다른 캡처 작업이 진행 중입니다. 잠시 후 다시 시도하세요.'})

    try:
        # 드라이버가 실행 중인지만 확인 (녹화 여부와 무관하게)
        topics = check_ros_topics()
        if not topics['available'] or topics['count'] == 0:
            return jsonify({
                'success': False,
                'message': '라이다 토픽을 찾을 수 없습니다. "Start Recording"을 누르면 드라이버가 자동으로 시작됩니다.'
            })

        # 이전 결과 파일 삭제 (stale 데이터 방지)
        try:
            if CAPTURE_JSON_PATH.exists():
                CAPTURE_JSON_PATH.unlink()
        except Exception:
            pass

        # 캡처 스크립트 실행 (ROS2 환경 소싱 필요)
        # 가상환경 비활성화 후 ROS2 환경만 사용
        script_path = SCRIPTS_DIR / 'capture' / 'capture_pointcloud.py'
        cmd = f'bash -c "source {ROS_SETUP} && timeout 15 python3 {script_path}"'
        result = run_command(cmd, timeout=20, clear_venv=True)

        # 결과 확인
        if not result:
            return jsonify({'success': False, 'message': '캡처 실행에 실패했습니다.'})

        # JSON 파일이 생성되었는지 확인
        time.sleep(0.5)  # 파일 생성 대기

        if not CAPTURE_JSON_PATH.exists():
            # 상세 오류 제공
            stderr_msg = (result.stderr or '').strip() if hasattr(result, 'stderr') else ''
            stdout_msg = (result.stdout or '').strip() if hasattr(result, 'stdout') else ''
            diag = stderr_msg or stdout_msg or '캡처 스크립트가 정상적으로 종료되지 않았습니다.'

            # 전체 에러 메시지 로깅
            print(f"=== CAPTURE ERROR DEBUG ===")
            print(f"STDERR: {stderr_msg}")
            print(f"STDOUT: {stdout_msg}")
            print(f"RETURNCODE: {getattr(result, 'returncode', None)}")
            print(f"==========================")

            return jsonify({
                'success': False,
                'message': f'캡처 실패: {diag[:500]}',
                'returncode': getattr(result, 'returncode', None),
                'stderr': stderr_msg[:500],
                'stdout': stdout_msg[:500]
            })

        with open(CAPTURE_JSON_PATH, 'r') as f:
            img_dict = json.load(f)

        capture_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        return jsonify({
            'success': True,
            'message': f'Near-IR 이미지를 캡처했습니다. ({capture_time})',
            'image': f'data:image/png;base64,{img_dict["nearir"]}',
            'timestamp': capture_time
        })
    except Exception as e:
        return jsonify({'success': False, 'message': f'오류: {str(e)}'})
    finally:
        CAPTURE_LOCK.release()


@app.route('/api/replay/stop', methods=['POST'])
def stop_replay():
    """재생 중지"""
    global replay_process

    target_pids = set(list_active_pids(['ros2 bag play']))
    if replay_process and replay_process.poll() is None:
        target_pids.add(replay_process.pid)

    if not target_pids:
        replay_process = None
        return jsonify({'success': False, 'message': '재생 중이 아닙니다.'})

    try:
        for pid in target_pids:
            terminate_process_tree(pid)
        replay_process = None

        return jsonify({'success': True, 'message': '재생을 중지했습니다.'})
    except Exception as e:
        return jsonify({'success': False, 'message': f'오류: {str(e)}'})


@app.route('/api/files/info', methods=['POST'])
def get_file_info():
    """bag 파일 정보 조회"""
    try:
        data = request.json or {}
        bag_path = data.get('bag_path')

        if not bag_path or not Path(bag_path).exists():
            return jsonify({'success': False, 'message': '유효하지 않은 파일 경로입니다.'})

        cmd = f'source {ROS_SETUP} && ros2 bag info {bag_path}'
        result = run_command(cmd)

        if result and result.returncode == 0:
            info = result.stdout
            # 간단한 파싱
            lines = info.split('\n')
            parsed_info = {}
            for line in lines:
                if 'Duration:' in line:
                    parsed_info['duration'] = line.split('Duration:')[1].strip()
                elif 'Messages:' in line:
                    parsed_info['messages'] = line.split('Messages:')[1].strip()
                elif 'Bag size:' in line:
                    parsed_info['size'] = line.split('Bag size:')[1].strip()

            return jsonify({
                'success': True,
                'info': info,
                'parsed': parsed_info,
                'path': bag_path
            })
        else:
            return jsonify({'success': False, 'message': 'bag 정보를 가져올 수 없습니다.'})
    except Exception as e:
        return jsonify({'success': False, 'message': f'오류: {str(e)}'})


@app.route('/api/files/delete', methods=['POST'])
def delete_file():
    """파일 삭제"""
    try:
        data = request.json or {}
        bag_path = data.get('bag_path')

        if not bag_path:
            return jsonify({'success': False, 'message': '파일 경로가 없습니다.'})

        bag_dir = Path(bag_path)
        if bag_dir.exists() and bag_dir.is_dir():
            import shutil
            shutil.rmtree(bag_dir)
            return jsonify({'success': True, 'message': f'{bag_dir.name}을(를) 삭제했습니다.'})
        else:
            return jsonify({'success': False, 'message': '파일을 찾을 수 없습니다.'})
    except Exception as e:
        return jsonify({'success': False, 'message': f'오류: {str(e)}'})


@app.route('/api/metadata/<bag_name>', methods=['GET'])
def get_metadata(bag_name):
    """bag 파일의 메타데이터 조회"""
    try:
        bag_path = DATA_DIR / bag_name
        if not bag_path.exists():
            return jsonify({'success': False, 'message': 'Bag 파일이 없습니다.'})

        # 메타데이터 파일 찾기
        json_meta = bag_path / 'rich_metadata.json'
        yaml_meta = bag_path / 'rich_metadata.yaml'

        if json_meta.exists():
            with open(json_meta, 'r', encoding='utf-8') as f:
                metadata = json.load(f)
                return jsonify({
                    'success': True,
                    'metadata': metadata,
                    'bag_name': bag_name
                })
        elif yaml_meta.exists():
            import yaml
            with open(yaml_meta, 'r', encoding='utf-8') as f:
                metadata = yaml.safe_load(f)
                return jsonify({
                    'success': True,
                    'metadata': metadata,
                    'bag_name': bag_name
                })
        else:
            # 메타데이터가 없으면 기본 생성
            metadata_obj = RosbagMetadata(str(bag_path))
            metadata_obj.analyze_bag_file()
            return jsonify({
                'success': True,
                'metadata': metadata_obj.metadata,
                'bag_name': bag_name,
                'is_new': True
            })

    except Exception as e:
        return jsonify({'success': False, 'message': f'오류: {str(e)}'})


@app.route('/api/metadata/<bag_name>', methods=['POST'])
def update_metadata(bag_name):
    """bag 파일의 메타데이터 업데이트"""
    try:
        bag_path = DATA_DIR / bag_name
        if not bag_path.exists():
            return jsonify({'success': False, 'message': 'Bag 파일이 없습니다.'})

        data = request.json or {}

        # 기존 메타데이터 로드 또는 새로 생성
        metadata_obj = RosbagMetadata(str(bag_path))

        # 기존 메타데이터가 있으면 로드
        json_meta = bag_path / 'rich_metadata.json'
        if json_meta.exists():
            metadata_obj.load_metadata(str(json_meta))

        # 실험/프로젝트 정보 업데이트
        if 'experiment' in data:
            for key, value in data['experiment'].items():
                if key == 'tags' and isinstance(value, list):
                    metadata_obj.add_tags(value)
                elif key == 'notes':
                    metadata_obj.add_notes(value)
                else:
                    metadata_obj.add_experiment_info(**{key: value})

        # 위치 정보 업데이트
        if 'location' in data:
            metadata_obj.add_location(**data['location'])

        # 환경 정보 업데이트
        if 'environment' in data:
            if 'weather' in data['environment']:
                metadata_obj.add_weather(**data['environment']['weather'])
            if 'lighting' in data['environment']:
                metadata_obj.metadata['environment']['lighting'] = data['environment']['lighting']
            if 'surface_type' in data['environment']:
                metadata_obj.metadata['environment']['surface_type'] = data['environment']['surface_type']

        # bag 파일 분석 (아직 안 되어 있으면)
        if not metadata_obj.metadata['data_characteristics']['duration_seconds']:
            metadata_obj.analyze_bag_file()

        # 메타데이터 저장
        output_path = metadata_obj.save_metadata()

        return jsonify({
            'success': True,
            'message': '메타데이터가 업데이트되었습니다.',
            'metadata': metadata_obj.metadata,
            'saved_to': str(output_path)
        })

    except Exception as e:
        return jsonify({'success': False, 'message': f'오류: {str(e)}'})


if __name__ == '__main__':
    print("=" * 60)
    print("  Ouster 라이다 웹 대시보드")
    print("=" * 60)
    print(f"  접속 주소: http://localhost:5001")
    print(f"  센서 IP: {SENSOR_IP}")
    print("=" * 60)

    app.run(host='0.0.0.0', port=5001, debug=True, use_reloader=False)
