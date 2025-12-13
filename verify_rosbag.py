#!/usr/bin/env python3

import os
import sys
import subprocess
import json
from datetime import datetime
from pathlib import Path
import argparse

class RosbagVerifier:
    def __init__(self, bag_path):
        self.bag_path = Path(bag_path)
        self.results = {
            'file_info': {},
            'topics': {},
            'errors': [],
            'warnings': []
        }

    def verify(self):
        """Main verification method"""
        print(f"\n{'='*60}")
        print(f"Verifying ROS bag: {self.bag_path}")
        print(f"{'='*60}\n")

        # Check file existence and readability
        if not self._check_file_exists():
            return False

        # Get bag info
        if not self._get_bag_info():
            return False

        # Verify required topics
        if not self._verify_topics():
            return False

        # Check data integrity
        if not self._check_integrity():
            return False

        # Generate report
        self._generate_report()

        return len(self.results['errors']) == 0

    def _check_file_exists(self):
        """Check if bag file/directory exists and is readable"""
        print("1. Checking bag existence...")

        if not self.bag_path.exists():
            self.results['errors'].append(f"Path does not exist: {self.bag_path}")
            print(f"   ❌ Path not found")
            return False

        # ROS2 bags can be either a single .db3 file or a directory with metadata.yaml
        if self.bag_path.is_file():
            # Single file bag
            if not str(self.bag_path).endswith('.db3'):
                self.results['warnings'].append(f"File may not be a ROS2 bag: {self.bag_path}")

            file_size = self.bag_path.stat().st_size
            self.results['file_info']['size_bytes'] = file_size
            self.results['file_info']['size_mb'] = round(file_size / (1024*1024), 2)

            if file_size == 0:
                self.results['errors'].append("File is empty")
                print(f"   ❌ File is empty")
                return False

            print(f"   ✓ Bag file exists ({self.results['file_info']['size_mb']} MB)")

        elif self.bag_path.is_dir():
            # Directory-based bag (ROS2 format)
            metadata_file = self.bag_path / 'metadata.yaml'
            if not metadata_file.exists():
                self.results['errors'].append(f"Not a valid ROS2 bag directory (missing metadata.yaml): {self.bag_path}")
                print(f"   ❌ Invalid bag directory")
                return False

            # Calculate total size of directory
            total_size = sum(f.stat().st_size for f in self.bag_path.rglob('*') if f.is_file())
            self.results['file_info']['size_bytes'] = total_size
            self.results['file_info']['size_mb'] = round(total_size / (1024*1024), 2)

            # Check for data files (.db3 or .mcap) in directory
            db3_files = list(self.bag_path.glob('*.db3'))
            mcap_files = list(self.bag_path.glob('*.mcap'))
            data_files = db3_files + mcap_files

            if not data_files:
                self.results['errors'].append("No data files (.db3 or .mcap) found in bag directory")
                print(f"   ❌ No data files in bag directory")
                return False

            self.results['file_info']['data_file_count'] = len(data_files)
            self.results['file_info']['data_file_type'] = 'mcap' if mcap_files else 'db3'
            print(f"   ✓ Bag directory exists ({self.results['file_info']['size_mb']} MB, {len(data_files)} {self.results['file_info']['data_file_type']} files)")

        else:
            self.results['errors'].append(f"Path is neither file nor directory: {self.bag_path}")
            print(f"   ❌ Invalid path type")
            return False

        return True

    def _get_bag_info(self):
        """Get detailed bag information using ros2 bag info"""
        print("\n2. Getting bag information...")

        try:
            # Run ros2 bag info command
            cmd = ['ros2', 'bag', 'info', str(self.bag_path)]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)

            if result.returncode != 0:
                self.results['errors'].append(f"Failed to read bag info: {result.stderr}")
                print(f"   ❌ Cannot read bag info")
                return False

            # Parse the output
            output = result.stdout
            self.results['file_info']['raw_info'] = output

            # Extract key information
            lines = output.split('\n')
            for line in lines:
                if 'Duration:' in line:
                    duration = line.split('Duration:')[1].strip()
                    self.results['file_info']['duration'] = duration
                elif 'Start:' in line:
                    start_time = line.split('Start:')[1].strip()
                    self.results['file_info']['start_time'] = start_time
                elif 'End:' in line:
                    end_time = line.split('End:')[1].strip()
                    self.results['file_info']['end_time'] = end_time
                elif 'Messages:' in line and 'Topic information:' not in line:
                    msg_count = line.split('Messages:')[1].strip()
                    self.results['file_info']['total_messages'] = msg_count

            print(f"   ✓ Bag info retrieved")
            print(f"     Duration: {self.results['file_info'].get('duration', 'N/A')}")
            print(f"     Messages: {self.results['file_info'].get('total_messages', 'N/A')}")

            return True

        except subprocess.TimeoutExpired:
            self.results['errors'].append("Timeout while reading bag info")
            print(f"   ❌ Timeout")
            return False
        except Exception as e:
            self.results['errors'].append(f"Error reading bag info: {str(e)}")
            print(f"   ❌ Error: {str(e)}")
            return False

    def _verify_topics(self):
        """Verify important topics and message counts"""
        print("\n3. Verifying topics...")

        # Define critical topics to check
        critical_topics = [
            '/ouster/points',
            '/ouster/range_image',
            '/ouster/imu',
            '/microstrain/imu/data',
            '/microstrain/nav/odom',
            '/microstrain/nav/heading'
        ]

        # Parse topic information from raw info
        raw_info = self.results['file_info'].get('raw_info', '')
        topic_section = False

        for line in raw_info.split('\n'):
            if 'Topic information:' in line:
                topic_section = True
                continue
            if topic_section and line.strip() and 'Topic:' in line:
                # Parse topic line
                parts = line.strip().split('|')
                if len(parts) >= 3:
                    topic_name = parts[0].replace('Topic:', '').strip()
                    msg_count = parts[1].strip().split()[0]
                    msg_type = parts[2].replace('Type:', '').strip()

                    self.results['topics'][topic_name] = {
                        'count': msg_count,
                        'type': msg_type
                    }

        # Check critical topics
        missing_topics = []
        low_message_topics = []

        for topic in critical_topics:
            if topic in self.results['topics']:
                count = self.results['topics'][topic]['count']
                print(f"   ✓ {topic}: {count} messages")

                # Warn if message count seems low
                try:
                    if int(count) < 10:
                        low_message_topics.append(topic)
                        self.results['warnings'].append(f"Low message count for {topic}: {count}")
                except:
                    pass
            else:
                missing_topics.append(topic)
                print(f"   ⚠ {topic}: Not found")

        # Report other topics found
        other_topics = [t for t in self.results['topics'] if t not in critical_topics]
        if other_topics:
            print("\n   Other topics found:")
            for topic in other_topics[:10]:  # Show first 10
                count = self.results['topics'][topic]['count']
                print(f"     • {topic}: {count} messages")
            if len(other_topics) > 10:
                print(f"     ... and {len(other_topics)-10} more")

        if missing_topics:
            self.results['warnings'].append(f"Missing topics: {', '.join(missing_topics)}")

        return True

    def _check_integrity(self):
        """Check data integrity and consistency"""
        print("\n4. Checking data integrity...")

        try:
            # Try to play the bag briefly to check if it's corrupted
            cmd = ['timeout', '2', 'ros2', 'bag', 'play', str(self.bag_path), '--rate', '100']
            result = subprocess.run(cmd, capture_output=True, text=True)

            # timeout returns 124 when it times out (which is expected)
            if result.returncode not in [0, 124]:
                if 'error' in result.stderr.lower() or 'corrupt' in result.stderr.lower():
                    self.results['errors'].append(f"Bag may be corrupted: {result.stderr}")
                    print(f"   ❌ Integrity check failed")
                    return False

            print(f"   ✓ Bag integrity verified")

            # Check for time consistency
            if 'duration' in self.results['file_info']:
                duration_str = self.results['file_info']['duration']
                # Parse duration (format: "X.Ys" or "Xm Y.Zs")
                if 's' in duration_str:
                    if 'm' in duration_str:
                        # Format: "Xm Y.Zs"
                        parts = duration_str.split('m')
                        minutes = float(parts[0])
                        seconds = float(parts[1].replace('s', '').strip())
                        total_seconds = minutes * 60 + seconds
                    else:
                        # Format: "X.Ys"
                        total_seconds = float(duration_str.replace('s', '').strip())

                    if total_seconds < 1:
                        self.results['warnings'].append(f"Very short recording: {duration_str}")
                        print(f"   ⚠ Very short recording duration")
                    elif total_seconds > 3600:
                        self.results['warnings'].append(f"Very long recording: {duration_str}")
                        print(f"   ⚠ Very long recording duration")

            return True

        except Exception as e:
            self.results['warnings'].append(f"Could not perform full integrity check: {str(e)}")
            print(f"   ⚠ Partial integrity check: {str(e)}")
            return True

    def _generate_report(self):
        """Generate final verification report"""
        print(f"\n{'='*60}")
        print("VERIFICATION REPORT")
        print(f"{'='*60}\n")

        # Overall status
        if len(self.results['errors']) == 0:
            print("✅ VERIFICATION PASSED")
        else:
            print("❌ VERIFICATION FAILED")

        # Summary
        print(f"\nFile: {self.bag_path.name}")
        print(f"Size: {self.results['file_info'].get('size_mb', 'N/A')} MB")
        print(f"Duration: {self.results['file_info'].get('duration', 'N/A')}")
        print(f"Total Messages: {self.results['file_info'].get('total_messages', 'N/A')}")
        print(f"Topics Found: {len(self.results['topics'])}")

        # Errors
        if self.results['errors']:
            print(f"\n❌ Errors ({len(self.results['errors'])}):")
            for error in self.results['errors']:
                print(f"   • {error}")

        # Warnings
        if self.results['warnings']:
            print(f"\n⚠ Warnings ({len(self.results['warnings'])}):")
            for warning in self.results['warnings']:
                print(f"   • {warning}")

        # Save detailed report
        report_path = self.bag_path.parent / f"{self.bag_path.stem}_verification.json"
        with open(report_path, 'w') as f:
            json.dump(self.results, f, indent=2)
        print(f"\n📄 Detailed report saved to: {report_path}")

        print(f"\n{'='*60}\n")


def main():
    parser = argparse.ArgumentParser(description='Verify ROS2 bag file integrity and contents')
    parser.add_argument('bag_file', help='Path to the ROS2 bag file or directory')
    parser.add_argument('--batch', action='store_true', help='Verify all bags in a directory')

    args = parser.parse_args()

    if args.batch:
        # Batch mode - verify all bags in directory
        bag_dir = Path(args.bag_file)
        if not bag_dir.is_dir():
            print(f"Error: {bag_dir} is not a directory")
            sys.exit(1)

        # Find all bag files
        bag_files = list(bag_dir.glob('*.db3'))
        bag_dirs = [d for d in bag_dir.iterdir() if d.is_dir() and (d / 'metadata.yaml').exists()]

        all_bags = bag_files + bag_dirs

        if not all_bags:
            print(f"No ROS bag files found in {bag_dir}")
            sys.exit(1)

        print(f"Found {len(all_bags)} bag(s) to verify\n")

        results = []
        for bag_path in all_bags:
            verifier = RosbagVerifier(bag_path)
            passed = verifier.verify()
            results.append((bag_path, passed))

        # Summary
        print(f"\n{'='*60}")
        print("BATCH VERIFICATION SUMMARY")
        print(f"{'='*60}\n")

        passed_count = sum(1 for _, p in results if p)
        failed_count = len(results) - passed_count

        print(f"Total bags: {len(results)}")
        print(f"✅ Passed: {passed_count}")
        print(f"❌ Failed: {failed_count}")

        if failed_count > 0:
            print("\nFailed bags:")
            for bag_path, passed in results:
                if not passed:
                    print(f"   • {bag_path.name}")

    else:
        # Single file mode
        verifier = RosbagVerifier(args.bag_file)
        passed = verifier.verify()

        if not passed:
            sys.exit(1)


if __name__ == '__main__':
    main()