# Path Recorder

Records vehicle trajectory from `/pf/pose/odom` and compares it against the global path to calculate lateral error statistics and visualization.

## Overview

The `path_recorder` package helps you analyze how well your vehicle follows the global path by:
- Recording actual vehicle trajectory from odometry
- Calculating lateral error (Frenet d-coordinate) relative to global path
- Computing statistics: average, standard deviation, min, max
- Visualizing recorded path and errors in RViz
- Saving data to CSV files for offline analysis

## Features

- **Real-time lateral error calculation** using Frenet coordinates
- **Live statistics** displayed at 1 Hz (configurable)
- **Advanced RViz visualization**:
  - Direction triangles: Show vehicle heading with velocity-based colors
    - Blue = low speed, Green = medium speed, Red = high speed
  - Colored arrows: Lateral errors (green = small, red = large)
  - Fully configurable triangle size and spacing
- **CSV export** with timestamps and error data
- **Summary statistics** saved to text file on shutdown
- **Memory-safe** with configurable max recording points

## Installation

The package is already part of the F1TENTH DAWGS workspace:

```bash
cd ~/f1tenth_dawgs
colcon build --packages-select path_recorder
source install/setup.bash
```

## Usage

### Basic Usage

```bash
# Launch with default parameters
ros2 launch path_recorder path_recorder.launch.py
```

### Custom Topics

```bash
# Specify custom odometry and global path topics
ros2 launch path_recorder path_recorder.launch.py \
    odom_topic:=/my_odom \
    global_path_topic:=/my_global_path
```

### Without CSV Saving

```bash
# Disable CSV output (statistics only)
ros2 launch path_recorder path_recorder.launch.py \
    save_to_csv:=false
```

### Direct Node Run

```bash
# Run node directly with custom parameters
ros2 run path_recorder path_recorder_node \
    --ros-args \
    -p odom_topic:=/pf/pose/odom \
    -p global_path_topic:=/global_centerline \
    -p recording_enabled:=true
```

## Offline Analysis

The offline analysis tool generates comprehensive matplotlib visualizations comparing recorded trajectories with the global centerline, **overlaid on the map image**. Reads directly from CSV and YAML files.

### Running Offline Analysis

```bash
# Basic usage (with default paths set in launch file)
ros2 launch path_recorder offline_analysis.launch.py

# With custom paths
ros2 launch path_recorder offline_analysis.launch.py \
    global_path_csv:=/path/to/centerline.csv \
    map_yaml:=/path/to/map.yaml

# Analyze specific recording file
ros2 launch path_recorder offline_analysis.launch.py \
    csv_file:=~/f1tenth_recordings/path_recording_20241030_140530.csv

# Headless mode (no GUI, save only)
ros2 launch path_recorder offline_analysis.launch.py \
    show_plots:=false

# Custom output directory
ros2 launch path_recorder offline_analysis.launch.py \
    output_directory:=~/my_analysis
```

### Required Files

1. **Global Path CSV**: Centerline waypoints
   - Format: `x,y,v,kappa` or `x_m,y_m,...`
   - Example: `mohyun_slam_v3_iqp.csv`

2. **Map YAML**: Map metadata (optional but recommended)
   - Standard ROS map_server format
   - Contains: image path, resolution, origin
   - Example: `mohyun_slam_v3.yaml`

3. **Map Image**: PNG/PGM file referenced in YAML
   - Automatically loaded from YAML
   - Example: `mohyun_slam_v3.png`

### Map Overlay Feature

When `map_yaml` is provided, the analysis displays:
- **Map image** as background (grayscale)
- **Global centerline** in green on top of map
- **Recorded trajectory** colored by speed on top of map
- Coordinates automatically converted from world to map pixels

### Analysis Plots

The offline analysis generates 8 comprehensive plots:

1. **Full Trajectory Comparison**: Recorded path vs global centerline (colored by speed)
2. **Zoomed Trajectory**: Close-up view of trajectory start
3. **Lateral Error vs Distance**: Error along the path (Frenet s-coordinate)
4. **Lateral Error vs Time**: Error evolution over time
5. **Lateral Error Histogram**: Distribution of lateral errors
6. **Speed Profile**: Speed over time
7. **Speed vs Error Correlation**: Relationship between speed and lateral error
8. **Statistics Summary**: Comprehensive statistics table

### Statistics Calculated

- **Lateral Error**: Mean, std dev, RMS, abs mean, abs max, range
- **Speed**: Mean, std dev, min, max
- **Performance**: Duration, total distance, average speed, data points

### Example Output

```
Statistics Summary:
  Lateral Error:
    Mean:            2.34 cm
    Std Dev:         3.45 cm
    RMS:             4.12 cm
    Abs Mean:        3.01 cm
    Abs Max:        12.56 cm

  Speed:
    Mean:            3.45 m/s
    Max:             6.78 m/s

  Performance:
    Duration:       45.67 s
    Distance:      157.89 m
    Avg Speed:       3.46 m/s
```

### Output Files

Analysis plots are saved to `~/f1tenth_analysis/` by default with timestamp:
- `analysis_20241030_143022.png` - Comprehensive analysis plot

## Configuration

Edit `config/recorder_params.yaml`:

```yaml
path_recorder_node:
  ros__parameters:
    # Topic configuration
    odom_topic: '/pf/pose/odom'
    global_path_topic: '/global_centerline'

    # Recording configuration
    recording_enabled: true
    save_to_csv: true
    csv_output_dir: '~/f1tenth_recordings'
    max_recording_points: 10000

    # Statistics update rate (Hz)
    stats_update_rate: 1.0

    # Visualization parameters
    triangle_size: 0.15        # Size of direction triangles (meters)
    triangle_spacing: 0.5      # Distance between triangles (meters)
    min_speed: 0.0             # Minimum speed for color mapping (m/s)
    max_speed: 8.0             # Maximum speed for color mapping (m/s)
```

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `odom_topic` | string | `/pf/pose/odom` | Odometry topic to record |
| `global_path_topic` | string | `/global_centerline` | Global path topic for comparison |
| `recording_enabled` | bool | `true` | Enable/disable recording |
| `save_to_csv` | bool | `true` | Save data to CSV file |
| `csv_output_dir` | string | `~/f1tenth_recordings` | Output directory for CSV files |
| `max_recording_points` | int | `10000` | Maximum points to record (memory limit) |
| `stats_update_rate` | float | `1.0` | Statistics display rate in Hz |
| `triangle_size` | float | `0.15` | Size of direction triangles in meters |
| `triangle_spacing` | float | `0.5` | Distance between triangles in meters |
| `min_speed` | float | `0.0` | Minimum speed for color mapping (m/s) |
| `max_speed` | float | `8.0` | Maximum speed for color mapping (m/s) |

## Topics

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/pf/pose/odom` | `nav_msgs/Odometry` | Vehicle odometry (position and velocity) |
| `/global_centerline` | `nav_msgs/Path` | Global path waypoints |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/path_recorder/recorded_path` | `visualization_msgs/Marker` | Direction triangles with velocity colors |
| `/path_recorder/error_visualization` | `visualization_msgs/MarkerArray` | Lateral error arrows |

## Visualization in RViz

Add the following displays in RViz:

1. **Recorded Path (Direction Triangles)**:
   - Type: `Marker`
   - Topic: `/path_recorder/recorded_path`
   - Shows vehicle trajectory as triangles pointing in direction of travel
   - Color coding:
     - **Blue**: Low speed (< 50% of max_speed)
     - **Green**: Medium speed (~50% of max_speed)
     - **Red**: High speed (> 50% of max_speed)
   - Triangle size and spacing controlled by parameters

2. **Error Visualization**:
   - Type: `MarkerArray`
   - Topic: `/path_recorder/error_visualization`
   - Shows arrows from actual position to nearest point on global path
   - Color indicates error magnitude (green = good, red = large error)

### Adjusting Visualization

To make triangles smaller and less obtrusive:
```bash
ros2 param set /path_recorder_node triangle_size 0.10
ros2 param set /path_recorder_node triangle_spacing 0.7
```

Or edit `config/recorder_params.yaml`:
```yaml
triangle_size: 0.10        # Smaller triangles
triangle_spacing: 0.7      # More spacing between them
```

## Output Files

### CSV File Format

Location: `~/f1tenth_recordings/path_recording_YYYYMMDD_HHMMSS.csv`

Columns:
- `timestamp`: Unix timestamp (seconds)
- `x_m`: X position (meters)
- `y_m`: Y position (meters)
- `vx_mps`: X velocity (m/s)
- `vy_mps`: Y velocity (m/s)
- `s_m`: Longitudinal position on global path (Frenet s)
- `d_m (lateral_error)`: Lateral error (Frenet d, meters)
- `abs_lateral_error`: Absolute lateral error (meters)

### Summary File

Location: `~/f1tenth_recordings/path_recording_YYYYMMDD_HHMMSS_summary.txt`

Contains:
```
Path Recording Summary
============================================================
Total points: 5432
Average lateral error: 2.34 cm
Std deviation: 1.56 cm
Min lateral error: -8.21 cm
Max lateral error: 9.87 cm
```

## Example Workflow

### 1. Start Recording During Autonomous Run

```bash
# Terminal 1: Launch your autonomous stack
ros2 launch agent_dawgs autonomous.launch.py

# Terminal 2: Start path recorder
ros2 launch path_recorder path_recorder.launch.py

# Terminal 3: Open RViz
rviz2 -d path_recorder_viz.rviz
```

### 2. Monitor Statistics

Watch the terminal output for real-time statistics:

```
[INFO] [path_recorder]: Lateral Error Stats | Count: 1234 | Avg: 2.34 cm | Std: 1.56 cm | Min: -8.21 cm | Max: 9.87 cm
```

### 3. Stop Recording

Press `Ctrl+C` to stop the node. It will automatically save:
- CSV file with all recorded data
- Summary text file with statistics

### 4. Analyze Data Offline

```python
import pandas as pd
import matplotlib.pyplot as plt

# Load CSV
df = pd.read_csv('~/f1tenth_recordings/path_recording_20251030_123456.csv')

# Plot lateral error over time
plt.figure(figsize=(12, 6))
plt.plot(df['timestamp'], df['d_m (lateral_error)'] * 100)  # Convert to cm
plt.xlabel('Time (s)')
plt.ylabel('Lateral Error (cm)')
plt.title('Lateral Error vs Time')
plt.grid(True)
plt.show()

# Plot histogram
plt.figure(figsize=(10, 6))
plt.hist(df['d_m (lateral_error)'] * 100, bins=50)
plt.xlabel('Lateral Error (cm)')
plt.ylabel('Frequency')
plt.title('Lateral Error Distribution')
plt.grid(True)
plt.show()
```

## Understanding Lateral Error

**Lateral error (d)** is the perpendicular distance from the vehicle's actual position to the nearest point on the global path, calculated using Frenet coordinates:

- **Positive d**: Vehicle is to the right of the path
- **Negative d**: Vehicle is to the left of the path
- **Zero d**: Vehicle is exactly on the path

### Good Performance Indicators

- **Average error < 5 cm**: Excellent tracking
- **Average error < 10 cm**: Good tracking
- **Average error > 15 cm**: Needs tuning

### Common Issues

| Symptom | Possible Cause | Solution |
|---------|---------------|----------|
| Large average error | Poor path tracking gains | Tune pure pursuit lookahead |
| High std deviation | Oscillation around path | Reduce controller gains |
| Consistent positive/negative bias | Vehicle alignment issue | Check steering calibration |
| Errors spike in corners | Too aggressive speed profile | Reduce curvature_gain |

## Integration with Other Tools

### With Path Tracker

```bash
# Launch path tracker and recorder together
ros2 launch agent_dawgs autonomous.launch.py &
ros2 launch path_recorder path_recorder.launch.py
```

### With Parameter Tuning

Use path recorder to evaluate different controller parameters:

```bash
# Test configuration A
ros2 param set /path_tracker lookahead_distance 1.0
# Record for one lap, save as test_A.csv

# Test configuration B
ros2 param set /path_tracker lookahead_distance 1.5
# Record for one lap, save as test_B.csv

# Compare results
```

## Troubleshooting

### "Waiting for global path..."

**Problem**: Node doesn't receive global path.

**Solution**:
```bash
# Check if global path is being published
ros2 topic echo /global_centerline --once

# Check topic names
ros2 topic list | grep global

# Use correct topic name
ros2 launch path_recorder path_recorder.launch.py \
    global_path_topic:=/your_actual_topic
```

### "Failed to initialize FrenetConverter"

**Problem**: Global path has insufficient points or invalid data.

**Solution**:
- Ensure global path has at least 3 waypoints
- Verify waypoints contain valid x, y, psi data
- Check if waypoints form a valid path

### No visualization in RViz

**Problem**: Markers not showing up.

**Solution**:
1. **Check Fixed Frame**: In RViz Global Options, set Fixed Frame to `map`
2. **Add Marker displays**:
   - Click "Add" → "By topic" → `/path_recorder/recorded_path` → Marker
   - Click "Add" → "By topic" → `/path_recorder/error_visualization` → MarkerArray
3. **Verify topics are publishing**:
   ```bash
   ros2 topic hz /path_recorder/recorded_path
   ros2 topic echo /path_recorder/recorded_path --no-arr
   ```
4. **Check node logs**:
   ```bash
   ros2 node info /path_recorder_node
   # Look for warnings like "No triangles generated!"
   ```
5. **Verify parameters**:
   ```bash
   ros2 param list /path_recorder_node
   ros2 param get /path_recorder_node triangle_spacing
   # If too large (>0.5), try smaller value:
   ros2 param set /path_recorder_node triangle_spacing 0.2
   ```
6. **Check if recording is active**:
   ```bash
   ros2 topic echo /path_recorder/recorded_path | head -20
   # Should see marker.points data
   ```

### High memory usage

**Problem**: Too many points recorded.

**Solution**:
- Reduce `max_recording_points` parameter
- Default limit is 10000 points (about 10 minutes at 20 Hz)

## Dependencies

### ROS2 Packages
- `rclpy`: ROS2 Python client library
- `nav_msgs`: Odometry and Path message types
- `geometry_msgs`: Pose message types
- `visualization_msgs`: RViz markers
- `frenet_conversion`: Frenet coordinate conversion

### Python Packages
- `numpy`: Numerical computations
- `scipy`: Interpolation and splines
- `matplotlib`: Plotting and visualization (for offline analysis)
- `pandas`: Data analysis and CSV handling (for offline analysis)
- `pillow` (PIL): Image loading for map overlay
- `pyyaml`: YAML file parsing for map metadata

## Development

### Building

```bash
colcon build --packages-select path_recorder --symlink-install
```

### Testing

```bash
# Run with test data
ros2 launch path_recorder path_recorder.launch.py \
    odom_topic:=/test/odom \
    global_path_topic:=/test/path
```

## Author

F1TENTH DAWGS Team

## License

MIT License

--- korean comments

● 완성했습니다! path_recorder 패키지가 성공적으로 생성되어 빌드되었습니다.

  📦 패키지 개요

  기능

  - /pf/pose/odom 토픽에서 실제 차량 경로 기록
  - /global_centerline 경로와 비교하여 lateral error (Frenet d 좌표) 계산
  - 실시간 통계 (평균, 표준편차, 최소, 최대) 표시
  - RViz 시각화 (실제 경로 + 에러 화살표)
  - CSV 파일로 저장 (상세 데이터 + 요약 통계)

  🚀 사용 방법

  1. 기본 실행

  # 패키지 소싱
  source ~/f1tenth_dawgs/install/setup.bash

  # Launch 파일로 실행
  ros2 launch path_recorder path_recorder.launch.py

  2. 커스텀 토픽 사용

  ros2 launch path_recorder path_recorder.launch.py \
      odom_topic:=/your_odom_topic \
      global_path_topic:=/your_global_path

  3. 실제 주행 중 기록

  # Terminal 1: 자율주행 시작
  ros2 launch agent_dawgs autonomous.launch.py

  # Terminal 2: Path recorder 시작
  ros2 launch path_recorder path_recorder.launch.py

  # 실시간 통계가 1초마다 출력됨:
  # [INFO] Lateral Error Stats | Count: 1234 | Avg: 2.34 cm | Std: 1.56 cm | Min: -8.21 cm | Max: 
  9.87 cm

  📊 출력

  콘솔 출력 (실시간)

  [INFO] Path Recorder Node Started
  [INFO] Odometry topic: /pf/pose/odom
  [INFO] Global path topic: /global_centerline
  [INFO] Waiting for global path...
  [INFO] Global path received!
  [INFO] Path length: 85.34 m
  [INFO] Now recording vehicle trajectory...
  [INFO] Lateral Error Stats | Count: 1234 | Avg: 2.34 cm | Std: 1.56 cm | Min: -8.21 cm | Max: 
  9.87 cm

  CSV 파일 (~/f1tenth_recordings/)

  timestamp,x_m,y_m,vx_mps,vy_mps,s_m,d_m (lateral_error),abs_lateral_error
  1698765432.123,10.5,20.3,2.5,0.1,15.2,0.023,0.023
  1698765432.223,10.6,20.4,2.6,0.1,15.4,0.019,0.019
  ...

  요약 파일 (*_summary.txt)

  Path Recording Summary
  ============================================================
  Total points: 5432
  Average lateral error: 2.34 cm
  Std deviation: 1.56 cm
  Min lateral error: -8.21 cm
  Max lateral error: 9.87 cm

  📈 RViz 시각화

  토픽 추가:

  1. Recorded Path (/path_recorder/recorded_path)
    - Type: Marker
    - 파란색 선으로 실제 주행 경로 표시
  2. Error Visualization (/path_recorder/error_visualization)
    - Type: MarkerArray
    - 실제 위치에서 global path까지 화살표
    - 색상: 녹색 (에러 작음) → 빨간색 (에러 큼)

  🔧 파일 구조

  path_recorder/
  ├── path_recorder/
  │   ├── __init__.py
  │   └── path_recorder_node.py       # 메인 노드 (lateral error 계산)
  ├── launch/
  │   └── path_recorder.launch.py     # Launch 파일
  ├── config/
  │   └── path_recorder_params.yaml   # 파라미터 설정
  ├── package.xml                       # 의존성 정의
  ├── setup.py                          # 설치 설정
  └── README.md                         # 상세 문서

  🎯 주요 기능 상세

  Lateral Error 계산

  - FrenetConverter 사용하여 Frenet 좌표 변환
  - d (lateral error) = 차량 위치에서 global path까지의 수직 거리
  - 양수 = 경로 우측, 음수 = 경로 좌측

  통계

  - 평균 (Average): 전체 에러의 평균값
  - 표준편차 (Std): 에러의 변동성
  - 최소/최대: 가장 작은/큰 에러값

  메모리 관리

  - max_recording_points: 10000 (기본값)
  - 약 10분 분량 (20Hz 기준)
  - 초과 시 오래된 데이터 자동 삭제

  📝 파라미터 설정 (config/path_recorder_params.yaml)

  /**:
    ros__parameters:
      odom_topic: '/pf/pose/odom'           # 오도메트리 토픽
      global_path_topic: '/global_centerline'  # Global path 토픽
      recording_enabled: true                # 기록 활성화
      save_to_csv: true                      # CSV 저장
      csv_output_dir: '~/f1tenth_recordings' # 저장 경로
      max_recording_points: 10000            # 최대 기록 포인트      stats_update_rate: 1.0                 # 통계 출력 주기 (Hz)

  🔍 성능 지표

  우수한 Tracking

  - 평균 lateral error < 5cm
  - 표준편차 < 3cm

  튜닝 필요

  - 평균 lateral error > 15cm
  - 표준편차 > 5cm
  - → Pure pursuit lookahead 거리 조정 필요

  path_recorder 패키지를 통해 path_tracker의 성능을 정량적으로 분석하고 개선할 수 있습니다!

-- kor comment
● 완성했습니다! recorded_path 시각화를 개선하여 속도 기반 색상과 방향 삼각형으로 표시하도록
  수정했습니다.

  🎨 변경 사항

  1. 방향 삼각형 시각화

  - LINE_STRIP → TRIANGLE_LIST로 변경
  - 각 삼각형의 끝점이 차량의 주행 방향을 가리킴
  - 삼각형 간격은 triangle_spacing 파라미터로 조절

  2. 속도 기반 색상 그라디언트

  Blue (파랑) → Green (녹색) → Red (빨강)
  낮은 속도        중간 속도        높은 속도
  - 0% ~ 50% 속도: 파랑 → 녹색
  - 50% ~ 100% 속도: 녹색 → 빨강
  - min_speed, max_speed 파라미터로 범위 설정

  3. 크기 조절 파라미터

  triangle_size: 0.15        # 삼각형 크기 (meters)
  triangle_spacing: 0.5      # 삼각형 간격 (meters)
  min_speed: 0.0             # 최소 속도 (m/s)
  max_speed: 8.0             # 최대 속도 (m/s)

  📝 수정된 파일

  1. path_recorder_node.py

  - 새 파라미터 추가 (line 46-50)
  - _publish_recorded_path_marker() 함수 완전 재작성 (line 293-398)
    - 속도 계산 및 색상 매핑
    - 주행 방향 기반 삼각형 생성
    - 간격 제어 로직

  2. config/recorder_params.yaml

  - Visualization 파라미터 섹션 추가 (line 16-20)

  3. README.md

  - Features 섹션 업데이트 (새로운 시각화 설명)
  - Configuration 섹션 업데이트 (새 파라미터 추가)
  - Visualization 섹션 강화 (색상 의미 설명)
  - 실시간 파라미터 조절 예제 추가

  🚀 사용 방법

  기본 설정으로 실행

  colcon build --packages-select path_recorder --symlink-install
  source install/setup.bash
  ros2 launch path_recorder path_recorder.launch.py

  작게 표시하고 싶다면

  # config/recorder_params.yaml
  triangle_size: 0.10        # 더 작게
  triangle_spacing: 0.7      # 더 넓은 간격

  또는 실행 중에 동적으로 변경:
  ros2 param set /path_recorder_node triangle_size 0.10
  ros2 param set /path_recorder_node triangle_spacing 0.8

  속도 범위 조정

  min_speed: 0.0    # 정지 상태
  max_speed: 10.0   # 최고 속도 (m/s)

  🎯 시각화 효과

  - 삼각형 끝점: 차량이 향하는 방향을 명확히 표시
  - 파란색: 저속 구간 (코너 진입, 감속)
  - 녹색: 중속 구간 (가속 중)
  - 빨간색: 고속 구간 (직선, 최고속)

  이제 global path와 실제 경로를 시각적으로 구별하기 쉽고, 속도 프로파일도 한눈에 파악할 수
  있습니다!

  빌드 후 실행하면 RViz에서 삼각형으로 표시된 실제 주행 경로를 볼 수 있습니다.

--- offline anlysis
● 완료했습니다! offline 분석 노드를 path_recorder 패키지에 추가했습니다.

  생성된 파일

  1. offline_analysis_node.py

  - CSV 리포트 파일을 읽어서 global_centerline과 비교
  - 8가지 종합 플롯 생성:
    a. 전체 경로 비교 (속도 컬러맵)
    b. 확대된 시작 구간
    c. Lateral error vs 거리
    d. Lateral error vs 시간
    e. Lateral error 히스토그램
    f. 속도 프로파일
    g. 속도 vs lateral error 상관관계
    h. 통계 요약 표

  2. offline_analysis.launch.py

  - Launch 파일로 쉽게 실행 가능
  - 파라미터로 CSV 파일/디렉토리 지정 가능

  3. 업데이트된 파일들

  - setup.py: 새로운 노드 추가
  - package.xml: matplotlib, pandas 의존성 추가
  - README.md: Offline Analysis 섹션 추가

  사용 방법

  기본 사용 (최근 녹화 파일 분석)

  # 빌드
  cd ~/f1tenth_dawgs
  colcon build --packages-select path_recorder
  source install/setup.bash

  # global_centerline 토픽을 발행하는 노드 실행 (예: path_planner)
  ros2 launch path_planner csv_path_publisher.launch.py

  # 새 터미널에서 offline 분석 실행
  ros2 launch path_recorder offline_analysis.launch.py

  특정 CSV 파일 분석

  ros2 launch path_recorder offline_analysis.launch.py \
      csv_file:=~/f1tenth_recordings/path_recording_20241030_140530.csv

  플롯만 저장 (헤드리스 시스템)

  ros2 launch path_recorder offline_analysis.launch.py \
      show_plots:=false \
      output_directory:=~/my_analysis

  생성되는 분석 내용

  콘솔 출력

  Statistics Summary:
    Lateral Error:
      Mean:        2.34 cm
      Std Dev:     3.45 cm
      RMS:         4.12 cm
      Abs Mean:    3.01 cm
      Abs Max:    12.56 cm

    Speed:
      Mean:        3.45 m/s
      Max:         6.78 m/s

    Performance:
      Duration:   45.67 s
      Distance:  157.89 m

  저장되는 플롯

  - ~/f1tenth_analysis/analysis_YYYYMMDD_HHMMSS.png
  - 16x12 인치 고해상도 (150 DPI)
  - 8개 서브플롯이 포함된 종합 분석

  주요 기능

  1. 자동 최신 파일 분석: CSV 파일 지정 안 하면 가장 최근 파일 분석
  2. 종합 시각화: 경로, 에러, 속도를 다각도로 분석
  3. 통계 계산: RMS, 평균, 표준편차 등 상세 통계
  4. 유연한 출력: 저장/표시 개별 제어 가능
  5. 자동 종료: 분석 완료 후 자동으로 노드 종료

  빌드하고 실행해보세요!

-- offline ploter
● 완료했습니다! offline 분석 노드를 ROS2 노드로 다시 만들고, map.png를 배경으로 
  global_centerline과 recorded_path를 표시하도록 수정했습니다.

  주요 변경 사항

  1. ROS2 노드 형태로 변경

  - rclpy.Node 상속으로 다시 구현
  - launch 파일로 실행 가능
  - 파라미터로 파일 경로 지정

  2. Map 오버레이 기능 추가

  - map.yaml 파일 읽기: resolution, origin 정보 파싱
  - map.png 이미지 로드: PIL로 이미지 읽기
  - 좌표 변환: 실제 좌표(m) → 맵 픽셀 좌표 변환
  - 오버레이 표시:
    - 맵 이미지를 grayscale 배경으로 표시
    - Global centerline을 녹색 선으로 오버레이
    - Recorded path를 속도 컬러맵으로 오버레이

  3. Launch 파일 업데이트

  - map_yaml 파라미터 추가
  - 기본값으로 mohyun_1030 맵 설정
  - Node 형태로 실행

  4. 좌표 변환 로직

  def world_to_map(self, x, y):
      """실제 좌표 → 맵 픽셀 좌표"""
      resolution = self.map_info['resolution']  # m/pixel
      origin_x, origin_y = self.map_info['origin'][0], self.map_info['origin'][1]

      px = (x - origin_x) / resolution
      py = (y - origin_y) / resolution
      py = self.map_info['height'] - py  # Y축 반전 (이미지는 위에서 아래)

      return px, py

  사용 방법

  기본 실행 (launch 파일 기본값 사용)

  # 빌드
  cd ~/f1tenth_dawgs
  colcon build --packages-select path_recorder
  source install/setup.bash

  # 실행 (기본 경로 사용)
  ros2 launch path_recorder offline_analysis.launch.py

  커스텀 경로 지정

  # 다른 맵 및 경로 사용
  ros2 launch path_recorder offline_analysis.launch.py \
      global_path_csv:=/path/to/your_centerline.csv \
      map_yaml:=/path/to/your_map.yaml

  # 특정 녹화 파일 분석
  ros2 launch path_recorder offline_analysis.launch.py \
      csv_file:=~/f1tenth_recordings/path_recording_20241030_140530.csv

  시각화 결과

  Plot 1-2: Map 오버레이 (새로운 기능!)

  - 배경: map.png 이미지 (grayscale)
  - 녹색 선: Global centerline
  - 컬러 점: Recorded path (속도로 색상 구분)
    - 파란색 = 저속
    - 녹색 = 중속
    - 빨간색 = 고속
  - 확대 뷰: 시작 지점 클로즈업

  Plot 3-8: 기존 통계 플롯

  - Lateral error vs distance
  - Lateral error vs time
  - Lateral error histogram
  - Speed profile
  - Speed vs error correlation
  - Statistics summary

  파일 구조

  mohyun_1030/
  ├── mohyun_slam_v3.yaml      # Map metadata (resolution, origin)
  ├── mohyun_slam_v3.png       # Map image
  └── mohyun_slam_v3_iqp.csv   # Global centerline

  빌드하고 실행하면 맵 위에 경로가 오버레이된 시각화를 볼 수 있습니다!
