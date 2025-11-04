# Path Sector Editor - 완전 가이드

## 새로운 기능 (Updated!)

### ✅ 추가된 기능:
1. **맵 배경 표시**: PNG/PGM 맵 이미지를 배경으로 표시
2. **속도 색상 표시**: 경로의 속도를 색상(jet colormap)으로 시각화
   - 빨간색 = 느린 속도
   - 파란색 = 빠른 속도
3. **자동 섹터 이름**: "Sector 1", "Sector 2"로 자동 생성 (이름 입력 불필요!)
4. **부드러운 경로 전환**: 섹터 경계에서 cosine smoothing 적용
5. **Smooth Points 조절**: 슬라이더로 전환 구간 점 개수 조정 (0-50)

## 빠른 시작

```bash
cd ~/f1tenth_dawgs
source install/setup.bash
ros2 launch path_sector_editor path_sector_editor.launch.py
```

## GUI 사용법

### 섹터 만들기 (간편해졌어요!):
1. **"New Sector"** 버튼 클릭
2. 트랙에서 **시작점** 클릭 (빨간 별 표시)
3. 트랙에서 **끝점** 클릭
4. ✨ **자동으로 "Sector 1" 생성!** (이름 입력 필요 없음)

### 섹터 튜닝:
1. 맵에서 섹터를 **클릭**하여 선택 (점선 테두리로 표시)
2. 슬라이더 조정:
   - **Vel Scale**: 속도 곱하기 (0.1 - 2.0)
   - **Vel Offset**: 속도 더하기/빼기 (-2.0 - 2.0 m/s)
   - **D Offset**: 좌우 이동 (-1.0 - 1.0 m, + = 오른쪽)
   - **Smooth Pts**: 부드러운 전환 점 개수 (0-50)
3. **"Apply"** 버튼 클릭

### 시각화 기능:
- **맵 배경**: 회색 음영으로 트랙 형상 표시
- **속도 색상**: 
  - 경로가 jet colormap으로 표시됨
  - 오른쪽 컬러바로 속도 확인 가능
- **섹터 마커**:
  - 원형 마커 = 시작점
  - 사각형 마커 = 끝점
  - 점선 = 선택된 섹터

### 부드러운 전환 설정:
- **Smooth Pts 슬라이더**: 
  - 0 = 즉시 전환 (급격한 변화)
  - 20 = 기본값 (적당한 부드러움)
  - 50 = 매우 부드러운 전환

예시:
- Smooth Pts = 20이면, 섹터 시작/끝 각 20개 포인트에서 부드럽게 전환
- 짧은 섹터는 자동으로 조정됨 (섹터 길이의 1/4까지만)

## 예제 워크플로우

```bash
# 1. 빌드 및 소스
cd ~/f1tenth_dawgs
source install/setup.bash

# 2. 실행
ros2 launch path_sector_editor path_sector_editor.launch.py

# 3. GUI에서:
#    a) 맵과 속도 색상으로 전체 경로 확인
#    b) "New Sector" → 코너 시작점 클릭 → 끝점 클릭 → "Sector 1" 자동 생성!
#    c) "New Sector" → 직선 시작점 클릭 → 끝점 클릭 → "Sector 2" 자동 생성!
#    d) Sector 1 클릭 → Vel Scale 0.6 (코너에서 감속)
#    e) Smooth Pts 30으로 조정 (부드러운 전환)
#    f) "Apply" 클릭 → 속도 색상 변화 확인
#    g) "Save CSV" 클릭

# 4. 저장된 파일:
# mohyun_slam_ekf_v2_iqp_modified.csv
```

## 설정 파일

`config/path_sector_editor.yaml`:
```yaml
path_sector_editor_node:
  ros__parameters:
    # CSV 파일 경로
    csv_file_path: "/path/to/waypoints.csv"
    
    # 맵 이미지 경로 (PNG or PGM)
    map_image_path: "/path/to/map.png"
    
    # 맵 메타데이터 (맵 yaml 파일에서 확인)
    map_resolution: 0.05     # meters per pixel
    map_origin_x: -12.4      # map origin x (meters)
    map_origin_y: -8.57      # map origin y (meters)
    
    # 부드러운 전환 설정
    smooth_transition_points: 20  # 전후 각 20개 포인트
    
    # ROS 토픽
    global_path_topic: "/global_centerline"
    frame_id: "map"
    publish_rate: 1.0
```

### 맵 설정 찾는 방법:
```bash
# 맵 yaml 파일 확인
cat /path/to/your/map.yaml

# 출력 예시:
# image: /path/to/map.png
# resolution: 0.05
# origin: [-12.4, -8.57, 0]  # [x, y, theta]
```

## 새 기능 상세 설명

### 1. 맵 배경 표시
- PNG 또는 PGM 이미지 자동 로딩
- 회색 음영(grayscale)으로 표시, alpha=0.6
- 맵 origin과 resolution에 맞춰 world 좌표로 변환

### 2. 속도 색상 표시 (Jet Colormap)
```python
# 색상 의미:
Red (빨강)    = 느린 속도 (v_min)
Yellow (노랑) = 중간 속도
Green (초록)  = 중간 높은 속도
Blue (파랑)   = 빠른 속도 (v_max)
```

### 3. 자동 섹터 이름
- 첫 번째 섹터: "Sector 1"
- 두 번째 섹터: "Sector 2"
- ...계속 증가
- "Clear All" 누르면 카운터 리셋

### 4. 부드러운 전환 (Cosine Smoothing)
```
섹터 시작 부분: weight = 0.5 * (1 - cos(π * t))
  → 0에서 1로 부드럽게 증가

섹터 끝 부분: weight = 0.5 * (1 - cos(π * t))
  → 1에서 0으로 부드럽게 감소

t = 현재 위치 / smooth_transition_points
```

예시:
- Smooth Pts = 20
- Sector가 100 포인트일 때:
  - 포인트 0-19: 0% → 100% (부드럽게 증가)
  - 포인트 20-79: 100% (완전 적용)
  - 포인트 80-99: 100% → 0% (부드럽게 감소)

## ROS2 토픽

### Publisher:
- `/global_centerline` (nav_msgs/Path): 수정된 경로

### 확인:
```bash
# 다른 터미널에서
ros2 topic echo /global_centerline --once
ros2 topic hz /global_centerline
```

## 트러블슈팅

### 맵이 안 보이는 경우:
```bash
# 맵 파일 경로 확인
ls -la /path/to/your/map.png

# 로그 확인 (맵 로딩 상태)
ros2 launch path_sector_editor path_sector_editor.launch.py
# 출력에서 "Loaded map image" 메시지 확인
```

### 맵이 잘못된 위치에 표시되는 경우:
```bash
# 맵 yaml 파일에서 origin 확인
cat /path/to/map.yaml

# config 파일에 정확히 입력
nano ~/f1tenth_dawgs/src/utilities/path_sector_editor/config/path_sector_editor.yaml
# map_origin_x, map_origin_y 수정

# 다시 빌드
colcon build --packages-select path_sector_editor
source install/setup.bash
```

### 속도 색상이 이상한 경우:
- 경로 데이터의 속도가 모두 같으면 단색으로 표시됨
- CSV 파일의 v 컬럼 확인

## 팁

1. **효율적인 섹터 정의**:
   - 코너, 직선, 추월 구간별로 섹터 나누기
   - 너무 많은 섹터보다는 큰 구간으로

2. **부드러운 전환 활용**:
   - 급격한 속도 변화가 필요한 곳: Smooth Pts 5-10
   - 일반적인 경우: Smooth Pts 20 (기본값)
   - 매우 부드러운 전환: Smooth Pts 30-50

3. **시각화 활용**:
   - 수정 전후 속도 색상 비교
   - 맵 배경으로 경로가 벽에 닿는지 확인

4. **CSV 저장**:
   - 원본 파일은 그대로 유지됨
   - 새 파일은 `*_modified.csv`로 저장됨
