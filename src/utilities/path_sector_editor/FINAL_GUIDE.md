# Path Sector Editor - 최종 완성 가이드

## 🎉 완전 자동화 기능

### ✅ YAML 파일에서 자동 로딩
- **map.yaml 파일 하나만 지정하면 끝!**
- image 경로 자동 읽기
- resolution 자동 읽기
- origin 자동 읽기
- **수동 설정 필요 없음!**

### ✅ 향상된 UI 기능 (최신!)
- **속도 컬러바 토글**: 필요에 따라 켜고 끌 수 있음
- **줌 레벨 유지**: 확대한 상태가 섹터 설정 시에도 계속 유지됨
- **실선 섹터 표시**: 섹터가 경로 뒤에 실선으로 명확하게 표시됨

## 빠른 시작

```bash
cd ~/f1tenth_dawgs
source install/setup.bash
ros2 launch path_sector_editor path_sector_editor.launch.py
```

## 설정 방법 (매우 간단!)

### 방법 1: config 파일 수정 (권장)

`config/path_sector_editor.yaml` 파일:

```yaml
path_sector_editor_node:
  ros__parameters:
    # CSV 경로만 지정
    csv_file_path: "/path/to/your/waypoints.csv"
    
    # YAML 경로만 지정 (나머지는 자동!)
    map_yaml_path: "/path/to/your/map.yaml"
    
    # 부드러운 전환 설정
    smooth_transition_points: 20
    
    # ROS 토픽
    global_path_topic: "/global_centerline"
    frame_id: "map"
    publish_rate: 1.0
```

**그게 끝!** resolution, origin 수동 입력 필요 없음!

### 방법 2: Launch 파일로 덮어쓰기

```bash
ros2 launch path_sector_editor path_sector_editor.launch.py \
    csv_file:=/path/to/waypoints.csv \
    map_yaml:=/path/to/map.yaml
```

## 자동으로 읽어오는 정보

YAML 파일 예시 (`map.yaml`):
```yaml
image: /home/dawgs_nx/f1tenth_dawgs/src/peripheral/maps/mohyun_1103/mohyun_slam_ekf_v2.png
mode: trinary
resolution: 0.05
origin: [-12.4, -8.57, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

**자동으로 읽는 것:**
- ✅ `image`: 맵 이미지 경로
- ✅ `resolution`: 0.05 m/pixel
- ✅ `origin`: [-12.4, -8.57, 0] → x=-12.4, y=-8.57

**상대 경로도 지원!**
- yaml에 `image: map.png`로 되어 있으면
- yaml 파일과 같은 디렉토리에서 자동으로 찾음

## 실행 예시

```bash
# 현재 설정으로 실행 (mohyun_1103 맵)
cd ~/f1tenth_dawgs
source install/setup.bash
ros2 launch path_sector_editor path_sector_editor.launch.py

# 로그 확인 - 자동 로딩 확인
# [INFO] [path_sector_editor_node]: Loaded map config from: /path/to/map.yaml
# [INFO] [path_sector_editor_node]:   Image: /path/to/map.png
# [INFO] [path_sector_editor_node]:   Resolution: 0.05 m/pixel
# [INFO] [path_sector_editor_node]:   Origin: (-12.4, -8.57)
# [INFO] [path_sector_editor_node]: Loaded map image: 481x348 pixels
# [INFO] [path_sector_editor_node]: Map extent: [-12.4, 11.65, -8.57, 8.83]
```

## GUI 사용법

### 1. 섹터 생성 (자동 이름)
- "New Sector" 클릭
- 시작점 클릭
- 끝점 클릭
- ✨ "Sector 1" 자동 생성!
- ✨ 섹터는 **실선**으로 경로 **뒤**에 표시됨

### 2. 섹터 튜닝
- 맵에서 섹터 클릭하여 선택
- 슬라이더 조정:
  - **Vel Scale**: 속도 곱하기
  - **Vel Offset**: 속도 더하기/빼기
  - **D Offset**: 좌우 이동
  - **Smooth Pts**: 부드러운 전환 (0-50)
- "Apply" 클릭

### 3. 뷰 옵션
- **Show Velocity Bar** 체크박스: 속도 컬러바 켜기/끄기
- **줌 유지**: 맵을 확대/축소한 상태가 섹터 설정 시에도 유지됨
- **마우스 휠**: 확대/축소 (matplotlib 기본 기능)
- **패닝**: 마우스 오른쪽 버튼 드래그로 이동 (matplotlib 기본 기능)

### 4. 저장
- "Save CSV" 클릭
- `*_modified.csv`로 저장됨

## 새로운 맵으로 작업하기

### Step 1: 맵 파일 준비
```bash
# 맵 디렉토리 구조
/path/to/maps/
├── mytrack.yaml  # 메타데이터
├── mytrack.pgm   # 또는 .png
└── mytrack.csv   # 경로 데이터
```

### Step 2: Config 수정
```bash
nano ~/f1tenth_dawgs/src/utilities/path_sector_editor/config/path_sector_editor.yaml
```

```yaml
csv_file_path: "/path/to/maps/mytrack.csv"
map_yaml_path: "/path/to/maps/mytrack.yaml"  # 이것만 바꾸면 됨!
```

### Step 3: 빌드 및 실행
```bash
colcon build --packages-select path_sector_editor
source install/setup.bash
ros2 launch path_sector_editor path_sector_editor.launch.py
```

## 전체 기능 요약

### 🎨 시각화
- ✅ **맵 배경**: PNG/PGM 이미지 자동 표시
- ✅ **속도 색상**: Jet colormap (빨강=느림, 파랑=빠름)
- ✅ **컬러바**: 속도 범위 표시 (켜기/끄기 가능)
- ✅ **섹터 표시**: 경로 뒤에 실선으로 표시
- ✅ **섹터 마커**: 원=시작, 사각=끝
- ✅ **줌 유지**: 확대/축소 상태가 섹터 수정 시에도 유지

### 🔧 편집 기능
- ✅ **자동 섹터 이름**: "Sector 1", "Sector 2"...
- ✅ **속도 조정**: Scale & Offset
- ✅ **경로 이동**: Frenet d 좌표
- ✅ **부드러운 전환**: Cosine smoothing

### ⚙️ 자동 설정
- ✅ **YAML 파싱**: image, resolution, origin 자동
- ✅ **상대 경로**: yaml 파일 기준으로 해석
- ✅ **에러 처리**: 파일 없으면 경고 메시지

## 트러블슈팅

### 맵이 안 보이는 경우

**1. YAML 파일 경로 확인**
```bash
ls -la /path/to/your/map.yaml
# 파일이 있는지 확인
```

**2. 로그 확인**
```bash
ros2 launch path_sector_editor path_sector_editor.launch.py
# 출력에서 다음 확인:
# [INFO] Loaded map config from: ...
# [INFO] Loaded map image: ...
```

**3. YAML 파일 내용 확인**
```bash
cat /path/to/your/map.yaml
# image, resolution, origin 필드가 있는지 확인
```

### 맵 위치가 이상한 경우

**YAML의 origin 확인**
```bash
cat /path/to/map.yaml | grep origin
# origin: [-12.4, -8.57, 0]  # [x, y, theta]
```

맵 yaml 파일이 정확하면 **자동으로 맞춰집니다!**

### 이미지 파일을 못 찾는 경우

**절대 경로 사용 권장**
```yaml
# 좋은 예 (절대 경로)
image: /home/user/maps/map.png

# 나쁜 예는 없음 - 상대 경로도 자동 처리!
image: map.png  # yaml과 같은 디렉토리에서 자동으로 찾음
```

## 비교: 이전 vs 현재

### 이전 (수동 설정)
```yaml
csv_file_path: "/path/to/waypoints.csv"
map_image_path: "/path/to/map.png"      # 직접 입력
map_resolution: 0.05                     # 직접 입력
map_origin_x: -12.4                      # 직접 입력
map_origin_y: -8.57                      # 직접 입력
```

### 현재 (자동!)
```yaml
csv_file_path: "/path/to/waypoints.csv"
map_yaml_path: "/path/to/map.yaml"      # 이것만!
```

**훨씬 간단해졌습니다!** 🎉

## ROS2 토픽

```bash
# 수정된 경로 확인
ros2 topic echo /global_centerline

# 퍼블리시 속도 확인
ros2 topic hz /global_centerline
# 출력: average rate: 1.000
```

## 개발 팁

### 여러 맵으로 작업할 때
```bash
# 맵별로 config 파일 만들기
cp config/path_sector_editor.yaml config/mohyun_config.yaml
cp config/path_sector_editor.yaml config/levine_config.yaml

# 각각 수정 후 실행
ros2 run path_sector_editor path_sector_editor_node \
    --ros-args --params-file config/mohyun_config.yaml
```

### Launch 파일에서 맵 선택
```python
# launch/path_sector_editor.launch.py 수정
DeclareLaunchArgument(
    'map_yaml',
    default_value='/path/to/default/map.yaml',
    description='Path to map YAML file'
)
```

```bash
# 실행 시 선택
ros2 launch path_sector_editor path_sector_editor.launch.py \
    map_yaml:=/path/to/other/map.yaml
```

## 요약

1. **YAML 파일 하나만 지정**
2. **나머지는 자동으로 로딩**
3. **섹터 이름도 자동 생성**
4. **부드러운 전환 자동 적용**
5. **속도 색상으로 시각화**
6. **컬러바 켜기/끄기 가능**
7. **줌 레벨 자동 유지**
8. **섹터는 경로 뒤 실선 표시**

**설정이 정말 간단하고 사용이 편리해졌습니다!** 🚀
