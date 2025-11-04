# Path Sector Editor - 사용 가이드

## ROS2 Foxy 빌드 완료

패키지가 성공적으로 빌드되었습니다!

## 실행 방법

### 방법 1: Launch 파일로 실행 (권장)

```bash
cd ~/f1tenth_dawgs
source install/setup.bash
ros2 launch path_sector_editor path_sector_editor.launch.py
```

### 방법 2: 커스텀 CSV 파일로 실행

```bash
ros2 launch path_sector_editor path_sector_editor.launch.py \
    csv_file:=/home/dawgs_nx/f1tenth_dawgs/src/peripheral/maps/mohyun_1103/mohyun_slam_ekf_v2_iqp.csv
```

### 방법 3: 노드 직접 실행

```bash
ros2 run path_sector_editor path_sector_editor_node \
    --ros-args \
    -p csv_file_path:=/home/dawgs_nx/f1tenth_dawgs/src/peripheral/maps/mohyun_1103/mohyun_slam_ekf_v2_iqp.csv
```

## GUI 사용법

### 섹터 정의하기:
1. **"New Sector"** 버튼 클릭
2. 트랙에서 **시작점** 클릭
3. 트랙에서 **끝점** 클릭
4. 텍스트 박스에 **섹터 이름** 입력
5. **Enter** 키를 눌러 섹터 생성

### 섹터 튜닝하기:
1. 맵에서 섹터를 **클릭**하여 선택
2. 슬라이더 조정:
   - **Vel Scale**: 속도 곱하기 (0.1 - 2.0)
   - **Vel Offset**: 속도 더하기 (-2.0 - 2.0 m/s)
   - **D Offset**: 좌우 이동 (-1.0 - 1.0 m, + = 오른쪽)
3. **"Apply Changes"** 버튼 클릭

### 기타 컨트롤:
- **Delete Sector**: 선택한 섹터 삭제
- **Reset All**: 모든 수정 초기화
- **Save CSV**: 수정된 경로를 CSV로 저장
- **Clear Sectors**: 모든 섹터 제거

## 예제 워크플로우

```bash
# 1. 빌드 및 소스
cd ~/f1tenth_dawgs
source install/setup.bash

# 2. 실행
ros2 launch path_sector_editor path_sector_editor.launch.py

# 3. GUI에서:
#    - 섹터 정의 (예: "코너1", "직선구간", "코너2")
#    - 각 섹터 속도 조정
#    - "Apply Changes" 클릭
#    - "Save CSV" 클릭하여 저장

# 4. 저장된 파일은 원본 경로에 *_modified.csv로 저장됨
# 예: mohyun_slam_ekf_v2_iqp_modified.csv
```

## 트러블슈팅

### matplotlib 창이 안 보이는 경우:
```bash
# SSH 사용 시 X11 forwarding 확인
echo $DISPLAY

# matplotlib 백엔드 확인
python3 -c "import matplotlib; print(matplotlib.get_backend())"
```

### "No waypoints loaded" 에러:
```bash
# CSV 파일 경로 확인
ls -la /home/dawgs_nx/f1tenth_dawgs/src/peripheral/maps/mohyun_1103/mohyun_slam_ekf_v2_iqp.csv

# config 파일 수정
nano ~/f1tenth_dawgs/src/utilities/path_sector_editor/config/path_sector_editor.yaml
```

## ROS2 토픽

### Publisher:
- `/global_centerline` (nav_msgs/Path): 수정된 경로 퍼블리시 (1 Hz)

### 확인:
```bash
# 다른 터미널에서 확인
ros2 topic echo /global_centerline
ros2 topic hz /global_centerline
```

## 설정 파일 수정

```bash
# 설정 파일 위치
nano ~/f1tenth_dawgs/src/utilities/path_sector_editor/config/path_sector_editor.yaml

# 수정 후 다시 빌드
colcon build --packages-select path_sector_editor
source install/setup.bash
```
