# Ackermann Mux Brake 기능 테스트 가이드

## 수정 사항 요약

### 수정된 파일
1. **ackermann_mux.hpp**: Brake publisher 및 파라미터 멤버 변수 추가
2. **ackermann_mux.cpp**:
   - `init()`: Brake publisher 초기화 및 파라미터 로드
   - `publishAckermann()`: 감속 감지 및 brake 명령 생성 로직 추가
3. **ackermann_mux_topics.yaml**: Brake 파라미터 설정 추가

### 주요 기능
- 이전 속도와 현재 속도를 비교하여 감속 감지
- 감속량에 비례하는 brake 전류 자동 생성
- `/commands/motor/brake` 토픽으로 VESC에 전달

---

## 빌드 및 설치

```bash
# ackermann_mux 패키지만 빌드
cd /home/dawgs_nx/f1tenth_dawgs
colcon build --packages-select ackermann_mux

# 환경 설정 적용
source install/setup.bash
```

---

## 기본 테스트

### 1. 파라미터 확인

```bash
# ackermann_mux 노드 실행 (별도 터미널)
ros2 run ackermann_mux ackermann_mux --ros-args \
  --params-file src/base_system/f1tenth_system/ackermann_mux/config/ackermann_mux_topics.yaml

# 파라미터 확인 (다른 터미널)
ros2 param list /ackermann_mux

# 개별 파라미터 값 확인
ros2 param get /ackermann_mux deceleration_threshold
ros2 param get /ackermann_mux brake_gain
ros2 param get /ackermann_mux max_brake_current
```

**예상 출력**:
```
Double value is: 0.5
Double value is: 1000.0
Double value is: 20000.0
```

---

### 2. 수동 Brake 테스트

#### Terminal 1: Ackermann Mux 실행
```bash
source install/setup.bash
ros2 run ackermann_mux ackermann_mux --ros-args \
  --params-file src/base_system/f1tenth_system/ackermann_mux/config/ackermann_mux_topics.yaml \
  --log-level debug
```

#### Terminal 2: Drive 토픽 모니터링
```bash
source install/setup.bash
ros2 topic echo /ackermann_cmd
```

#### Terminal 3: Brake 토픽 모니터링
```bash
source install/setup.bash
ros2 topic echo /commands/motor/brake
```

#### Terminal 4: Drive 명령 발행

**Step 1: 가속 (2.0 m/s)**
```bash
source install/setup.bash
ros2 topic pub /drive ackermann_msgs/msg/AckermannDriveStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'},
    drive: {speed: 2.0, steering_angle: 0.0}}" \
  --once
```

**Step 2: 감속 (0.5 m/s) - Brake 적용 예상**
```bash
# 2초 후 실행
ros2 topic pub /drive ackermann_msgs/msg/AckermannDriveStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'},
    drive: {speed: 0.5, steering_angle: 0.0}}" \
  --once
```

**예상 결과**:
- Terminal 1 (디버그 로그):
  ```
  [DEBUG] Deceleration detected: 2.00 m/s -> 0.50 m/s (diff: -1.50), brake: 1500 A
  ```
- Terminal 3 (Brake 토픽):
  ```yaml
  data: 1500.0
  ```

**Step 3: 완전 정지 (0.0 m/s)**
```bash
ros2 topic pub /drive ackermann_msgs/msg/AckermannDriveStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'},
    drive: {speed: 0.0, steering_angle: 0.0}}" \
  --once
```

**예상 결과**:
- Brake 전류: 500 A (0.5 m/s × 1000 A/(m/s))

---

### 3. 실시간 파라미터 튜닝

노드 실행 중에도 파라미터를 변경할 수 있습니다:

```bash
# 더 민감한 brake (낮은 임계값)
ros2 param set /ackermann_mux deceleration_threshold 0.3

# 더 강력한 brake (높은 게인)
ros2 param set /ackermann_mux brake_gain 1500.0

# 최대 brake 제한 변경
ros2 param set /ackermann_mux max_brake_current 15000.0
```

---

### 4. RQt Plot으로 시각화

```bash
# 속도와 brake 전류를 동시에 시각화
rqt_plot /ackermann_cmd/drive/speed /commands/motor/brake/data
```

**보는 법**:
- 빨간 선: 속도 (m/s)
- 파란 선: Brake 전류 (A)
- 속도가 떨어질 때 brake 전류가 증가하는지 확인

---

## 실제 차량 테스트

### 사전 준비
1. 차량 안전 확인 (충분한 공간, 장애물 없음)
2. 비상 정지 준비 (조이스틱의 deadman switch)
3. 파라미터 초기값 확인 (너무 높은 brake_gain은 위험)

### 테스트 절차

#### 1단계: 낮은 속도 테스트

```bash
# Launch ackermann_mux가 포함된 F1TENTH 스택
ros2 launch f1tenth_stack bringup_launch.py

# 낮은 속도로 주행 후 감속 테스트
# 조이스틱이나 컨트롤러로 1-2 m/s까지 가속
# 속도 명령을 낮춰서 감속 테스트
```

**확인 사항**:
- 차량이 부드럽게 감속하는지
- 급격한 제동으로 미끄러지지 않는지
- Brake 토픽에 적절한 값이 발행되는지

#### 2단계: 파라미터 조정

```bash
# 테스트 중 파라미터 조정
ros2 param set /ackermann_mux brake_gain 800.0  # 더 부드럽게
# 또는
ros2 param set /ackermann_mux brake_gain 1200.0  # 더 강력하게
```

#### 3단계: 실제 경로 주행

```bash
# Pure pursuit 컨트롤러와 함께 테스트
ros2 launch pure_pursuit pure_pursuit.launch.py
```

**관찰 사항**:
- 코너 진입 시 감속이 더 빠른지
- 랩타임 개선 여부
- 차량 안정성

---

## 데이터 수집 및 분석

### Bag 파일 기록

```bash
# 테스트 데이터 기록
ros2 bag record -o brake_test \
  /ackermann_cmd \
  /commands/motor/brake \
  /odom \
  /sensors/core \
  /sensors/imu/raw

# 재생 및 분석
ros2 bag play brake_test_0.db3
```

### 분석 스크립트

```python
#!/usr/bin/env python3
import rosbag2_py
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64

# Bag 파일 읽기
# ... (bag 파일 파싱 코드)

# 속도와 brake 전류 플롯
plt.figure(figsize=(12, 6))

plt.subplot(2, 1, 1)
plt.plot(times, speeds)
plt.ylabel('Speed (m/s)')
plt.title('Vehicle Speed')
plt.grid(True)

plt.subplot(2, 1, 2)
plt.plot(times, brake_currents)
plt.ylabel('Brake Current (A)')
plt.xlabel('Time (s)')
plt.title('Brake Current')
plt.grid(True)

plt.tight_layout()
plt.savefig('brake_analysis.png')
```

---

## 문제 해결

### 문제 1: Brake가 작동하지 않음

**증상**: `/commands/motor/brake` 토픽에 데이터가 발행되지 않음

**확인 사항**:
```bash
# 1. 노드가 실행 중인지
ros2 node list | grep ackermann_mux

# 2. 토픽이 존재하는지
ros2 topic list | grep brake

# 3. 파라미터가 로드되었는지
ros2 param list /ackermann_mux
```

**해결책**:
- Config 파일 경로 확인
- Launch 파일에서 파라미터 파일 로드 확인

---

### 문제 2: Brake가 너무 강함

**증상**: 차량이 급격하게 정지하거나 미끄러짐

**해결책**:
```bash
# Brake gain 낮추기
ros2 param set /ackermann_mux brake_gain 500.0

# 또는 임계값 높이기 (덜 민감하게)
ros2 param set /ackermann_mux deceleration_threshold 0.8
```

---

### 문제 3: Brake가 너무 약함

**증상**: 감속이 충분히 빠르지 않음

**해결책**:
```bash
# Brake gain 높이기
ros2 param set /ackermann_mux brake_gain 1500.0

# 또는 임계값 낮추기 (더 민감하게)
ros2 param set /ackermann_mux deceleration_threshold 0.3
```

---

### 문제 4: 가속 시에도 brake가 적용됨

**증상**: 가속이 느리거나 효과가 없음

**확인**:
```bash
# 디버그 로그 확인
ros2 run ackermann_mux ackermann_mux --ros-args --log-level debug
```

- "Acceleration/Maintaining" 로그가 보여야 함
- Brake 값이 0.0이어야 함

---

## 권장 파라미터 설정

### 일반 주행 (부드러운 제동)
```yaml
deceleration_threshold: 0.5
brake_gain: 800.0
max_brake_current: 15000.0
```

### 레이싱 (빠른 제동)
```yaml
deceleration_threshold: 0.3
brake_gain: 1500.0
max_brake_current: 20000.0
```

### 저속 주행 (안전 우선)
```yaml
deceleration_threshold: 0.7
brake_gain: 500.0
max_brake_current: 10000.0
```

---

## 성공 지표

다음 항목들이 개선되었다면 성공입니다:

- ✅ 감속 거리 20-30% 단축
- ✅ 코너 진입 속도 더 높이고도 안정적으로 감속
- ✅ 랩타임 1-2초 개선
- ✅ 부드러운 속도 프로파일 유지
- ✅ 차량 안정성 유지

---

## 추가 개선 아이디어

### 1. 속도 기반 게인 조정
현재는 고정 게인이지만, 속도에 따라 동적으로 조정:

```cpp
// 고속에서는 더 강한 brake
double adaptive_gain = brake_gain_ * (1.0 + 0.2 * current_speed);
```

### 2. 조향각 고려
코너링 중에는 brake를 약하게:

```cpp
double steering_factor = 1.0 - 0.3 * std::abs(msg->drive.steering_angle);
brake_current *= steering_factor;
```

### 3. 가속도 기반 제어
급감속일수록 더 강한 brake:

```cpp
double decel_rate = std::abs(speed_diff) / dt;
double brake_multiplier = 1.0 + std::min(decel_rate, 5.0) * 0.1;
```

---

## 참고 자료

- 원본 가이드: `brake_handling_guide.md`
- VESC 설정: `f1tenth_stack/config/vesc.yaml`
- Ackermann Mux 설정: `config/ackermann_mux_topics.yaml`
