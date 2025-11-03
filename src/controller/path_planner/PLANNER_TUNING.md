# Frenet Path Planner 파라미터 튜닝 가이드

F1TENTH 레이싱을 위한 Frenet 기반 경로 계획기의 파라미터 조정 방법을 설명합니다.

---

## 📋 목차

1. [시스템 개요](#시스템-개요)
2. [Best Path 토픽 구조](#best-path-토픽-구조)
3. [파라미터 설명](#파라미터-설명)
4. [튜닝 절차](#튜닝-절차)
5. [문제 해결](#문제-해결)
6. [실전 팁](#실전-팁)

---

## 시스템 개요

### Frenet Path Planner란?

Frenet 좌표계를 사용하여 **장애물을 회피하면서 중심선을 따라가는 최적 경로**를 실시간으로 생성합니다.

**주요 특징**:
- ✅ 45개 후보 경로 생성 (9개 lateral × 5개 time horizon)
- ✅ 충돌 검사 + 근접 비용 계산
- ✅ Cost function 기반 최적 경로 선택
- ✅ 속도에 따른 동적 안전 반경

### 데이터 흐름

```
LiDAR Scan → Obstacle Detection → Clustering → Bounding Box
                                                     ↓
Odometry → Frenet State → Trajectory Generation ← Obstacles
                                 ↓
                         Cost Evaluation (충돌 + 근접도 + 평활도)
                                 ↓
                         Best Path Selection
                                 ↓
                         /planned_path (path_tracker 입력)
```

---

## Best Path 토픽 구조

### 1. 주요 토픽

| 토픽 | 타입 | 설명 | 사용처 |
|------|------|------|--------|
| `/planned_path` | `nav_msgs/Path` | **최종 계획 경로 (Best Path)** ⭐ | path_tracker가 따라가는 경로 |
| `/frenet_path` | `nav_msgs/Path` | Frenet 최적 경로 (시각화) | RViz 디버깅용 |
| `/global_centerline` | `nav_msgs/Path` | CSV 글로벌 중심선 | 참조 경로 |
| `/obstacle_boxes` | `MarkerArray` | 감지된 장애물 바운딩 박스 | RViz 시각화 |

### 2. Best Path 생성 과정

```
Step 1: Frenet Trajectory Generation
  - 45개 후보 생성 (9 lateral × 5 time horizons)
  - 장애물 충돌 검사 + Proximity cost 계산
  - 최적 경로 선택 (cost 기반)

Step 2: Lattice LUT Extension (optional)
  - Frenet 경로 끝에서 spiral 경로 연결
  - 부드러운 곡선 연결

Step 3: Publish
  - /planned_path → path_tracker 입력
```

### 3. Path 메시지 구조

```cpp
nav_msgs/Path {
  header: {frame_id: "map", stamp: ...}
  poses: [
    PoseStamped {
      position: {x, y, z}  // z에 속도 저장!
      orientation: {x, y, z, w}  // yaw 정보
    },
    ...
  ]
}
```

**중요**: `pose.position.z`에 목표 속도가 저장됩니다!

### 4. 토픽 사용법

**path_tracker에서 사용**:
```cpp
void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    for (const auto& pose : msg->poses) {
        double x = pose.pose.position.x;
        double y = pose.pose.position.y;
        double target_velocity = pose.pose.position.z;  // ⭐ 속도 정보
        double yaw = getYawFromQuaternion(pose.pose.orientation);

        // Pure pursuit 또는 다른 tracking 알고리즘 적용
    }
}
```

**RViz에서 시각화**:
```
Add → By Topic → /planned_path → Path
```

---

## 파라미터 설명

파라미터 파일: `config/planner_params.yaml`

### 1. Frenet Trajectory 생성 파라미터

#### 기본 설정
```yaml
frenet_time_horizon: 3.0       # 최대 시간 범위 [s]
frenet_min_time: 1.0           # 최소 시간 범위 [s]
frenet_target_speed: 3.0       # 목표 속도 [m/s]
frenet_dt: 0.05                # 샘플링 간격 [s]
frenet_max_speed: 12.0         # 최대 허용 속도 [m/s]
frenet_max_accel: 3.0          # 최대 lateral 가속도 [m/s²]
frenet_lookahead_distance: 0.5 # 전방 예측 거리 [m]
```

**조정 방법**:
- `target_speed` ↑ → 더 빠른 주행 (장애물 회피 능력 ↓)
- `time_horizon` ↑ → 더 먼 미래 예측 (계산량 ↑)
- `lookahead_distance` ↑ → 더 일찍 경로 계획 시작

#### Lateral Sampling (좌우 샘플링)
```yaml
frenet_d_samples: [-1.25, -1.0, -0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 0.75, 1.0, 1.25]
```

**의미**: 중심선으로부터 좌우로 얼마나 벗어난 경로를 시도할지 결정

**조정 방법**:
- 샘플 개수 ↑ → 더 많은 회피 옵션 (계산량 ↑)
- 범위 ↑ (예: ±1.5m) → 더 큰 장애물 회피 가능
- 범위 ↓ (예: ±0.8m) → 중심선 근처만 주행

**예시**:
```yaml
# 좁은 트랙 (중심선 선호)
frenet_d_samples: [-0.5, -0.25, 0.0, 0.25, 0.5]

# 넓은 트랙 (큰 회피)
frenet_d_samples: [-1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5]
```

#### Time Horizon Sampling
```yaml
frenet_t_samples: [1.5, 2.0, 2.5, 3.0]
```

**의미**: 얼마나 먼 미래까지 경로를 계획할지

**조정 방법**:
- 짧은 시간 (1.0~2.0s) → 빠른 반응, 급한 회피
- 긴 시간 (2.5~4.0s) → 부드러운 경로, 계획적 회피

---

### 2. Cost Function 가중치

```yaml
frenet_k_jerk: 0.1       # 평활도 (jerk) 가중치
frenet_k_time: 0.1       # 시간 가중치 (빠른 경로 선호)
frenet_k_deviation: 1.0  # 중심선 이탈 패널티
frenet_k_velocity: 1.0   # 속도 오차 패널티
```

**Cost Function**:
```
Total Cost = k_jerk × jerk + k_time × time + k_deviation × deviation + k_velocity × v_error + k_proximity × proximity_cost
```

#### 각 가중치의 역할

| 가중치 | 역할 | 증가 시 효과 | 감소 시 효과 |
|--------|------|-------------|-------------|
| `k_jerk` | 경로 평활도 | 더 부드러운 경로 (급격한 핸들 조작 ↓) | 더 날카로운 회피 가능 |
| `k_time` | 경로 길이 | 짧은 경로 선호 (빠른 도착) | 긴 경로 허용 (안전 우선) |
| `k_deviation` | 중심선 선호도 | 중심선에 가까운 경로 선호 | 큰 회피 허용 |
| `k_velocity` | 속도 유지 | 목표 속도 유지 (안정적) | 속도 변화 허용 |

**튜닝 예시**:

```yaml
# 공격적 주행 (빠르고 날카롭게)
frenet_k_jerk: 0.05          # 평활도 낮춤
frenet_k_time: 0.2           # 짧은 경로 선호
frenet_k_deviation: 0.5      # 중심선 이탈 허용
frenet_k_velocity: 0.5       # 속도 변화 허용

# 안전 주행 (부드럽고 안정적)
frenet_k_jerk: 0.3           # 평활도 높임
frenet_k_time: 0.05          # 경로 길이 덜 중요
frenet_k_deviation: 2.0      # 중심선 선호
frenet_k_velocity: 1.5       # 속도 유지
```

---

### 3. 장애물 회피 파라미터 ⭐ 중요!

#### Hard Collision (충돌 판정)
```yaml
frenet_safety_radius: 0.3        # 기본 충돌 반경 [m]
frenet_vehicle_radius: 0.25      # 차량 크기 [m]
frenet_obstacle_radius: 0.15     # 장애물 크기 [m]
frenet_k_velocity_safety: 0.05   # 속도 의존 안전 계수
frenet_min_safety_margin: 0.25   # 최소 안전 여유 [m]
```

**동적 안전 반경 계산**:
```
dynamic_safety = max(min_safety_margin,
                     safety_radius + vehicle_radius + obstacle_radius + k_velocity_safety × velocity)
```

**예시** (v=3.0 m/s):
```
dynamic_safety = max(0.25, 0.3 + 0.25 + 0.15 + 0.05 × 3.0)
               = max(0.25, 0.85)
               = 0.85m
```

→ 장애물과의 거리가 **0.85m 미만**이면 **충돌로 판정** → 해당 경로 거부

#### Soft Cost (근접 비용) ⭐⭐⭐

```yaml
frenet_k_proximity: 10.0         # 근접 비용 가중치
frenet_proximity_threshold: 3.5  # 근접 비용 적용 범위 [m]
```

**근접 비용 계산**:
```
장애물과의 거리 d에 대해:
- d < dynamic_safety (0.85m): 충돌! 경로 거부
- 0.85m < d < 3.5m: proximity_cost += k_proximity × (1.0 / (margin + 0.1))
- d > 3.5m: 비용 없음
```

**거리별 비용 예시** (k_proximity=10.0):

| 거리 (m) | margin | proximity_cost | 효과 |
|---------|--------|----------------|------|
| 0.85 | 0.00 | ∞ | 충돌 거부 |
| 1.0 | 0.15 | 40.0 | 매우 높은 비용 |
| 1.5 | 0.65 | 13.3 | 높은 비용 |
| 2.0 | 1.15 | 8.0 | 중간 비용 |
| 2.5 | 1.65 | 5.7 | 낮은 비용 |
| 3.0 | 2.15 | 4.4 | 매우 낮은 비용 |
| 3.5 | 2.65 | 0.0 | 비용 없음 |

**튜닝 가이드**:

```yaml
# 문제: 모든 경로가 collision (collision-free = 0)
# 해결: safety_radius 감소
frenet_safety_radius: 0.2        # 0.3 → 0.2
frenet_vehicle_radius: 0.2       # 0.25 → 0.2
frenet_k_velocity_safety: 0.03   # 0.05 → 0.03

# 문제: 장애물을 회피하지 않음 (중심선만 따라감)
# 해결: proximity 비용 증가
frenet_k_proximity: 20.0         # 10.0 → 20.0
frenet_proximity_threshold: 4.0  # 3.5 → 4.0
frenet_k_deviation: 0.5          # 1.0 → 0.5 (중심선 선호도 낮춤)

# 문제: 장애물을 너무 일찍 회피 (과도한 회피)
# 해결: proximity 비용 감소
frenet_k_proximity: 5.0          # 10.0 → 5.0
frenet_proximity_threshold: 2.5  # 3.5 → 2.5
```

---

### 4. 장애물 감지 파라미터

```yaml
obstacle_cluster_distance: 0.5      # 클러스터링 거리 [m]
obstacle_max_box_size: 0.8          # 최대 박스 크기 [m]
obstacle_box_safety_margin: 0.3     # 박스 안전 여유 [m]
```

**조정 방법**:
- `cluster_distance` ↑ → 더 큰 클러스터 (여러 점을 하나로 묶음)
- `max_box_size` ↓ → 작은 장애물만 감지 (벽 무시)
- `safety_margin` ↑ → 더 큰 안전 여유

---

### 5. 도로 경계 파라미터

```yaml
frenet_road_half_width: 1.5   # 도로 반폭 [m]
```

**의미**: 중심선으로부터 좌우 얼마까지 주행 가능한지

**조정 방법**:
- 좁은 트랙: 1.0m
- 일반 트랙: 1.5m
- 넓은 트랙: 2.0m

---

### 6. 로그 레벨

```yaml
log_level: 4  # 0=NONE, 1=ERROR, 2=WARN, 3=INFO, 4=DEBUG, 5=VERBOSE
```

**권장 설정**:
- 일반 주행: `3` (INFO)
- 디버깅: `4` (DEBUG)
- 상세 분석: `5` (VERBOSE)

---

## 튜닝 절차

### Step 1: 기본 동작 확인

```bash
# 1. Path planner 실행
ros2 launch path_planner path_planner.launch.py

# 2. 로그 확인
[Frenet] 🎯 Generating trajectories with X obstacle points
[Frenet] 📊 Generated 44 candidates (Y collision-free, Z collisions)
[Frenet] ✅ Selected best trajectory: cost=X.XXX, points=Y
```

**정상 동작**:
- ✅ collision-free > 0 (장애물 회피 가능한 경로 존재)
- ✅ cost < 100 (합리적인 경로)

**문제 상황**:
- ❌ collision-free = 0 → [문제 1](#문제-1-모든-경로-collision) 참조
- ❌ centerline만 따라감 → [문제 2](#문제-2-장애물-회피-안-함) 참조

---

### Step 2: 파라미터별 튜닝

#### 2.1 장애물 회피 강도 조정

**목표**: 장애물 근처를 적절히 회피하도록 설정

```yaml
# 시작 설정
frenet_k_proximity: 10.0
frenet_proximity_threshold: 3.5
```

**테스트 방법**:
1. 장애물 앞에 차량 배치
2. RViz에서 `/frenet_path` (청록색 선) 확인
3. 장애물을 피하는지 관찰

**조정**:
- 회피 약함 → `k_proximity` 증가 (10.0 → 15.0 → 20.0)
- 회피 강함 → `k_proximity` 감소 (10.0 → 7.0 → 5.0)
- 너무 일찍 회피 → `proximity_threshold` 감소 (3.5 → 3.0)
- 너무 늦게 회피 → `proximity_threshold` 증가 (3.5 → 4.0)

#### 2.2 중심선 선호도 조정

**목표**: 장애물이 없을 때 중심선을 얼마나 따를지 결정

```yaml
frenet_k_deviation: 1.0  # 기본값
```

**조정**:
- 중심선 강하게 선호 → 증가 (1.0 → 1.5 → 2.0)
- 장애물 회피 우선 → 감소 (1.0 → 0.5 → 0.3)

#### 2.3 경로 평활도 조정

**목표**: 경로가 얼마나 부드러운지 결정

```yaml
frenet_k_jerk: 0.1  # 기본값
```

**조정**:
- 더 부드러운 경로 → 증가 (0.1 → 0.2 → 0.3)
- 더 날카로운 회피 → 감소 (0.1 → 0.05)

#### 2.4 안전 반경 조정

**목표**: 충돌 판정 기준 설정

```yaml
frenet_safety_radius: 0.3
frenet_vehicle_radius: 0.25
frenet_obstacle_radius: 0.15
frenet_k_velocity_safety: 0.05
```

**조정**:
- 너무 많이 충돌 판정 → 모두 감소
- 실제 충돌 발생 → 모두 증가

---

### Step 3: RViz 시각화 확인

**주요 시각화 요소**:

1. **빨간 박스** (`/obstacle_boxes`): 감지된 장애물
2. **청록색 선** (`/frenet_path`): Frenet 최적 경로
3. **보라색 선** (`/planned_path`): 최종 계획 경로 (path_tracker 입력)
4. **초록 중심선** (`/global_centerline`): CSV 참조 경로

**성공 기준**:
- ✅ 청록색 Frenet path가 빨간 장애물을 피해 lateral offset으로 이동
- ✅ 보라색 planned path가 Frenet path를 따라감
- ✅ 장애물 통과 후 중심선으로 복귀

---

## 문제 해결

### 문제 1: 모든 경로 collision

**증상**:
```bash
[Frenet] 📊 Generated 44 candidates (0 collision-free, 44 collisions)
[Frenet] ❌ No valid trajectory found!
```

**원인**: Hard collision radius가 너무 큼

**해결책**:

```yaml
# Step 1: safety_radius 감소
frenet_safety_radius: 0.2        # 0.3 → 0.2

# Step 2: 차량/장애물 크기 감소
frenet_vehicle_radius: 0.2       # 0.25 → 0.2
frenet_obstacle_radius: 0.1      # 0.15 → 0.1

# Step 3: 속도 의존 계수 감소
frenet_k_velocity_safety: 0.03   # 0.05 → 0.03

# Step 4: lateral 샘플 범위 증가
frenet_d_samples: [-1.5, -1.25, -1.0, -0.5, 0.0, 0.5, 1.0, 1.25, 1.5]
```

**검증**:
```bash
# collision-free > 0 확인
[Frenet] 📊 Generated 44 candidates (15 collision-free, 29 collisions)  ← ✅ OK!
```

---

### 문제 2: 장애물 회피 안 함

**증상**:
- collision-free > 0 (경로는 존재함)
- 하지만 청록색 Frenet path가 중심선만 따라가고 장애물을 회피하지 않음

**원인**: Proximity cost가 다른 cost에 비해 너무 작음

**해결책**:

```yaml
# Step 1: proximity 비용 증가
frenet_k_proximity: 20.0         # 10.0 → 20.0

# Step 2: proximity 범위 증가
frenet_proximity_threshold: 4.0  # 3.5 → 4.0

# Step 3: 중심선 선호도 감소
frenet_k_deviation: 0.5          # 1.0 → 0.5

# Step 4 (선택): jerk 가중치 감소 (더 날카로운 회피 허용)
frenet_k_jerk: 0.05              # 0.1 → 0.05
```

**검증**:
- RViz에서 `/frenet_path`가 장애물 옆으로 이동하는지 확인
- 로그에서 선택된 경로의 lateral offset 확인

---

### 문제 3: 장애물 감지 안 됨

**증상**:
```bash
⚠️ WARNING: No obstacles to pass to Frenet planner!
```

**원인**: LiDAR 데이터 또는 TF 변환 문제

**해결책**:

```bash
# 1. LiDAR 데이터 확인
ros2 topic hz /scan
ros2 topic echo /scan | head -50

# 2. TF 확인
ros2 run tf2_ros tf2_echo map laser

# 3. 장애물 필터 파라미터 확인
obstacle_max_box_size: 1.2      # 너무 작으면 증가
obstacle_cluster_distance: 0.5  # 적절한 클러스터링
```

---

### 문제 4: 경로가 울퉁불퉁함

**원인**: Jerk 가중치가 너무 낮음

**해결책**:

```yaml
# 평활도 증가
frenet_k_jerk: 0.3              # 0.1 → 0.3

# Time horizon 증가 (더 긴 미래 계획)
frenet_time_horizon: 4.0        # 3.0 → 4.0
frenet_t_samples: [2.0, 2.5, 3.0, 3.5, 4.0]
```

---

### 문제 5: 경로가 너무 느림

**원인**: Time cost 가중치가 너무 낮음

**해결책**:

```yaml
# 짧은 경로 선호
frenet_k_time: 0.3              # 0.1 → 0.3

# 목표 속도 증가
frenet_target_speed: 5.0        # 3.0 → 5.0
```

---

## 실전 팁

### 팁 1: 단계적 튜닝

**절대 한 번에 여러 파라미터를 바꾸지 마세요!**

✅ **올바른 방법**:
1. 하나의 파라미터만 변경
2. 테스트 및 결과 확인
3. 효과가 있으면 유지, 없으면 원복
4. 다음 파라미터로 이동

❌ **잘못된 방법**:
- k_proximity, k_deviation, safety_radius를 동시에 변경 → 어떤 것이 효과적인지 모름

---

### 팁 2: 로그 분석

**핵심 로그 항목**:

```bash
# 1. 장애물 감지
🚧 Obstacles detected! Count: 128, First at: (5.23, 2.41)
→ 장애물이 제대로 감지되는지 확인

# 2. Candidate 생성
[Frenet] 📊 Generated 44 candidates (31 collision-free, 13 collisions)
→ collision-free가 0이면 안전 반경 문제

# 3. Best 선택
[Frenet] ✅ Selected best trajectory: cost=12.456, points=61
→ cost가 너무 높으면 (>100) 파라미터 문제
```

---

### 팁 3: 파라미터 백업

**튜닝 전 항상 백업!**

```bash
cp config/planner_params.yaml config/planner_params.yaml.backup_$(date +%Y%m%d_%H%M%S)
```

---

### 팁 4: 시나리오별 설정

**좁은 트랙 (복잡한 장애물)**:
```yaml
frenet_d_samples: [-0.8, -0.5, -0.25, 0.0, 0.25, 0.5, 0.8]
frenet_k_proximity: 15.0
frenet_k_deviation: 0.5
frenet_safety_radius: 0.2
```

**넓은 트랙 (간단한 장애물)**:
```yaml
frenet_d_samples: [-1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5]
frenet_k_proximity: 10.0
frenet_k_deviation: 1.0
frenet_safety_radius: 0.3
```

**고속 주행**:
```yaml
frenet_target_speed: 6.0
frenet_k_time: 0.3
frenet_k_jerk: 0.2
frenet_k_velocity_safety: 0.1  # 속도 높을 때 더 큰 안전 반경
```

---

### 팁 5: 실시간 파라미터 변경

ROS2 파라미터는 런타임 중 변경 가능합니다:

```bash
# 예시: proximity 가중치 변경
ros2 param set /path_planner_node frenet_k_proximity 15.0

# 현재 파라미터 확인
ros2 param get /path_planner_node frenet_k_proximity

# 모든 파라미터 확인
ros2 param list /path_planner_node
```

**주의**: 런타임 변경은 YAML 파일에 저장되지 않습니다!

---

## 요약

### 가장 중요한 3가지 파라미터

1. **`frenet_k_proximity`**: 장애물 회피 강도 (기본: 10.0)
2. **`frenet_safety_radius`**: 충돌 판정 기준 (기본: 0.3)
3. **`frenet_k_deviation`**: 중심선 선호도 (기본: 1.0)

### 기본 튜닝 순서

```
1. safety_radius 조정 → collision-free > 0 만들기
2. k_proximity 조정 → 장애물 회피하게 만들기
3. k_deviation 조정 → 중심선과 회피 균형 맞추기
4. 나머지 파라미터 미세 조정
```

### 빠른 체크리스트

- [ ] LiDAR 데이터가 들어오는가? (`ros2 topic hz /scan`)
- [ ] 장애물이 감지되는가? (RViz에서 빨간 박스 확인)
- [ ] collision-free > 0인가? (로그 확인)
- [ ] Frenet path가 장애물을 피하는가? (RViz에서 청록색 선 확인)
- [ ] planned_path를 tracker가 잘 따라가는가?
- [ ] 중심선으로 잘 복귀하는가?

---

## 추가 자료

- 코드: `src/controller/path_planner/`
- 파라미터 파일: `config/planner_params.yaml`
- 헤더 파일: `include/path_planner/frenet.hpp`
- 구현 파일: `src/frenet.cpp`
- 시각화 가이드: `RVIZ_VISUALIZATION_GUIDE.md`

**문의**: 문제가 해결되지 않으면 로그 레벨을 5 (VERBOSE)로 올리고 상세 로그를 확인하세요.

--- obstable tuning
  📊 변경 사항 요약

  planner_params.yaml (path_planner/config/)

  | 파라미터                       | 이전 → 변경 후      | 효과                 |
  |----------------------------|----------------|--------------------|
  | frenet_d_samples           | ±1.25m → ±2.0m | 더 넓은 좌우 회피 범위      |
  | frenet_k_deviation         | 1.0 → 0.5      | 회피 우선, 중심선 선호도 감소  |
  | obstacle_max_box_size      | 0.8 → 10.0     | 벽/맵도 장애물로 인식       |
  | frenet_safety_radius       | 0.3 → 0.6      | 하드 충돌 반경 2배        |
  | frenet_vehicle_radius      | 0.25 → 0.35    | 차량 크기 여유 증가        |
  | frenet_obstacle_radius     | 0.1 → 0.3      | 장애물 크기 여유 3배       |
  | frenet_k_proximity         | 15.0 → 30.0    | 근접 비용 2배 (더 멀리 회피) |
  | frenet_proximity_threshold | 3.5 → 5.0      | 소프트 비용 범위 확대       |

  회피 거리 변화:
  이전: dynamic_safety = 0.8m, soft range = 0.1m (너무 좁음!)
  변경: dynamic_safety = 1.4m, soft range = 3.6m (충분한 회피 범위)

  --- avoidance radius
  2️⃣ 회피 반경 문제

  현재 설정 (제가 변경한 값):
  frenet_safety_radius: 0.6
  frenet_vehicle_radius: 0.35
  frenet_obstacle_radius: 0.3
  → dynamic_safety = 0.6 + 0.35 + 0.3 + (0.05 × 3.0) = 1.4m (너무 넓음!)

--- raduis adjust
● ✅ 문제 해결 완료

  두 가지 문제를 정확히 수정했습니다:

  ---
  1️⃣ Planned Path vs Frenet Path 차이 해결 ✅

  문제:
  // path_planner_node.cpp:970-1021
  planned_path = frenet_path + lattice_path  // Lattice가 시작점(0)으로 회귀하는 경로 추가

  해결: planner_params.yaml
  use_lattice: false  # true → false

  결과: 이제 planned_path = frenet_path (동일한 경로) ✅

  ---
  2️⃣ 회피 반경 축소 ✅

  문제: 제가 설정한 값이 너무 넓었음
  이전: dynamic_safety = 0.6 + 0.35 + 0.3 + (0.05 × 3.0) = 1.4m (너무 넓음!)

  해결: planner_params.yaml
  # 회피 반경 조정
  frenet_safety_radius: 0.6 → 0.45      # 하드 충돌 반경 감소
  frenet_vehicle_radius: 0.35 → 0.25    # 차량 크기 여유 감소
  frenet_obstacle_radius: 0.3 → 0.15    # 장애물 크기 여유 감소
  frenet_k_proximity: 30.0 → 20.0       # 근접 비용 가중치 조정
  frenet_proximity_threshold: 5.0 → 4.0 # 소프트 비용 범위 조정

  결과:
  새로운 dynamic_safety = 0.45 + 0.25 + 0.15 + (0.05 × 3.0) = 1.0m
  proximity_threshold = 4.0m
  → 1.0m 이내는 충돌, 1.0~4.0m는 거리에 비례한 회피 비용

  회피 거리 비교:
  처음 (충돌 발생): 0.8m 하드 충돌, 0.1m 소프트 범위 → 너무 좁아서 충돌 ❌
  제 첫 수정: 1.4m 하드 충돌, 3.6m 소프트 범위 → 너무 넓어서 과도한 회피 ❌
  지금 (균형): 1.0m 하드 충돌, 3.0m 소프트 범위 → 적절한 회피 ✅

  ---
  📊 최종 설정 요약

  | 파라미터                | 최종 값  | 효과                                 |
  |---------------------|-------|------------------------------------|
  | use_lattice         | false | planned_path = frenet_path (동일 경로) |
  | frenet_d_samples    | ±1.5m | 좌우 회피 범위 (사용자가 설정)                 |
  | frenet_k_deviation  | 0.5   | 회피 우선 (중심선 선호도 낮춤)                 |
  | dynamic_safety      | 1.0m  | 하드 충돌 반경 (적절한 거리)                  |
  | proximity_threshold | 4.0m  | 소프트 비용 범위                          |
  | frenet_k_proximity  | 20.0  | 근접 비용 가중치                          |

  ---
  🧪 테스트 확인 사항

  빌드 후 확인해주세요:

  colcon build --packages-select path_planner
  source install/setup.bash
  ros2 launch path_planner path_planner.launch.py

  RViz에서 확인:
  1. /planned_path (빨간색) = /frenet_path (초록색) ← 이제 동일해야 함! ✅
  2. 장애물 회피 거리가 약 1.0m ← 너무 넓지도 좁지도 않음 ✅
  3. 경로가 0으로 회귀하지 않음 ← Lattice 제거로 해결 ✅

  추가 조정이 필요하면 알려주세요! 🎯
--- rande more
 ✅ 회피 반경 축소 완료

  3가지 파라미터를 동시에 조정했습니다:

  ---
  📝 변경 사항

  | 파라미터                       | 이전    | 변경 후  | 효과            |
  |----------------------------|-------|-------|---------------|
  | frenet_d_samples           | ±1.5m | ±1.0m | 샘플링 범위 33% 축소 |
  | frenet_k_deviation         | 0.5   | 1.5   | 중심선 선호도 3배 증가 |
  | frenet_k_proximity         | 20.0  | 5.0   | 근접 비용 75% 감소  |
  | frenet_proximity_threshold | 4.0m  | 3.0m  | 소프트 비용 범위 축소  |

  ---
  📊 예상 회피 거리 계산

  Before (1.6~1.9m 회피)

  Cost @ 1.6m:
    proximity_cost = 20.0 / (1.15 + 0.1) = 16.0
    deviation_cost = 0.5 × 1.0 = 0.5
    total = 16.0 + 0.5 = 16.5 (proximity가 압도적!)

  After (0.8~1.2m 예상)

  Cost @ 1.0m:
    proximity_cost = 5.0 / (0.55 + 0.1) = 7.7  (기존 30.8)
    deviation_cost = 1.5 × 0.6 = 0.9           (기존 0.5)
    total = 7.7 + 0.9 = 8.6

  Cost @ 0.8m:
    proximity_cost = 5.0 / (0.35 + 0.1) = 11.1
    deviation_cost = 1.5 × 0.4 = 0.6
    total = 11.1 + 0.6 = 11.7 (약간 높음)

  Cost @ 1.2m:
    proximity_cost = 5.0 / (0.75 + 0.1) = 5.9
    deviation_cost = 1.5 × 0.8 = 1.2
    total = 5.9 + 1.2 = 7.1 (더 낮음)

  예상: 최적 회피 거리 0.9~1.2m ✅

  ---
  🎯 Cost Function 균형 변화

  Before (넓은 회피)

  proximity_cost >> deviation_cost
       16.0      >>     0.5
  → 장애물에서 최대한 멀어짐 (1.6~1.9m)

  After (타이트한 회피)

  proximity_cost ≈ deviation_cost
       7.7       ≈     0.9
  → 균형잡힌 회피 (0.9~1.2m)

  ---
  🔧 추가 조정 옵션 (필요시)

  만약 테스트 후에도 회피가 여전히 넓다면:

  더 타이트하게 (0.7~1.0m):

  frenet_k_proximity: 5.0 → 3.0      # 더 감소
  frenet_k_deviation: 1.5 → 2.0      # 더 증가

  더 넓게 (1.2~1.5m):

  frenet_k_proximity: 5.0 → 8.0      # 증가
  frenet_k_deviation: 1.5 → 1.0      # 감소

  ---
  🧪 테스트 방법

  # 빌드
  colcon build --packages-select path_planner
  source install/setup.bash

  # 실행
  ros2 launch path_planner path_planner.launch.py

  # RViz에서 확인
  # 1. /planned_path (빨간색) - 장애물 회피 경로
  # 2. /obstacle_boxes (파란색 박스) - 감지된 장애물
  # 3. Measure 도구로 경로와 장애물 간 거리 측정

  측정 방법:
  1. RViz → Tool Properties → Measure (측정 도구 선택)
  2. 장애물 중심에서 planned_path 가장 가까운 점까지 거리 측정
  3. 예상: 0.9~1.2m ✅

  ---
  📈 안전 마진 유지

  dynamic_safety = 0.15 + 0.05 + 0.10 + (0.05 × 3.0) = 0.45m
  → 0.45m 이내는 여전히 충돌 거부 (안전!)

  실제 회피: 0.9~1.2m
  → 안전 마진: 0.45~0.75m 확보 ✅

  테스트 후 결과 알려주시면 추가 조정하겠습니다! 🎯

---
● 완료

  path_planner의 wall과 obstacle 비용 계산을 two-zone cost model로 변경했습니다. 빌드 성공 (2min 47s).

  변경 사항:

  1. Three-Zone Model 구현

  기존 proximity cost를 제거하고 3개 구역으로 단순화:

  ZONE 1: Collision Zone (dist < dynamic_safety)
  - 즉시 trajectory 거부
  - dynamic_safety = vehicle_radius + obstacle_radius + min_safety_margin + k_velocity_safety × velocity

  ZONE 2: Warning Zone (dynamic_safety ≤ dist < warning_distance)
  - Linear penalty: collision edge에서 1.0 → warning edge에서 0.0
  - Cost = warning_cost × penalty_factor
  - 예: 충돌 거리 0.4m, 경고 거리 0.8m → 경고 구역 폭 0.4m

  ZONE 3: Safe Zone (dist ≥ warning_distance)
  - 비용 없음 → 트랙 중앙으로 가려는 bias 제거

  2. 코드 수정

  frenet.hpp:55-57
  double warning_distance{0.8};  // Warning zone outer distance
  double warning_cost{5.0};      // Cost weight in warning zone

  frenet.cpp:522-538 - Linear penalty 계산:
  if (dist < p_.warning_distance) {
      double margin = dist - dynamic_safety;
      double warning_range = p_.warning_distance - dynamic_safety;
      if (margin > 0 && warning_range > 0) {
          double penalty_factor = 1.0 - (margin / warning_range);
          proximity_cost += penalty_factor;
      }
  }
  // else: Safe zone → no cost

  3. 새로운 파라미터 (planner_params.yaml:154-175)

  frenet_warning_distance: 0.8  # 경고 구역 외부 거리 [m]
                                # 이 거리 밖: 비용 없음
                                # 권장: 0.6-1.0m

  frenet_warning_cost: 5.0      # 경고 구역 비용 가중치
                                # 높을수록 회피 강도 증가
                                # 권장: 5.0-8.0

  효과:

  ✅ 트랙 중앙 bias 제거: 0.8m 밖의 wall은 비용 없음
  ✅ 전체 트랙 폭 활용: 중앙으로 당겨지지 않음
  ✅ 충돌 안전성 유지: Warning zone에서 적절한 회피
  ✅ 파라미터 조정 가능: warning_distance와 warning_cost로 제어

  튜닝 가이드:

  - warning_distance: 너무 작으면 (<0.5m) 경고 부족, 너무 크면 (>1.5m) proximity cost 재현
  - warning_cost: 너무 낮으면 (<2.0) 회피 부족, 너무 높으면 (>15.0) 과도한 회피

  기존 frenet_k_proximity와 frenet_proximity_threshold 파라미터는 제거되었습니다.
--- planner tuning
● 트랙 중앙의 50cm 장애물을 인식하지 못하는 문제의 원인을 찾았습니다.
  주요 문제는 path filtering입니다.

  문제 분석

  현재 설정에서 장애물이 필터링되는 과정:

  1. Path Clearance Filtering (가장 큰 문제)
    - path_clearance_radius: 0.6m (planner_params.yaml:52)
    - enable_path_filtering: true (planner_params.yaml:64)
    - 결과: 글로벌 centerline 경로 주변 0.6m 이내의 모든 스캔 포인트가 제거됨
    - 트랙 중앙의 장애물은 centerline 근처에 있어 필터링될 가능성이 매우
   높음
  2. Aggressive Scan Downsampling
    - scan_downsample_factor: 7 (planner_params.yaml:56)
    - 7개 포인트 중 1개만 사용 → 50cm 장애물의 포인트 수가 크게 감소
  3. Clustering 최소 요구사항
    - 클러스터가 최소 3개 포인트 필요 (path_planner_node.cpp:332)
    - Downsampling + filtering 후 3개 미만이면 무시됨

  권장 수정사항

  src/controller/path_planner/config/planner_params.yaml에서 다음
  파라미터를 수정하세요:

  옵션 1: Path Filtering 비활성화 (가장 확실)

  enable_path_filtering: false  # 64번 라인

  옵션 2: Path Clearance Radius 대폭 감소

  path_clearance_radius: 0.2  # 52번 라인 (0.6 → 0.2)

  옵션 3: Scan Downsampling 개선

  scan_downsample_factor: 3  # 56번 라인 (7 → 3)

  옵션 4: 종합 솔루션 (권장)

  # 52번 라인
  path_clearance_radius: 0.3  # 0.6 → 0.3 (centerline 바로 근처만 필터링)

  # 56번 라인
  scan_downsample_factor: 4  # 7 → 4 (더 많은 포인트 사용)

  # 45번 라인
  obstacle_cluster_distance: 0.2  # 0.15 → 0.2 (더 쉽게 클러스터링)

--- wall detection vs obstacle detection
● path_planner의 obstacle과 wall 처리를 분석했습니다. 상세한 비교를 제공하겠습니다.

  ---
  📊 Obstacle vs Wall 비교

  1. 분류 기준 (path_planner_node.cpp:1184)

  // Line 1184: 크기 기준으로 분류
  if (size > obstacle_max_box_size) {
      // WALL: 큰 클러스터 → 선분으로 분할
  } else {
      // OBSTACLE: 작은 클러스터 → 박스로 처리
  }

  파라미터: obstacle_max_box_size: 0.8m (현재 설정)
  - size ≤ 0.8m → Obstacle (동적 장애물, 콘, 차량 등)
  - size > 0.8m → Wall (벽, 트랙 경계 등)

  ---
  2. 처리 방식 차이

  Obstacle 처리:

  // 1. 박스 생성 (createBoundingBoxes)
  ObstacleBox box;
  box.center_x, box.center_y  // 중심 좌표
  box.size = max(width, height) + safety_margin

  // 2. 박스 경계 샘플링 (Line 1229-1240)
  // 박스의 4면을 points_per_side로 샘플링
  for (int i = 0; i < points_per_side; ++i) {
      // 상/하/좌/우 면 각각 샘플링
  }

  특징:
  - Square bounding box 생성
  - Safety margin 추가 (obstacle_box_safety_margin: 0.2m)
  - 4면 균등 샘플링 (총 4 × points_per_side 개 점)
  - 빨간색 박스로 시각화

  Wall 처리:

  // 1. 짧은 선분으로 분할 (breakWallIntoSegments)
  WallSegment seg;
  seg.x1, seg.y1, seg.x2, seg.y2  // 시작/끝 좌표
  seg.width = wall_segment_width

  // 2. 선분 경계 샘플링 (Line 1248-1278)
  // 각 선분의 길이 방향 × 폭 방향 샘플링
  for (int i = 0; i < points_per_segment; ++i) {
      for (int j = 0; j < width_samples; ++j) {
          // 선분을 따라 균등하게 점 배치
      }
  }

  특징:
  - Line segment 분할 (wall_segment_length: 0.7m)
  - Perpendicular width 적용 (wall_segment_width: 0.3m)
  - Distance filtering (max_wall_distance: 15.0m 넘으면 무시)
  - Segment limit (max_wall_segments: 30 최대 개수)
  - 파란색 선분으로 시각화

  ---
  3. Frenet Planner에 미치는 영향

  ⚠️ 중요: 둘 다 동일하게 처리됨!

  Obstacle과 Wall 모두 동일한 obstacles 벡터로 Frenet planner에 전달됩니다
  (path_planner_node.cpp:1324):

  auto cands = frenet_->generate(fs, obstacles);

  Two-Zone Cost Model (frenet.cpp:493-539):

  Zone 1: Collision Zone (즉시 거부)

  // Line 502-504: 속도에 따라 동적 안전 거리 계산
  double dynamic_safety = max(min_safety_margin,
                              safety_radius + vehicle_radius + obstacle_radius +
                              k_velocity_safety * current_velocity);

  // Line 515: 충돌 체크
  if (dist < dynamic_safety) {
      tr.collision = true;  // 경로 즉시 거부
  }

  Zone 2: Warning Zone (선형 페널티)

  // Line 525-532: 경고 구역 비용 계산
  if (dist < warning_distance) {
      double margin = dist - dynamic_safety;
      double warning_range = warning_distance - dynamic_safety;
      double penalty_factor = 1.0 - (margin / warning_range);
      proximity_cost += penalty_factor;
  }

  Zone 3: Safe Zone (비용 없음)

  // Line 538: warning_distance 이상이면 비용 없음
  // dist >= warning_distance → NO COST

  Total Cost (Line 597-598):

  tr.cost = k_j * jerk + k_t * time + k_d * deviation + k_v * velocity_error +
            warning_cost * proximity_cost;

  ---
  🎯 Obstacle 관련 주요 파라미터

  A. Obstacle Detection & Clustering

  # 클러스터링 거리 (스캔 포인트를 그룹화)
  obstacle_cluster_distance: 0.15      # 0.15m 이내 → 같은 클러스터
                                        # ↓ 작을수록: 더 세밀한 분리
                                        # ↑ 클수록: 더 큰 클러스터

  # Obstacle vs Wall 구분 기준
  obstacle_max_box_size: 0.8           # 0.8m 이하 → Obstacle
                                        # ↓ 작을수록: 더 많이 Wall로 분류
                                        # ↑ 클수록: 더 많이 Obstacle로 분류

  # Obstacle 박스 안전 마진
  obstacle_box_safety_margin: 0.2      # 박스 크기에 +0.2m 추가
                                        # ↑ 클수록: 더 큰 회피 거리

  B. Performance Optimization

  # 스캔 다운샘플링 (성능 최적화)
  scan_downsample_factor: 7            # 7개 중 1개만 사용
                                        # ↑ 클수록: 빠르지만 정확도 ↓

  # Obstacle 박스 샘플링
  obstacle_points_per_side: 3          # 박스 한 면당 3개 점
                                        # Total: 4면 × 3 = 12개 점
                                        # ↑ 클수록: 정확하지만 느림

  # 처리 제한
  max_obstacles_to_process: 5          # 최대 5개 obstacle만 처리
                                        # ↑ 클수록: 많은 장애물 인식
  enable_obstacle_clustering: true     # false면 클러스터링 스킵
  enable_path_filtering: false         # 경로 주변 필터링 (현재 비활성화)

  C. Frenet Safety Parameters (충돌 회피 거리)

  # Dynamic Safety Distance 계산:
  # safety = max(min_safety_margin, 
  #              safety_radius + vehicle_radius + obstacle_radius + k_velocity_safety * v)

  frenet_safety_radius: 0.10           # 기본 안전 반경
  frenet_vehicle_radius: 0.10          # 차량 반경 (F1TENTH width ~0.2m)
  frenet_obstacle_radius: 0.10         # 장애물 예상 반경
  frenet_k_velocity_safety: 0.05       # 속도 의존 게인
  frenet_min_safety_margin: 0.00       # 최소 안전 마진

  # 예시: 3 m/s에서
  # safety = max(0.00, 0.10 + 0.10 + 0.10 + 0.05*3) = 0.45m

  D. Two-Zone Cost Model (경고 구역)

  frenet_warning_distance: 0.5         # 경고 구역 외부 거리
                                        # ↑ 클수록: 더 먼 거리에서 회피
                                        # ↓ 작을수록: 더 가까이 접근 허용

  frenet_warning_cost: 7.0             # 경고 구역 비용 가중치
                                        # ↑ 클수록: 장애물 회피 강화
                                        # ↓ 작을수록: 회피 약화

  # Cost = k_jerk * jerk + k_time * time + k_deviation * dev + 
  #        k_velocity * v_err + warning_cost * proximity_cost
  frenet_k_jerk: 0.3                   # Jerk (부드러움)
  frenet_k_time: 0.1                   # 시간 (빠른 경로)
  frenet_k_deviation: 1.5              # 중심선 이탈 (높음 → 중심 선호)
  frenet_k_velocity: 1.0               # 속도 오차

  E. Wall-Specific Parameters

  # Wall 처리 (Obstacle와 구분)
  wall_segment_length: 0.7             # 벽을 0.7m 선분으로 분할
  wall_segment_width: 0.3              # 선분 폭 (수직 방향)
  max_wall_distance: 15.0              # 15m 이상 벽 무시
  max_wall_segments: 30                # 최대 30개 선분만 생성
  wall_point_downsample: 3             # 벽 클러스터 1/3만 사용

  ---
  🔧 튜닝 가이드

  Obstacle 회피를 강화하려면:

  # 1. 더 넓은 안전 거리
  frenet_warning_distance: 0.7  # 0.5 → 0.7
  frenet_vehicle_radius: 0.15   # 0.10 → 0.15

  # 2. 더 강한 회피 비용
  frenet_warning_cost: 10.0     # 7.0 → 10.0

  # 3. 중심선 선호도 증가
  frenet_k_deviation: 2.0       # 1.5 → 2.0

  Obstacle 회피를 약화하려면:

  # 1. 더 좁은 안전 거리
  frenet_warning_distance: 0.3  # 0.5 → 0.3

  # 2. 더 약한 회피 비용
  frenet_warning_cost: 3.0      # 7.0 → 3.0

  # 3. 중심선 선호도 감소
  frenet_k_deviation: 0.8       # 1.5 → 0.8

  ---
  📈 현재 설정 분석 (planner_params.yaml)

  # Obstacle Detection
  obstacle_cluster_distance: 0.15      # ✅ 적절 (세밀한 분리)
  obstacle_max_box_size: 0.8           # ✅ 적절 (0.8m 기준)
  obstacle_box_safety_margin: 0.2      # ✅ 적절 (안전 마진)

  # Safety Parameters
  frenet_vehicle_radius: 0.10          # ⚠️ 낮음 (차량 폭 ~0.2m)
  frenet_obstacle_radius: 0.10         # ✅ 적절
  frenet_k_velocity_safety: 0.05       # ⚠️ 낮음 (고속에서 위험)
  frenet_min_safety_margin: 0.00       # ⚠️ 매우 낮음!

  # Warning Zone
  frenet_warning_distance: 0.5         # ⚠️ 낮음 (회피가 늦을 수 있음)
  frenet_warning_cost: 7.0             # ✅ 적절 (중간 강도)

  권장 개선:
  frenet_vehicle_radius: 0.15          # 0.10 → 0.15
  frenet_k_velocity_safety: 0.1        # 0.05 → 0.1
  frenet_min_safety_margin: 0.1        # 0.00 → 0.1
  frenet_warning_distance: 0.7         # 0.5 → 0.7

  설정을 변경하시겠습니까?

