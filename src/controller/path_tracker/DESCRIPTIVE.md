● path_tracker 노드의 steering과 speed 계산 과정과 상호 영향을 분석하여 설명드리겠습니다.

  Steering과 Speed의 상호 영향 관계

  1️⃣ Steering → Speed 영향 (제동 효과)

  Friction Circle Constraint (path_tracker_node.cpp:1172-1227)
  a_total² = a_lateral² + a_longitudinal²

  - Lateral Acceleration 계산 (line 1152-1169):
  curvature = tan(steering_angle) / wheelbase
  lateral_accel = v² × |curvature|
  - 결과: Steering angle이 크면
    - ① Lateral acceleration ↑
    - ② Friction circle에서 longitudinal acceleration 여유 ↓
    - ③ 가속/감속 능력 제한 → Speed 제한

  예시:
  - 큰 steering (0.4 rad) + 5 m/s → lateral_accel ≈ 6 m/s²
  - 만약 max_total_accel = 9.81 m/s² → 남은 longitudinal_accel = √(9.81² - 6²) ≈ 7.6 m/s²
  - 급가속 제한됨!

  ---
  2️⃣ Speed → Steering 영향 (안정성 조정)

  A. Adaptive Lookahead Distance (line 449-470)
  base_lookahead = lookahead_base + lookahead_k × velocity
  // 속도 높을수록 → 더 먼 지점을 목표로 → 부드러운 steering

  B. Stanley Lateral Error Correction (line 1293-1319)
  stanley_term = atan(k × lateral_error / velocity)
  // 속도 높을수록 → error correction 감소 → 급격한 steering 방지

  C. Lateral Acceleration 제한 (line 1152-1169)
  lateral_accel = v² × curvature
  // 같은 steering에서도 속도 높으면 → 더 큰 lateral force 필요

  결과: Speed가 높으면
  - ① Lookahead distance 증가 → 더 smooth한 steering 경로
  - ② Stanley correction 감소 → 안정성 우선
  - ③ 같은 steering에서도 lateral acceleration 증가 → curvature 제약

  ---
  3️⃣ Curvature가 중개 역할

  Path curvature (κ)가 steering과 speed를 연결:

  Path Curvature → Adaptive Lookahead (line 1266-1289)
  curv_adjustment = k_curvature / (|curvature| + ε)
  // High curvature (급커브) → lookahead 감소 → 민첩한 steering
  // Low curvature (직선) → lookahead 증가 → 안정적 steering

  Path Curvature → Speed Limit (line 929-959)
  v_max = sqrt(friction_coeff × g / |kappa|)
  // Curvature 클수록 → 안전 속도 감소

  Steering Angle → Curvature → Lateral Acceleration
  curvature ≈ tan(steering_angle) / wheelbase
  lateral_accel = v² × |curvature|

  ---
  4️⃣ Speed Mode별 상호작용

  CURVATURE_BASED Mode (line 825-828, 929-959):
  double kappa = computeCurvature(...);
  v_max = sqrt(μ × g / |kappa|);
  - Path curvature로 최대 속도 결정
  - Steering이 크면 → curvature 크므로 → speed 자동 감소

  OPTIMIZE Mode (line 830-871):
  speed = min(path_speed, curv_speed);
  // 또는 weighted: w1×path_speed + w2×curv_speed
  - Path velocity와 curvature-based speed 중 작은 값 선택
  - 안전성 우선 전략

  Friction Circle Limiting (line 572-585):
  if (use_acceleration_limit_) {
      commanded_speed = applyAccelerationLimit(target_speed, v, dt, steering);
  }
  - 실시간으로 steering에 따라 speed 조정

  ---
  종합 흐름도

  ┌─────────────────┐
  │  Path Planner   │ → Generates trajectory with curvature (κ)
  └────────┬────────┘
           │
           v
  ┌─────────────────────────────────────────────────────┐
  │           PATH TRACKER (odom callback)              │
  ├─────────────────────────────────────────────────────┤
  │                                                     │
  │  [STEERING CALCULATION]                            │
  │  1. Closest point → lateral_error                  │
  │  2. Curvature → adaptive_lookahead                 │
  │     ├─ High κ → Short lookahead → Sharp steering  │
  │     └─ Low κ → Long lookahead → Smooth steering   │
  │  3. Pure Pursuit + Stanley + Heading Control       │
  │     └─ stanley = atan(k × e / v) ← Speed 의존!    │
  │  4. Low-pass filter → steering_angle               │
  │                                                     │
  │  [SPEED CALCULATION]                               │
  │  5. Speed mode:                                     │
  │     ├─ PATH_VELOCITY: Use path velocity           │
  │     ├─ CURVATURE: v_max = √(μg/κ) ← Curvature!   │
  │     └─ OPTIMIZE: min(path_v, curv_v)              │
  │                                                     │
  │  [FRICTION CIRCLE LIMITING]  ⚡ 핵심 상호작용!      │
  │  6. a_lat = v² × |tan(steering)/wheelbase|        │
  │     ├─ Steering ↑ → a_lat ↑                       │
  │     └─ Speed ↑ → a_lat ↑↑ (제곱)                  │
  │  7. a_long_max = √(a_total² - a_lat²)             │
  │     └─ a_lat 클수록 → 가속/감속 능력 ↓            │
  │  8. commanded_speed = limited by friction circle   │
  │                                                     │
  └─────────────────────────────────────────────────────┘
           │
           v
      VESC Motor

  ---
  실제 주행 시나리오

  급커브 진입 상황:
  1. Path curvature 감지 → lookahead distance 감소
  2. Steering angle 증가 (급회전)
  3. Friction Circle 작동:
    - lateral_accel = v² × tan(steering)/wheelbase 증가
    - longitudinal acceleration 여유 감소
    - Speed 자동 감소 (line 1197-1227)
  4. 속도 감소 → lateral acceleration 재계산 → 안정화

  직선 가속 상황:
  1. Path curvature 낮음 → lookahead distance 증가
  2. Steering angle 작음 (직진)
  3. Friction Circle 작동:
    - lateral_accel 작음
    - longitudinal acceleration 여유 충분
    - Full acceleration 가능

  ---
  핵심 파라미터

  path_tracker/config/tracker_params.yaml:
  # Steering에 영향
  lookahead_base: 1.5      # 기본 lookahead
  lookahead_k: 0.3         # 속도 의존성 (v↑ → lookahead↑)
  stanley_k: 0.5           # Lateral error correction
  k_curvature: 0.5         # Curvature → lookahead 조정
  steering_alpha: 0.3      # Low-pass filter (smoothing)

  # Speed에 영향
  friction_coeff: 0.9      # μ (타이어 마찰계수)
  max_total_acceleration: 9.81  # Friction circle 한계

  # Friction Circle 활성화
  use_acceleration_limit: true

  이러한 상호작용을 통해 path_tracker는 물리적 한계 내에서 안전하고 빠른 주행을 실현합니다.
