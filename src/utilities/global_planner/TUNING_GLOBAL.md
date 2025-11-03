● 곡선 진입 속도를 높이기 위해 튜닝할 수 있는 주요 파라미터들을 찾았습니다.
  global_planner_offline_params.yaml에서 다음 파라미터들을 조정할 수 있습니다:

  🎯 곡선 진입 속도 관련 주요 파라미터

  1. curvature_gain (Line 45) - 가장 중요

  curvature_gain: 1.2  # 현재 값
  - 공식: vx = max_speed / (1 + curvature_gain × |kappa|)
  - 낮출수록: 곡선에서 속도 감소가 적어짐
  - 권장 범위: 0.8 ~ 1.5
  - 예시: 1.2 → 0.9로 낮추면 곡선 속도 약 15-20% 증가

  2. max_speed (Line 41)

  max_speed: 12.0  # m/s
  - 전체 최고 속도 제한
  - 직선 및 곡선 모두에 영향

  3. max_accel / max_decel (Line 43-44)

  max_accel: 3.0   # m/s^2
  max_decel: -3.0  # m/s^2
  - 곡선 진입 전 더 빠른 감속이 필요하면 max_decel을 -4.0 이상으로 증가
  - 곡선 탈출 후 더 빠른 가속이 필요하면 max_accel을 4.0 이상으로 증가

  4. safety_width (Line 19, 23)

  safety_width: 1.2      # IQP trajectory
  safety_width_sp: 1.2   # Shortest path
  - 낮출수록: 더 타이트한 레이싱 라인 사용 가능 → 곡선 속도 증가
  - 주의: 너무 낮으면 충돌 위험
  - 권장 범위: 0.8 ~ 1.5m

  🚀 추천 튜닝 전략

  보수적 접근 (안전성 우선)

  curvature_gain: 1.0     # 1.2 → 1.0 (약 10% 속도 증가)
  max_accel: 3.5          # 3.0 → 3.5
  max_decel: -3.5         # -3.0 → -3.5
  safety_width: 1.1       # 1.2 → 1.1

  공격적 접근 (속도 우선)

  curvature_gain: 0.8     # 1.2 → 0.8 (약 20-25% 속도 증가)
  max_accel: 4.0          # 3.0 → 4.0
  max_decel: -4.5         # -3.0 → -4.5
  safety_width: 0.9       # 1.2 → 0.9

  TUM global trajectory optimizer의 추가 설정도 확인해보시겠습니까?

--- Tum optimizer 설정
● TUM Global Trajectory Optimizer 추가 설정

  곡선 진입 속도를 높이기 위해 변경할 수 있는 TUM optimizer 설정을 찾았습니다.

  📁 주요 설정 파일

  1. racecar_dawgs.ini

  src/utilities/global_planner/config/global_planner/racecar_dawgs.ini

  2. ggv.csv (G-G Diagram)

  src/utilities/global_planner/config/global_planner/veh_dyn_info/ggv.csv

  ---
  🎯 곡선 진입 속도 관련 핵심 파라미터

  [GENERAL_OPTIONS] Section

  1. v_max (Line 45) - 최대 속도 제한

  veh_params = {"v_max":15.0,  # 현재 15 m/s
  - 역할: TUM optimizer가 계산하는 최대 속도 상한선
  - 권장: 15.0 → 18.0 (곡선 진입 및 직선 속도 모두 증가)

  2. curvlim (Line 50) - 차량 최대 곡률 한계

  "curvlim": 1.0,  # rad/m
  - 역할: 차량이 달릴 수 있는 최대 곡률 (급격한 회전 가능 여부)
  - 높일수록: 더 타이트한 코너를 더 빠르게 통과 가능
  - 권장: 1.0 → 1.2 또는 1.3

  3. dyn_model_exp (Line 57) - 차량 동역학 모델 지수

  vel_calc_opts = {"dyn_model_exp": 1.0,  # 범위 [1.0, 2.0]
  - 역할: 속도 프로파일 계산에 사용되는 모델 지수
  - 낮을수록: 더 선형적인 속도 프로파일 (부드러운 가속)
  - 높을수록: 더 공격적인 속도 프로파일 (곡선에서 더 빠름)
  - 권장: 1.0 → 1.3 (공격적 레이싱)

  ---
  [OPTIMIZATION_OPTIONS] Section

  4. width_opt (minimum curvature) (Line 73) - IQP 최적화 폭

  optim_opts_mincurv={"width_opt": 0.8,  # meters
  - 역할: 최적 경로 계산 시 차량 안전 폭
  - 낮을수록: 더 타이트한 레이싱 라인 → 곡선 속도 증가
  - 권장: 0.8 → 0.6 (공격적) 또는 0.7 (보수적)

  5. mue (Line 102) - 마찰 계수

  optim_opts_mintime={"width_opt": 0.8,
                      "mue": 1.0,  # 타이어-노면 마찰 계수
  - 역할: 타이어 그립 수준 결정 (Magic Formula D = F_z × mue)
  - 높을수록: 더 높은 횡방향 가속도 허용 → 곡선 속도 증가
  - 권장:
    - 보수적: 1.0 → 1.1
    - 공격적: 1.0 → 1.3 (실제 타이어 그립 테스트 필요)

  6. penalty_delta (Line 100) - 조향 부드러움 가중치

  "penalty_delta": 1.0,  # 범위 [0.0, 50.0]
  - 낮을수록: 더 급격한 조향 허용 → 코너 진입 더 빠름
  - 권장: 1.0 → 0.5 (조향 민첩성 증가)

  ---
  Tire Parameters (Line 169-178)

  7. 타이어 Magic Formula 파라미터

  tire_params_mintime = {"c_roll": 0.010,  # 구름 저항
                         "B_front": 7.4,    # 형상 계수
                         "C_front": 1.2,    # 형상 계수
                         "E_front": 0.85,   # 곡률 계수
  - B (강성 계수): 높을수록 타이어 반응 빠름
    - 권장: 7.4 → 8.0 (더 민감한 조향)
  - C (형상 계수): 피크 슬립각 조정
    - 권장: 1.2 → 1.3 (더 높은 피크 그립)

  ---
  Vehicle Dynamics - ggv.csv (G-G Diagram)

  8. 속도별 가속도 한계

  src/utilities/global_planner/config/global_planner/veh_dyn_info/ggv.csv

  현재 설정:
  # v_mps,ax_max_mps2,ay_max_mps2
  0.0,12.0,12.0
  4.0,12.0,12.0
  8.0,12.0,12.0

  역할: 각 속도에서 차량의 종방향(ax) 및 횡방향(ay) 가속도 한계

  곡선 속도 증가 방법:
  # v_mps,ax_max_mps2,ay_max_mps2
  0.0,15.0,15.0    # 저속 가속도 증가
  4.0,14.0,14.0
  8.0,13.0,13.0
  12.0,12.0,12.0   # 고속 유지

  ---
  🚀 추천 튜닝 전략

  보수적 접근 (안전성 우선)

  # racecar_dawgs.ini 변경
  veh_params = {"v_max":16.0,           # 15.0 → 16.0
                "curvlim": 1.1,         # 1.0 → 1.1

  vel_calc_opts = {"dyn_model_exp": 1.2,  # 1.0 → 1.2

  optim_opts_mincurv={"width_opt": 0.7,   # 0.8 → 0.7

  optim_opts_mintime={"mue": 1.1,         # 1.0 → 1.1
                      "penalty_delta": 0.7,  # 1.0 → 0.7

  공격적 접근 (속도 우선)

  # racecar_dawgs.ini 변경
  veh_params = {"v_max":18.0,           # 15.0 → 18.0
                "curvlim": 1.3,         # 1.0 → 1.3

  vel_calc_opts = {"dyn_model_exp": 1.5,  # 1.0 → 1.5

  optim_opts_mincurv={"width_opt": 0.6,   # 0.8 → 0.6

  optim_opts_mintime={"mue": 1.3,         # 1.0 → 1.3
                      "penalty_delta": 0.3,  # 1.0 → 0.3

  tire_params_mintime = {"B_front": 8.5,  # 7.4 → 8.5
                         "C_front": 1.3,  # 1.2 → 1.3

  ggv.csv 변경 (공격적):
  # v_mps,ax_max_mps2,ay_max_mps2
  0.0,18.0,18.0
  4.0,16.0,16.0
  8.0,14.0,14.0
  12.0,13.0,13.0
  16.0,12.0,12.0

  ---
  📝 변경 후 적용 방법

  1. 파일 수정 후 trajectory 재생성

  # 오프라인 글로벌 플래너 실행
  ros2 launch global_planner global_planner_offline.launch.py

  # 생성된 waypoint CSV 확인
  ls ~/f1tenth_dawgs/src/peripheral/maps/mohyun_1029/

  2. 생성된 trajectory 테스트

  # Path tracker로 테스트
  ros2 launch path_tracker path_tracker.launch.py

  3. 실제 차량 동역학 확인 필요

  - 중요: mue, ggv 값은 실제 차량 테스트 데이터 기반으로 조정해야 합니다
  - Friction circle 분석: src/controller/vehicle_calibration/ 참고

  원하는 파라미터를 수정하시겠습니까? 아니면 특정 파라미터에 대해 더 자세한 설명이 필요하신가요?

--- tuning 
● Global Path 생성 프로세스 및 파라미터 설명

  Global planner에서 global path를 만드는 과정은 크게 3단계로 구성됩니다:

  📍 1단계: Centerline 추출 및 Interpolation

  프로세스:
  Binary Map → Skeletonization → Extract Centerline → Smooth → Interpolate → Coordinate Transform

  관련 파라미터:

  global_planner_offline_params.yaml:

  - resample_step: 0.1 (meters)
    - Centerline을 균일한 간격으로 재샘플링
    - 최종 waypoint 간격 결정
    - 더 작은 값 = 더 조밀한 waypoints (계산량 증가)
  - smooth_window: 7
    - Moving average 윈도우 크기
    - Centerline 노이즈 제거용
    - 더 큰 값 = 더 부드러운 경로 (세부사항 손실)

  config/global_planner/racecar_dawgs.ini:

  - stepsize_prep: 0.05 (meters)
    - Spline approximation 전 linear interpolation 간격
    - Initial track point 밀도
    - 더 작은 값 = 더 정확한 초기 표현
  - stepsize_reg: 0.2 (meters)
    - Optimization 중 spline interpolation 간격
    - Track normal의 개수 결정
    - Trade-off: 정확도 vs 계산 속도
  - stepsize_interp_after_opt: 0.1 (meters)
    - 최적화 후 최종 spline interpolation 간격
    - 출력 trajectory의 해상도
    - 일반적으로 resample_step과 동일

  ---
  🎯 2단계: B-Spline Smoothing

  프로세스:
  Interpolated Centerline → B-Spline Regression → Smoothed Track Representation

  관련 파라미터 (racecar_dawgs.ini):

  - k_reg: 3
    - B-spline의 차수 (order)
    - 3 = cubic splines (표준)
    - 더 높은 값 = 더 부드러운 곡선 (과도하게 부드러워질 위험)
  - s_reg: 1
    - Spline regression smoothing factor [1.0 ~ 100.0]
    - 낮은 값 (1) = 원본 데이터에 가깝게
    - 높은 값 (100) = 더 부드럽게 (정확도 감소)
    - 현재값 1 = 최소 smoothing, 최대 정확도

  ---
  🚗 3단계: Velocity Profile Smoothing

  프로세스:
  Optimized Trajectory → Velocity Calculation → Moving Average Filter → Final Velocity Profile

  관련 파라미터:

  global_planner_offline_params.yaml:

  - curvature_gain: 1.2
    - 속도 프로파일 생성 시 곡률 영향도
    - vx = max_speed / (1 + curvature_gain * |kappa|)
    - 더 큰 값 = 코너에서 더 느리게
  - max_speed: 12.0 (m/s)
    - 목표 최대 속도
    - 직선 구간 속도 상한

  racecar_dawgs.ini:

  - vel_profile_conv_filt_window: null
    - Velocity profile moving average 필터 윈도우
    - 현재 비활성화 (null)
    - 활성화하려면 정수 값 설정 (예: 5, 7)
    - 속도 변화를 부드럽게 만듦

  ---
  🔧 파라미터 튜닝 가이드

  더 부드러운 경로를 원한다면:

  # global_planner_offline_params.yaml
  smooth_window: 9  # 7 → 9로 증가
  # racecar_dawgs.ini
  s_reg: 3  # 1 → 3으로 증가 (smoothing factor)

  더 정밀한 경로를 원한다면:

  # global_planner_offline_params.yaml
  resample_step: 0.05  # 0.1 → 0.05로 감소 (2배 밀도)
  # racecar_dawgs.ini
  stepsize_reg: 0.1  # 0.2 → 0.1로 감소

  Velocity profile을 부드럽게 만들려면:

  # racecar_dawgs.ini
  vel_profile_conv_filt_window: 7  # null → 7로 활성화

  ---
  📊 파라미터 위치 요약

  | 파라미터                         | 파일   | 경로
                            | 기본값   | 역할                 |
  |------------------------------|------|---------------------------------------------------------
  ------------------|-------|--------------------|
  | resample_step                | YAML |
  src/utilities/global_planner/config/global_planner_offline_params.yaml:28 | 0.1m  | Centerline
  보간 간격   |
  | smooth_window                | YAML |
  src/utilities/global_planner/config/global_planner_offline_params.yaml:46 | 7     | Moving
  average 윈도우 |
  | stepsize_prep                | INI  |
  src/utilities/global_planner/config/global_planner/racecar_dawgs.ini:14   | 0.05m | 초기
  interpolation   |
  | stepsize_reg                 | INI  |
  src/utilities/global_planner/config/global_planner/racecar_dawgs.ini:15   | 0.2m  | Optimization
   중     |
  | stepsize_interp_after_opt    | INI  |
  src/utilities/global_planner/config/global_planner/racecar_dawgs.ini:16   | 0.1m  | 최적화 후
             |
  | k_reg                        | INI  |
  src/utilities/global_planner/config/global_planner/racecar_dawgs.ini:22   | 3     | B-spline
  차수        |
  | s_reg                        | INI  |
  src/utilities/global_planner/config/global_planner/racecar_dawgs.ini:23   | 1     | Smoothing
  factor   |
  | vel_profile_conv_filt_window | INI  |
  src/utilities/global_planner/config/global_planner/racecar_dawgs.ini:60   | null  | 속도
  smoothing       |

  이 파라미터들을 조정하여 트랙 특성과 주행 스타일에 맞는 최적의 global path를 생성할 수 있습니다.
