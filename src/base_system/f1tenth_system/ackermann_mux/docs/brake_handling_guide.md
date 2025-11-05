# Ackermann Mux Brake 처리 가이드

## 문제 상황
속도가 잘 안 줄어드는 이슈가 있는 경우, drive 토픽의 speed만으로는 빠른 감속이 어려울 수 있습니다. VESC는 별도의 brake 명령을 통해 더 강력한 제동을 지원합니다.

## 현재 시스템 구조

### 1. Ackermann 메시지 구조
```
ackermann_msgs/msg/AckermannDrive:
  - steering_angle: float32
  - steering_angle_velocity: float32
  - speed: float32          # 목표 속도 (m/s)
  - acceleration: float32   # 목표 가속도 (m/s²)
  - jerk: float32          # 목표 저크 (m/s³)
```

**주의**: AckermannDrive 메시지에는 `brake` 필드가 없습니다!

### 2. VESC Brake 토픽
VESC는 별도의 brake 토픽을 지원합니다:
```
토픽: /commands/motor/brake
타입: std_msgs/msg/Float64
데이터: 제동 전류 (Amps)
범위: brake_min ~ brake_max (기본값: -20000.0 ~ 200000.0)
```

### 3. 현재 Ackermann Mux 동작

`ackermann_mux.cpp`의 `VelocityTopicHandle::callback()`:
```cpp
void callback(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr msg)
{
    stamp_ = mux_->now();
    msg_ = *msg;

    if (mux_->hasPriority(*this)) {
        mux_->publishAckermann(msg);  // 그대로 전달만 함
    }
}
```

**현재 문제점**:
- 이전 속도와 비교하지 않음
- 감속 시 brake 신호를 자동으로 생성하지 않음
- 단순히 우선순위에 따라 메시지를 pass-through

## 추천 해결 방법

### 방법 1: Ackermann Mux에서 Brake 처리 추가 (권장)

Ackermann Mux에 감속 감지 및 brake 명령 생성 기능을 추가합니다.

#### 수정 위치
파일: `src/base_system/f1tenth_system/ackermann_mux/src/ackermann_mux.cpp`

#### 구현 내용

```cpp
// ackermann_mux.hpp에 추가
private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr brake_pub_;
    ackermann_msgs::msg::AckermannDriveStamped last_published_cmd_;
    double deceleration_threshold_;  // 감속 판정 임계값
    double brake_gain_;              // brake 전류 게인

// ackermann_mux.cpp의 init()에 추가
void AckermannMux::init()
{
    // ... 기존 코드 ...

    // Brake publisher 생성
    brake_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "commands/motor/brake", qos);

    // 파라미터 로드
    this->declare_parameter("deceleration_threshold", 0.5);  // m/s
    this->declare_parameter("brake_gain", 1000.0);           // Amps per m/s

    deceleration_threshold_ = this->get_parameter("deceleration_threshold")
        .as_double();
    brake_gain_ = this->get_parameter("brake_gain").as_double();

    // 초기화
    last_published_cmd_.drive.speed = 0.0;
}

// publishAckermann() 수정
void AckermannMux::publishAckermann(
    const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr & msg)
{
    // 현재 속도와 이전 속도 비교
    double current_speed = msg->drive.speed;
    double last_speed = last_published_cmd_.drive.speed;
    double speed_diff = current_speed - last_speed;

    // 감속 중인지 확인
    bool is_decelerating = (speed_diff < -deceleration_threshold_);

    if (is_decelerating) {
        // 감속량에 비례한 brake 전류 계산
        double brake_current = std::abs(speed_diff) * brake_gain_;

        // Brake 명령 발행
        auto brake_msg = std_msgs::msg::Float64();
        brake_msg.data = brake_current;
        brake_pub_->publish(brake_msg);

        RCLCPP_DEBUG(get_logger(),
            "Deceleration detected: %.2f m/s -> %.2f m/s, brake: %.0f A",
            last_speed, current_speed, brake_current);
    } else {
        // 가속 또는 유지 시에는 brake 해제
        auto brake_msg = std_msgs::msg::Float64();
        brake_msg.data = 0.0;
        brake_pub_->publish(brake_msg);
    }

    // Drive 명령 발행
    cmd_pub_->publish(*msg);

    // 현재 명령 저장
    last_published_cmd_ = *msg;
}
```

#### 설정 파일 추가
파일: `config/ackermann_mux_topics.yaml`

```yaml
# Brake 처리 파라미터
/**:
  ros__parameters:
    # 감속 판정 임계값 (m/s) - 이 값보다 크게 감속되면 brake 적용
    deceleration_threshold: 0.5

    # Brake 전류 게인 (Amps per m/s)
    # 예: 1 m/s 감속 시 1000 A의 brake 전류
    brake_gain: 1000.0

    # 최대 brake 전류 (Amps) - 안전을 위한 상한
    max_brake_current: 20000.0
```

#### 장점
- ✅ 중앙 집중식 처리 - 모든 컨트롤러가 자동으로 brake 혜택 받음
- ✅ 기존 컨트롤러 코드 수정 불필요
- ✅ 일관된 brake 정책 적용 가능

#### 단점
- ❌ Ackermann Mux 수정 필요 (시스템 코어 수정)
- ❌ 모든 입력 토픽에 동일한 brake 정책 적용

---

### 방법 2: 개별 컨트롤러에서 Brake 처리

각 컨트롤러(path_planner, pure_pursuit 등)에서 직접 brake 토픽을 발행합니다.

#### 구현 예시 (Pure Pursuit 컨트롤러)

```cpp
// pure_pursuit_node.hpp에 추가
private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr brake_pub_;
    double last_commanded_speed_;
    double brake_gain_;

// pure_pursuit_node.cpp
void PurePursuit::init()
{
    // ... 기존 코드 ...

    brake_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "/commands/motor/brake", qos);

    this->declare_parameter("brake_gain", 1000.0);
    brake_gain_ = this->get_parameter("brake_gain").as_double();

    last_commanded_speed_ = 0.0;
}

void PurePursuit::publishDriveCommand(double speed, double steering_angle)
{
    // Drive 명령 생성
    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    drive_msg.drive.speed = speed;
    drive_msg.drive.steering_angle = steering_angle;

    // 감속 확인 및 brake 적용
    double speed_diff = speed - last_commanded_speed_;
    if (speed_diff < -0.3) {  // 0.3 m/s 이상 감속
        double brake_current = std::abs(speed_diff) * brake_gain_;

        auto brake_msg = std_msgs::msg::Float64();
        brake_msg.data = brake_current;
        brake_pub_->publish(brake_msg);
    } else {
        // Brake 해제
        auto brake_msg = std_msgs::msg::Float64();
        brake_msg.data = 0.0;
        brake_pub_->publish(brake_msg);
    }

    drive_pub_->publish(drive_msg);
    last_commanded_speed_ = speed;
}
```

#### 장점
- ✅ 컨트롤러별 맞춤형 brake 정책 가능
- ✅ 시스템 코어 수정 불필요
- ✅ 더 정교한 제어 가능 (경로 정보 활용 가능)

#### 단점
- ❌ 모든 컨트롤러에 개별 구현 필요
- ❌ 코드 중복 발생
- ❌ 일관성 유지 어려움

---

### 방법 3: Brake 전용 노드 추가 (중간 방법)

Drive 토픽을 모니터링하고 brake 신호만 생성하는 독립 노드를 만듭니다.

#### 새 노드: `brake_controller_node`

```cpp
class BrakeController : public rclcpp::Node
{
public:
    BrakeController() : Node("brake_controller")
    {
        // Drive 토픽 구독
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
        qos.best_effort();

        drive_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "/ackermann_cmd", qos,
            std::bind(&BrakeController::driveCallback, this, std::placeholders::_1));

        brake_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/commands/motor/brake", qos);

        // 파라미터
        this->declare_parameter("deceleration_threshold", 0.5);
        this->declare_parameter("brake_gain", 1000.0);
        this->declare_parameter("max_brake_current", 20000.0);

        decel_threshold_ = this->get_parameter("deceleration_threshold").as_double();
        brake_gain_ = this->get_parameter("brake_gain").as_double();
        max_brake_ = this->get_parameter("max_brake_current").as_double();

        last_speed_ = 0.0;
    }

private:
    void driveCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
    {
        double current_speed = msg->drive.speed;
        double speed_diff = current_speed - last_speed_;

        auto brake_msg = std_msgs::msg::Float64();

        if (speed_diff < -decel_threshold_) {
            // 감속 중 - brake 적용
            double brake_current = std::abs(speed_diff) * brake_gain_;
            brake_msg.data = std::min(brake_current, max_brake_);

            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "Braking: %.2f m/s -> %.2f m/s, brake: %.0f A",
                last_speed_, current_speed, brake_msg.data);
        } else {
            // 가속/유지 - brake 해제
            brake_msg.data = 0.0;
        }

        brake_pub_->publish(brake_msg);
        last_speed_ = current_speed;
    }

    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr brake_pub_;

    double last_speed_;
    double decel_threshold_;
    double brake_gain_;
    double max_brake_;
};
```

#### Launch 파일에 추가
```python
Node(
    package='brake_controller',
    executable='brake_controller_node',
    name='brake_controller',
    parameters=[{
        'deceleration_threshold': 0.5,
        'brake_gain': 1000.0,
        'max_brake_current': 20000.0,
    }]
)
```

#### 장점
- ✅ 시스템 분리 - 독립적으로 개발/테스트 가능
- ✅ 기존 코드 수정 최소화
- ✅ 쉽게 활성화/비활성화 가능
- ✅ 실시간 파라미터 튜닝 가능

#### 단점
- ❌ 추가 노드로 인한 약간의 지연
- ❌ 새 패키지 생성 필요

---

## 파라미터 튜닝 가이드

### 1. deceleration_threshold (감속 임계값)
```yaml
deceleration_threshold: 0.5  # m/s
```
- **낮은 값 (0.1-0.3)**: 작은 감속에도 brake 적용, 민감한 제동
- **중간 값 (0.5-1.0)**: 일반적인 경우 권장
- **높은 값 (1.0+)**: 급감속에만 brake 적용, 부드러운 주행

### 2. brake_gain (제동 게인)
```yaml
brake_gain: 1000.0  # Amps per (m/s)
```
- **계산**: `brake_current = abs(speed_diff) * brake_gain`
- **예시**: 1 m/s 감속 시 1000 A 적용
- **낮은 게인 (500-800)**: 부드러운 제동
- **높은 게인 (1500-2000)**: 강력한 제동

### 3. max_brake_current (최대 제동 전류)
```yaml
max_brake_current: 20000.0  # Amps
```
- 안전을 위한 상한값
- VESC 설정의 `brake_max`보다 작아야 함

---

## 테스트 방법

### 1. 수동 테스트
```bash
# Terminal 1: Drive 명령 발행
ros2 topic pub /drive ackermann_msgs/msg/AckermannDriveStamped \
  "{drive: {speed: 2.0}}" --once

# 2초 후
ros2 topic pub /drive ackermann_msgs/msg/AckermannDriveStamped \
  "{drive: {speed: 0.5}}" --once

# Terminal 2: Brake 토픽 모니터링
ros2 topic echo /commands/motor/brake
```

### 2. Rqt Plot으로 시각화
```bash
rqt_plot /ackermann_cmd/drive/speed /commands/motor/brake/data
```

### 3. Bag 파일 분석
```bash
ros2 bag record /ackermann_cmd /commands/motor/brake
ros2 bag play <bag_file>
```

---

## 최종 권장 사항

**권장 방법**: **방법 3 (Brake 전용 노드)**

이유:
1. 기존 시스템 수정 최소화
2. 독립적으로 개발/테스트 가능
3. 실시간 파라미터 조정 용이
4. 문제 발생 시 쉽게 비활성화

**구현 순서**:
1. `brake_controller` 패키지 생성
2. 노드 구현 및 테스트
3. Launch 파일에 통합
4. 실제 주행 테스트 및 파라미터 튜닝
5. 성능 개선 확인

**성공 지표**:
- 감속 거리 단축
- 더 짧은 제동 시간
- 부드러운 속도 프로파일
- 랩타임 개선

---

## 참고 문서

- VESC 드라이버: `src/base_system/f1tenth_system/vesc/vesc_driver/`
- Ackermann Mux: `src/base_system/f1tenth_system/ackermann_mux/`
- VESC 설정: `f1tenth_stack/config/vesc.yaml`
