#include <cmath>
#include <cstring>

#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/pid/pid.hpp"

extern "C" float get_imu_yaw();
extern "C" sp::DBusSwitchMode get_remote_right_switch();
extern "C" sp::DBusSwitchMode get_remote_left_switch();

extern sp::DBus remote;

sp::CAN can1(&hcan1);

sp::RM_Motor motor6020_A(1, sp::RM_Motors::GM6020, 1.0f, true);

sp::RM_Motor motor6020_B(2, sp::RM_Motors::GM6020, 1.0f, true);

// 保留原来的绘图接口。
float target_angle_A = 0.0f;
float target_angle_B = 0.0f;

float test_mode = 0.0f;
float test_torque_A = 0.0f;

// 新增调试变量：0未识别手拨、1手拨A、2手拨B。
float test_driver = 0.0f;

namespace
{

constexpr float PI = 3.14159265358979323846f;
constexpr float TWO_PI = 2.0f * PI;
constexpr float DT = 0.001f;

// 保留当前已经能驱动电机的参数。
constexpr float KP_A = 0.8f;
constexpr float KI_A = 0.0f;
constexpr float KD_A = 0.02f;

constexpr float KP_B = 0.8f;
constexpr float KI_B = 0.0f;
constexpr float KD_B = 0.02f;

constexpr float MAX_TORQUE = 0.40f;
constexpr float MAX_INTEGRAL = 0.05f;

constexpr float ERROR_DEADBAND = 0.00873f;  // 0.5°

// 编码器正方向与C板yaw正方向一致用1，相反用-1。
constexpr float DIR_A = 1.0f;
constexpr float DIR_B = 1.0f;

constexpr float R_ZERO_A = 0.0f;
constexpr float R_ZERO_B = 0.0f;

// ================= 手动识别参数 =================

// 用50ms内的板角度变化判断板是否在转，
// 避免只依赖相邻1ms的微小变化。
constexpr uint32_t BOARD_SAMPLE_MS = 50;
constexpr float BOARD_MOVE_ANGLE = 0.00175f;  // 约0.1°
constexpr uint32_t BOARD_SETTLE_MS = 500;

// 每20ms检查一次编码器位置变化。
constexpr uint32_t HAND_SAMPLE_MS = 20;

// 电机偏离目标约1°，并向外累计移动约0.5°，
// 才认定为手动输入。
constexpr float HAND_ERROR = 0.01745f;
constexpr float HAND_TRAVEL = 0.00873f;

// 小于一个编码器计数，过滤浮点微小变化。
constexpr float POSITION_EPS = 0.0002f;

// 手动状态下，累计移动约两个编码器计数，
// 就刷新“仍在手拨”的计时。
constexpr float HAND_KEEP_MOVE = 0.0015f;

// 停止拨动约500ms后，退出手动输入状态。
constexpr uint32_t HAND_RELEASE_MS = 500;

enum class Mode
{
  DISABLED,
  FOLLOW,
  RESET
};

enum class Driver
{
  NONE,
  A,
  B
};

float clamp(float value, float limit)
{
  if (value > limit) return limit;
  if (value < -limit) return -limit;
  return value;
}

float wrap(float angle)
{
  while (angle > PI) angle -= TWO_PI;
  while (angle < -PI) angle += TWO_PI;
  return angle;
}

float deadband(float error)
{
  if (error > ERROR_DEADBAND) {
    return error - ERROR_DEADBAND;
  }

  if (error < -ERROR_DEADBAND) {
    return error + ERROR_DEADBAND;
  }

  return 0.0f;
}

// ================= 摩擦补偿 =================

struct FrictionCompensation
{
  float torque;
  float full_error;

  float calculate(float error) const { return torque * clamp(error / full_error, 1.0f); }
};

FrictionCompensation friction_a{0.02f, 0.05236f};
FrictionCompensation friction_b{0.02f, 0.05236f};

// ================= 角度单环PID =================

// angular=false：保留多圈误差。
// alpha=0.1：对D项滤波。
sp::PID pid_a(DT, KP_A, KI_A, KD_A, MAX_TORQUE, MAX_INTEGRAL, 0.1f, false);

sp::PID pid_b(DT, KP_B, KI_B, KD_B, MAX_TORQUE, MAX_INTEGRAL, 0.1f, false);

void clear_control()
{
  pid_a.clear();
  pid_b.clear();
}

float control(sp::PID & pid, const FrictionCompensation & friction, float target, float actual)
{
  const float error = deadband(target - actual);

  pid.calc(actual + error, actual);

  return clamp(pid.out + friction.calculate(error), MAX_TORQUE);
}

float ratio_for(sp::DBusSwitchMode left)
{
  switch (left) {
    case sp::DBusSwitchMode::DOWN:
      return 0.5f;

    case sp::DBusSwitchMode::MID:
      return -1.0f;

    case sp::DBusSwitchMode::UP:
      return 3.0f;

    default:
      return 0.5f;
  }
}

// ================= 新的手动识别模块 =================

struct HandDetector
{
  float last_a = 0.0f;
  float last_b = 0.0f;

  float last_error_a = 0.0f;
  float last_error_b = 0.0f;

  float outward_a = 0.0f;
  float outward_b = 0.0f;

  uint32_t last_sample = 0;

  void reset(float a, float b, float target_a, float target_b, uint32_t now)
  {
    last_a = a;
    last_b = b;

    last_error_a = a - target_a;
    last_error_b = b - target_b;

    outward_a = 0.0f;
    outward_b = 0.0f;

    last_sample = now;
  }

  // 只累计“实际位置正在远离目标”的移动。
  // 正常追赶目标时，误差减小，不作为手动输入。
  void update_outward(float movement, float error, float previous_error, float & outward)
  {
    if (std::fabs(movement) <= POSITION_EPS) {
      return;
    }

    const bool moving_away =
      movement * error > 0.0f && std::fabs(error) > std::fabs(previous_error) + POSITION_EPS;

    if (moving_away) {
      outward += std::fabs(movement);
    }
    else {
      outward = 0.0f;
    }
  }

  Driver detect(float a, float b, float target_a, float target_b, float ratio, uint32_t now)
  {
    if (now - last_sample < HAND_SAMPLE_MS) {
      return Driver::NONE;
    }

    const float error_a = a - target_a;
    const float error_b = b - target_b;

    update_outward(a - last_a, error_a, last_error_a, outward_a);

    update_outward(b - last_b, error_b, last_error_b, outward_b);

    last_a = a;
    last_b = b;

    last_error_a = error_a;
    last_error_b = error_b;

    last_sample = now;

    const bool hand_a = std::fabs(error_a) >= HAND_ERROR && outward_a >= HAND_TRAVEL;

    const bool hand_b = std::fabs(error_b) >= HAND_ERROR && outward_b >= HAND_TRAVEL;

    if (hand_a && hand_b) {
      // 同时满足时，以换算到A后的外部位移较大者为输入。
      return outward_a >= outward_b / std::fabs(ratio) ? Driver::A : Driver::B;
    }

    if (hand_a) return Driver::A;
    if (hand_b) return Driver::B;

    return Driver::NONE;
  }
};

// ================= CAN发送 =================

void send_commands(float torque_a, float torque_b)
{
  motor6020_A.cmd(DIR_A * torque_a);
  motor6020_B.cmd(DIR_B * torque_b);

  std::memset(can1.tx_data, 0, 8);

  motor6020_A.write(can1.tx_data);
  motor6020_B.write(can1.tx_data);

  can1.send(motor6020_A.tx_id);
}

}  // namespace

extern "C" void can_task()
{
  can1.config();
  can1.start();

  Mode previous_mode = Mode::DISABLED;
  Driver driver = Driver::NONE;

  HandDetector hand_detector;

  bool yaw_ready = false;

  float last_yaw = 0.0f;
  float total_yaw = 0.0f;

  float board_sample_yaw = 0.0f;
  uint32_t board_sample_time = 0;
  uint32_t last_board_move = 0;

  float yaw_ref = 0.0f;
  float a_ref = 0.0f;
  float b_ref = 0.0f;

  float target_a = 0.0f;
  float target_b = 0.0f;

  float ratio = 1.0f;

  float last_hand_position = 0.0f;
  uint32_t last_hand_move = 0;

  uint32_t wake_time = osKernelSysTick();

  while (true) {
    const auto right = get_remote_right_switch();

    const float yaw = get_imu_yaw();
    const bool yaw_valid = std::isfinite(yaw);

    // 读取同一时刻的电机反馈。
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    const uint32_t now = osKernelSysTick();

    const bool motors_alive = motor6020_A.is_alive(now) && motor6020_B.is_alive(now);

    const float raw_a = motor6020_A.angle;
    const float raw_b = motor6020_B.angle;

    __set_PRIMASK(primask);

    const float a = DIR_A * (raw_a - R_ZERO_A);
    const float b = DIR_B * (raw_b - R_ZERO_B);

    // ================= 累计yaw =================

    if (!yaw_valid) {
      yaw_ready = false;
    }
    else if (!yaw_ready) {
      last_yaw = yaw;
      total_yaw = yaw;
      yaw_ready = true;

      board_sample_yaw = total_yaw;
      board_sample_time = now;
      last_board_move = now;
    }
    else {
      total_yaw += wrap(yaw - last_yaw);
      last_yaw = yaw;
    }

    // ================= 判断板是否在转 =================

    if (now - board_sample_time >= BOARD_SAMPLE_MS) {
      if (std::fabs(total_yaw - board_sample_yaw) > BOARD_MOVE_ANGLE) {
        last_board_move = now;
      }

      board_sample_yaw = total_yaw;
      board_sample_time = now;
    }

    // ================= 选择模式 =================

    Mode mode = Mode::DISABLED;

    if (remote.is_alive(osKernelSysTick()) && motors_alive) {
      if (right == sp::DBusSwitchMode::MID && yaw_valid) {
        mode = Mode::FOLLOW;
      }
      else if (right == sp::DBusSwitchMode::UP) {
        mode = Mode::RESET;
      }
    }

    // ================= 模式切换 =================

    if (mode != previous_mode) {
      clear_control();
      driver = Driver::NONE;

      if (mode == Mode::FOLLOW) {
        yaw_ref = total_yaw;

        a_ref = a;
        b_ref = b;

        target_a = a;
        target_b = b;

        ratio = ratio_for(get_remote_left_switch());

        last_board_move = now;
        board_sample_yaw = total_yaw;
        board_sample_time = now;
      }
      else if (mode == Mode::RESET) {
        target_a = a + wrap(-a);
        target_b = b + wrap(-b);
      }

      hand_detector.reset(a, b, target_a, target_b, now);

      previous_mode = mode;
    }

    float torque_a = 0.0f;
    float torque_b = 0.0f;

    // ================= 中档联动 =================

    if (mode == Mode::FOLLOW) {
      const float board_delta = total_yaw - yaw_ref;

      target_a = a_ref + board_delta;
      target_b = b_ref + ratio * board_delta;

      const float new_ratio = ratio_for(get_remote_left_switch());

      if (new_ratio != ratio) {
        // 比例切换时保持目标连续。
        a_ref = target_a;
        b_ref = target_b;
        yaw_ref = total_yaw;

        ratio = new_ratio;
        driver = Driver::NONE;

        clear_control();

        hand_detector.reset(a, b, target_a, target_b, now);
      }

      const bool board_still = now - last_board_move >= BOARD_SETTLE_MS;

      if (!board_still) {
        // 转板时优先执行姿态联动。
        if (driver != Driver::NONE) {
          clear_control();
        }

        driver = Driver::NONE;

        hand_detector.reset(a, b, target_a, target_b, now);
      }
      else if (driver == Driver::NONE) {
        driver = hand_detector.detect(a, b, target_a, target_b, ratio, now);

        if (driver != Driver::NONE) {
          last_hand_move = now;

          last_hand_position = driver == Driver::A ? a : b;

          clear_control();
        }
      }

      // ================= 接受手动输入 =================

      if (driver != Driver::NONE) {
        const float hand_position = driver == Driver::A ? a : b;

        // 不再依赖电机反馈的speed。
        if (std::fabs(hand_position - last_hand_position) >= HAND_KEEP_MOVE) {
          last_hand_position = hand_position;
          last_hand_move = now;
        }

        // A被拨动：B跟随比例×A位移。
        // B被拨动：A跟随B位移/比例。
        const float shift_a = driver == Driver::A ? a - target_a : (b - target_b) / ratio;

        target_a += shift_a;
        target_b += ratio * shift_a;

        // 保存新的参考位置，不修改IMU实际yaw。
        a_ref = target_a;
        b_ref = target_b;
        yaw_ref = total_yaw;

        if (now - last_hand_move >= HAND_RELEASE_MS) {
          driver = Driver::NONE;

          clear_control();

          hand_detector.reset(a, b, target_a, target_b, now);
        }
      }

      // ================= 电机控制 =================

      // 手拨A时释放A，只控制B。
      if (driver == Driver::A) {
        pid_a.clear();
        torque_a = 0.0f;
      }
      else {
        torque_a = control(pid_a, friction_a, target_a, a);
      }

      // 手拨B时释放B，只控制A。
      if (driver == Driver::B) {
        pid_b.clear();
        torque_b = 0.0f;
      }
      else {
        torque_b = control(pid_b, friction_b, target_b, b);
      }
    }

    // ================= 上档归零 =================

    else if (mode == Mode::RESET) {
      torque_a = control(pid_a, friction_a, target_a, a);

      torque_b = control(pid_b, friction_b, target_b, b);
    }

    // ================= 下档或掉线 =================

    else {
      clear_control();

      driver = Driver::NONE;

      target_a = a;
      target_b = b;

      hand_detector.reset(a, b, target_a, target_b, now);
    }

    // ================= 调试输出 =================

    target_angle_A = R_ZERO_A + DIR_A * target_a;
    target_angle_B = R_ZERO_B + DIR_B * target_b;

    test_mode = static_cast<float>(static_cast<int>(mode));

    test_driver = static_cast<float>(static_cast<int>(driver));

    test_torque_A = DIR_A * torque_a;

    send_commands(torque_a, torque_b);

    osDelayUntil(&wake_time, 1);
  }
}

// ================= CAN接收回调 =================

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  if (hcan != &hcan1) {
    return;
  }

  const uint32_t stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    can1.recv();

    if (can1.rx_id == motor6020_A.rx_id) {
      motor6020_A.read(can1.rx_data, stamp_ms);
    }
    else if (can1.rx_id == motor6020_B.rx_id) {
      motor6020_B.read(can1.rx_data, stamp_ms);
    }
  }
}