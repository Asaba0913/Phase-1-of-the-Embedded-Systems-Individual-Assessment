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

// A：ID 1；B：ID 2。
// 电流控制模式，开启多圈角度反馈。
sp::RM_Motor motor6020_A(1, sp::RM_Motors::GM6020, 1.0f, true);

sp::RM_Motor motor6020_B(2, sp::RM_Motors::GM6020, 1.0f, true);

// 给Plotter使用：原始电机坐标系中的目标角度，单位rad。
float target_angle_A = 0.0f;
float target_angle_B = 0.0f;

// 新增诊断变量。
// 模式：0失能、1联动、2复位。
float test_mode = 0.0f;

// A电机最终力矩命令，单位N·m。
float test_torque_A = 0.0f;

namespace
{

constexpr float PI = 3.14159265358979323846f;
constexpr float TWO_PI = 2.0f * PI;
constexpr float DT = 0.001f;

// 保留你当前的参数，先诊断，不同时调整PID。
constexpr float KP_A = 0.8f;
constexpr float KI_A = 0.0f;
constexpr float KD_A = 0.02f;

constexpr float KP_B = 0.8f;
constexpr float KI_B = 0.0f;
constexpr float KD_B = 0.02f;

constexpr float MAX_TORQUE = 0.40f;
constexpr float MAX_INTEGRAL = 0.05f;

// 约0.5°的角度误差死区。
constexpr float ERROR_DEADBAND = 0.00873f;

// 电机正方向与C板yaw正方向一致时为1，相反时为-1。
constexpr float DIR_A = 1.0f;
constexpr float DIR_B = 1.0f;

// R标零位对应的原始反馈角度，暂按要求设置为0。
constexpr float R_ZERO_A = 0.0f;
constexpr float R_ZERO_B = 0.0f;

// 手动拨动识别阈值。
constexpr float BOARD_STILL_SPEED = 0.035f;
constexpr uint32_t BOARD_SETTLE_MS = 300;

constexpr float HAND_SPEED = 0.10f;
constexpr float HAND_ERROR = 0.035f;
constexpr uint32_t HAND_RELEASE_MS = 200;

float clamp(float value, float limit)
{
  if (value > limit) {
    return limit;
  }

  if (value < -limit) {
    return -limit;
  }

  return value;
}

// 处理yaw跨越±π，以及选择最近的复位方向。
float wrap(float angle)
{
  while (angle > PI) {
    angle -= TWO_PI;
  }

  while (angle < -PI) {
    angle += TWO_PI;
  }

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

// 最大补偿0.02 N·m。
// 有效误差达到约3°时，达到最大补偿。
// 这些数值仍需根据实际负载调整。
FrictionCompensation friction_a{0.02f, 0.05236f};
FrictionCompensation friction_b{0.02f, 0.05236f};

// ================= 角度PID =================

// angular=false：保留多圈误差。
// alpha=0.1：对D项滤波，没有另外套速度PID。
sp::PID pid_a(DT, KP_A, KI_A, KD_A, MAX_TORQUE, MAX_INTEGRAL, 0.1f, false);

sp::PID pid_b(DT, KP_B, KI_B, KD_B, MAX_TORQUE, MAX_INTEGRAL, 0.1f, false);

float control(sp::PID & pid, const FrictionCompensation & friction, float target, float actual)
{
  const float error = deadband(target - actual);

  // 保留真实反馈，D项仍根据实际角度变化提供阻尼。
  pid.calc(actual + error, actual);

  return clamp(pid.out + friction.calculate(error), MAX_TORQUE);
}

float ratio_for(sp::DBusSwitchMode left_switch)
{
  switch (left_switch) {
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

void clear_control()
{
  pid_a.clear();
  pid_b.clear();
}

void send_commands(float torque_a, float torque_b)
{
  motor6020_A.cmd(DIR_A * torque_a);
  motor6020_B.cmd(DIR_B * torque_b);

  // 未使用的电机通道也清零。
  std::memset(can1.tx_data, 0, 8);

  // 两台电机写入同一控制帧。
  motor6020_A.write(can1.tx_data);
  motor6020_B.write(can1.tx_data);

  can1.send(motor6020_A.tx_id);
}

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

}  // namespace

extern "C" void can_task()
{
  can1.config();
  can1.start();

  Mode previous_mode = Mode::DISABLED;
  Driver driver = Driver::NONE;

  bool yaw_ready = false;

  float last_yaw = 0.0f;
  float total_yaw = 0.0f;

  float yaw_ref = 0.0f;
  float a_ref = 0.0f;
  float b_ref = 0.0f;

  float target_a = 0.0f;
  float target_b = 0.0f;

  float ratio = 1.0f;

  uint32_t last_board_move = 0;
  uint32_t last_hand_move = 0;
  uint32_t wake_time = osKernelSysTick();

  while (true) {
    const auto right = get_remote_right_switch();
    const float yaw = get_imu_yaw();
    const bool yaw_valid = std::isfinite(yaw);

    float yaw_step = 0.0f;

    // 同时取得两台电机的反馈快照。
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    const uint32_t now = osKernelSysTick();

    const bool motors_alive = motor6020_A.is_alive(now) && motor6020_B.is_alive(now);

    const float raw_a = motor6020_A.angle;
    const float raw_b = motor6020_B.angle;

    const float speed_a = DIR_A * motor6020_A.speed;
    const float speed_b = DIR_B * motor6020_B.speed;

    __set_PRIMASK(primask);

    // 将yaw展开为累计角度。
    if (!yaw_valid) {
      yaw_ready = false;
    }
    else if (!yaw_ready) {
      last_yaw = yaw;
      total_yaw = yaw;

      yaw_ready = true;
      last_board_move = now;
    }
    else {
      yaw_step = wrap(yaw - last_yaw);
      total_yaw += yaw_step;
      last_yaw = yaw;
    }

    if (std::fabs(yaw_step) / DT > BOARD_STILL_SPEED) {
      last_board_move = now;
    }

    // 转换到统一方向，并扣除R零位。
    const float a = DIR_A * (raw_a - R_ZERO_A);
    const float b = DIR_B * (raw_b - R_ZERO_B);

    // 默认失能。
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
        // 进入联动时，以当前位置作为起点。
        yaw_ref = total_yaw;

        a_ref = a;
        b_ref = b;

        target_a = a;
        target_b = b;

        ratio = ratio_for(get_remote_left_switch());
        last_board_move = now;
      }
      else if (mode == Mode::RESET) {
        // 回到最近的等效0°位置。
        target_a = a + wrap(-a);
        target_b = b + wrap(-b);
      }

      previous_mode = mode;
    }

    float torque_a = 0.0f;
    float torque_b = 0.0f;

    // ================= 中档：联动 =================

    if (mode == Mode::FOLLOW) {
      const float board_delta = total_yaw - yaw_ref;

      target_a = a_ref + board_delta;
      target_b = b_ref + ratio * board_delta;

      const float new_ratio = ratio_for(get_remote_left_switch());

      if (new_ratio != ratio) {
        // 切换比例时保持目标连续。
        a_ref = target_a;
        b_ref = target_b;
        yaw_ref = total_yaw;

        ratio = new_ratio;

        clear_control();
      }

      const bool board_still = now - last_board_move >= BOARD_SETTLE_MS;

      // 同时转板和手拨时，优先执行板联动。
      if (!board_still) {
        driver = Driver::NONE;
      }

      // 板停稳后，识别手动拨动。
      if (board_still && driver == Driver::NONE) {
        const float err_a = a - target_a;
        const float err_b = b - target_b;

        const bool hand_a = std::fabs(err_a) > HAND_ERROR && std::fabs(speed_a) > HAND_SPEED &&
                            err_a * speed_a > 0.0f;

        const bool hand_b = std::fabs(err_b) > HAND_ERROR && std::fabs(speed_b) > HAND_SPEED &&
                            err_b * speed_b > 0.0f;

        if (hand_a || hand_b) {
          driver = hand_a ? Driver::A : Driver::B;
          last_hand_move = now;

          clear_control();
        }
      }

      if (driver != Driver::NONE) {
        // 将手拨位移换算成A的等效位移。
        const float shift_a = driver == Driver::A ? a - target_a : (b - target_b) / ratio;

        target_a += shift_a;
        target_b += ratio * shift_a;

        // 更新参考点，不修改实际IMU yaw。
        a_ref = target_a;
        b_ref = target_b;
        yaw_ref = total_yaw;

        const float hand_speed = driver == Driver::A ? speed_a : speed_b;

        if (std::fabs(hand_speed) > HAND_SPEED) {
          last_hand_move = now;
        }

        if (now - last_hand_move >= HAND_RELEASE_MS) {
          driver = Driver::NONE;
          clear_control();
        }
      }

      // 被识别为手拨输入的电机暂时释放。
      if (driver != Driver::A) {
        torque_a = control(pid_a, friction_a, target_a, a);
      }
      else {
        pid_a.clear();
      }

      if (driver != Driver::B) {
        torque_b = control(pid_b, friction_b, target_b, b);
      }
      else {
        pid_b.clear();
      }
    }

    // ================= 上档：归零 =================

    else if (mode == Mode::RESET) {
      torque_a = control(pid_a, friction_a, target_a, a);

      torque_b = control(pid_b, friction_b, target_b, b);
    }

    // ================= 下档或掉线：失能 =================

    else {
      clear_control();

      target_a = a;
      target_b = b;
    }

    // 给绘图任务提供原始电机坐标系的目标。
    target_angle_A = R_ZERO_A + DIR_A * target_a;
    target_angle_B = R_ZERO_B + DIR_B * target_b;

    // 新增：记录当前模式和A电机最终力矩命令。
    test_mode = static_cast<float>(static_cast<int>(mode));
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