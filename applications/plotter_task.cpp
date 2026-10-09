#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "io/plotter/plotter.hpp"

// 引用imu_task.cpp中已经创建的IMU对象。
// 不要在这里重新创建一个BMI088对象。
extern sp::BMI088 bmi088;

// 通过UART1发送到上位机。
sp::Plotter plotter(&huart1);

extern "C" void plotter_task()
{
  while (true) {
    // 三轴角速度，单位：rad/s。
    plotter.plot(
      bmi088.gyro[0],  // Channel 1：X轴
      bmi088.gyro[1],  // Channel 2：Y轴
      bmi088.gyro[2]   // Channel 3：Z轴
    );

    osDelay(10);
  }
}