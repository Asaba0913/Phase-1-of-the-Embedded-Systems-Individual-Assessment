#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"

// 暂时不用DMA，改用串口中断接收，排查DMA相关问题。
sp::DBus remote(&huart3, false);

// 检查任务有没有执行。
volatile uint32_t remote_task_started = 0;

// 最近一次USART3接收回调收到的字节数。
volatile uint32_t remote_rx_size = 0;

// 可在调试器观察：接收回调次数、串口错误次数。
volatile uint32_t remote_rx_count = 0;
volatile uint32_t remote_error_count = 0;
volatile uint32_t remote_last_error = 0;

extern "C" sp::DBusSwitchMode get_remote_right_switch() { return remote.sw_r; }

extern "C" sp::DBusSwitchMode get_remote_left_switch() { return remote.sw_l; }

extern "C" void uart_task()
{
  remote_task_started = 1;

  remote.request();

  while (true) {
    osDelay(10);
  }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  if (huart != &huart3) {
    return;
  }

  remote_rx_size = Size;
  remote_rx_count++;

  remote.update(Size, osKernelSysTick());
  remote.request();
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart != &huart3) {
    return;
  }

  remote_error_count++;
  remote_last_error = huart->ErrorCode;

  remote.request();
}