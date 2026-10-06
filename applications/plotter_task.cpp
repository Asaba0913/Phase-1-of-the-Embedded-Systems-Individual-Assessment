#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "io/plotter/plotter.hpp"

extern sp::BMI088 bmi088;

sp::Plotter plotter(&huart1);

extern "C" void plotter_task()
{
  while (1) {
    plotter.plot(bmi088.gyro[0], bmi088.gyro[1], bmi088.gyro[2]);

    osDelay(1);
  }
}