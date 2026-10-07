#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"

extern sp::BMI088 bmi088;
extern sp::RM_Motor motor6020_A;
extern sp::RM_Motor motor6020_B;

sp::Plotter plotter(&huart1);

extern "C" void plotter_task()
{
  while (1) {
    // plotter.plot(bmi088.gyro[0], bmi088.gyro[1], bmi088.gyro[2]);
    //plotter.plot(motor6020_A.angle, motor6020_B.angle);
    plotter.plot(1);

    osDelay(1);
  }
}