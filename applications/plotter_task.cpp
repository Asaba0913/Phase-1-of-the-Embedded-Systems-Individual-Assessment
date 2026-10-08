#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"

extern sp::RM_Motor motor6020_A;
extern float target_angle_A;
extern float test_mode;
extern float test_torque_A;

sp::Plotter plotter(&huart1);

extern "C" void plotter_task()
{
  while (true) {
    plotter.plot(test_mode, target_angle_A, motor6020_A.angle, test_torque_A);

    osDelay(10);
  }
}