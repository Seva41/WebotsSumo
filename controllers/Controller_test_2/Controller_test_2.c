#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>

#define TIME_STEP 64
#define MAX_SPEED 10

int main(int argc, char **argv) {
  wb_robot_init();

  WbDeviceTag motor_d = wb_robot_get_device("MotorD");
  WbDeviceTag motor_i = wb_robot_get_device("MotorI");
  WbDeviceTag motor_d2 = wb_robot_get_device("MotorD2");
  WbDeviceTag motor_i2 = wb_robot_get_device("MotorI2");

  WbDeviceTag pos_sensor_rd = wb_robot_get_device("PosSensorRD");
  WbDeviceTag pos_sensor_ri = wb_robot_get_device("PosSensorRI");

  wb_position_sensor_enable(pos_sensor_rd, TIME_STEP);
  wb_position_sensor_enable(pos_sensor_ri, TIME_STEP);

  double target_position = 0.0;

  while (wb_robot_step(TIME_STEP) != -1) {
    double current_position_rd = wb_position_sensor_get_value(pos_sensor_rd);
    double current_position_ri = wb_position_sensor_get_value(pos_sensor_ri);

    if (current_position_rd > 1.0) {
      target_position = -1.0;
    } else if (current_position_rd < -1.0) {
      target_position = 1.0;
    }

    wb_motor_set_position(motor_d, target_position);
    wb_motor_set_position(motor_i, target_position);
    wb_motor_set_position(motor_d2, target_position);
    wb_motor_set_position(motor_i2, target_position);

    wb_motor_set_velocity(motor_d, MAX_SPEED);
    wb_motor_set_velocity(motor_i, MAX_SPEED);
    wb_motor_set_velocity(motor_d2, MAX_SPEED);
    wb_motor_set_velocity(motor_i2, MAX_SPEED);
  }

  wb_robot_cleanup();

  return 0;
}
