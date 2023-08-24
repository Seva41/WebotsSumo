#include <webots/robot.h>
#include <webots/supervisor.h>
#include <stdio.h>

#define TIME_STEP 32

int main(int argc, char **argv) {
  wb_robot_init();
  WbNodeRef robot1 = wb_supervisor_node_get_from_def("EMISOR");
  const WbFieldRef pos = wb_supervisor_node_get_field(robot1, "translation");
  
  while (wb_robot_step(TIME_STEP) != -1) {
    const double *posEmi = wb_supervisor_field_get_sf_vec3f(pos);
    double Xe = posEmi[0];
    double Ye = posEmi[1];
    double Ze = posEmi[2];
    //printf("SUP dice: PosXYZ EMI=> %.3f %.3f %.3f\n", Xe, Ye, Ze);
  };

  wb_robot_cleanup();
  return 0;
}
