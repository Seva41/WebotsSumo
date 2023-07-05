#include <webots/camera.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <stdio.h>
#include <math.h>

#define TIME_STEP 32
#define MAX_SPEED 10
#define BLUE_OBJECT_COLOR_R 0
#define BLUE_OBJECT_COLOR_G 0
#define BLUE_OBJECT_COLOR_B 1

int main(int argc, char **argv) {
  // Initialize Webots
  wb_robot_init();

  // Get the camera device
  WbDeviceTag camera = wb_robot_get_device("Camara");
  wb_camera_enable(camera, TIME_STEP);

  // Get the motor devices
  WbDeviceTag left_motor = wb_robot_get_device("MotorIzq");
  WbDeviceTag right_motor = wb_robot_get_device("MotorDer");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  int blueObjectDetected = 0;

  while (wb_robot_step(TIME_STEP) != -1) {
    // Get the camera image
    const unsigned char* image = wb_camera_get_image(camera);
    int width = wb_camera_get_width(camera);
    int height = wb_camera_get_height(camera);

    // Iterate over the image pixels
    int blueObjectIndex = -1;
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        // Get the RGB values of the pixel
        int red = wb_camera_image_get_red(image, width, x, y);
        int green = wb_camera_image_get_green(image, width, x, y);
        int blue = wb_camera_image_get_blue(image, width, x, y);
        
        // Check if the pixel color matches the blue object color
        if (blue == BLUE_OBJECT_COLOR_B) {
          blueObjectIndex = y * width + x;
          printf("BOI: %i\n", blueObjectIndex);
          break;
        }
      }
    }
    
    if (blueObjectIndex != -1) {
  // Get the position and size of the blue object
  const WbCameraRecognitionObject *blueObject = wb_camera_recognition_get_objects(camera);
  int positionOnImageX = blueObject[blueObjectIndex].position_on_image[0];
  int positionOnImageY = blueObject[blueObjectIndex].position_on_image[1];
  int sizeOnImageX = blueObject[blueObjectIndex].size_on_image[0];
  int sizeOnImageY = blueObject[blueObjectIndex].size_on_image[1];

  // Calculate the center position of the blue object
  int centerX = positionOnImageX + (sizeOnImageX / 2);
  int centerY = positionOnImageY + (sizeOnImageY / 2);

  // Calculate the difference between the center of the image and the center of the blue object
  int diffX = centerX - (width / 2);
  int diffY = centerY - (height / 2);

  // Check if the blue object has the correct number of colors

    // Get the RGB values of the blue object
    double blue = blueObject[blueObjectIndex].colors[2];
    
    // Check if the blue object is indeed blue (with value 0 0 1)
    if (blue == 1.0) {
      // Calculate the scaling factor based on the camera's field of view
      float scalingFactor = (float)width / wb_camera_get_fov(camera);

      // Calculate the desired direction and distance to move
      float desiredDirection = atan2(diffX * scalingFactor, diffY * scalingFactor);

      // Set the velocity of the motors based on the desired direction
      float velocity = 8.0;
      float leftMotorVelocity = velocity * (2.0 - desiredDirection);
      float rightMotorVelocity = velocity * (2.0 + desiredDirection);

      // Calculate the desired distance to the object
      float desiredDistance = sqrt(diffX * diffX + diffY * diffY) * scalingFactor;

      // Calculate the motor speeds based on the desired distance
      float leftSpeed = MAX_SPEED * (1 - (diffX / desiredDistance));
      float rightSpeed = MAX_SPEED * (1 + (diffX / desiredDistance));

      printf("speeds L,R: %f, %f\n", leftSpeed, rightSpeed);

      // Set the velocities of the left and right motors
      wb_motor_set_velocity(left_motor, -leftSpeed);
      wb_motor_set_velocity(right_motor, -rightSpeed);

      // Set the flag to indicate that a blue object has been detected
      blueObjectDetected = 1;
    }
  


    blueObjectDetected = 0; // Reset the flag

    // If no blue objects are detected, spin in place
    wb_motor_set_velocity(left_motor, MAX_SPEED/2);
    wb_motor_set_velocity(right_motor, -MAX_SPEED/2);
  }
}

    
  

  wb_robot_cleanup();
  return 0;
}
