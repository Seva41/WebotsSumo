#include <webots/camera.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <stdio.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

#define TIME_STEP 32
#define WB_CHANNEL_BROADCAST -1
#define MAX_SPEED 10

#define BLUE_OBJECT_ID 640

// Function to calculate the angle to the object based on its position on the image
double calculateObjectAngle(int positionOnImageX, int imageWidth, double fieldOfView)
{
  double normalizedX = (double)positionOnImageX / (double)imageWidth;
  return (normalizedX - 0.5) * fieldOfView;
}

int main(int argc, char **argv)
{
  // Define and initialize devices
  wb_robot_init();
  WbDeviceTag M_left = wb_robot_get_device("MotorIzq");
  WbDeviceTag M_right = wb_robot_get_device("MotorDer");
  WbDeviceTag camera = wb_robot_get_device("Camara");

  wb_camera_enable(camera, TIME_STEP);
  wb_camera_recognition_enable(camera, TIME_STEP);

  int img_width = wb_camera_get_width(camera);
  int img_height = wb_camera_get_height(camera);

  // Set initial motor velocities
  wb_motor_set_velocity(M_left, 0.0);
  wb_motor_set_velocity(M_right, 0.0);

  // Enable position control for the motors
  wb_motor_set_position(M_left, INFINITY);
  wb_motor_set_position(M_right, INFINITY);

  // Field of view in radians
  double fieldOfView = 0.785;

  while (wb_robot_step(TIME_STEP) != -1)
  {
    // Get the camera image
    const unsigned char *image = wb_camera_get_image(camera);

    // Get camera recognition data
    int num_objects = wb_camera_recognition_get_number_of_objects(camera);

    if (num_objects > 0)
    {
      const WbCameraRecognitionObject *objects = wb_camera_recognition_get_objects(camera);

      /*// Print the IDs of the recognized objects
      for (int i = 0; i < num_objects; i++) {
        printf("Object %d: ID = %d\nChar: %s\n", i, objects[i].id, objects[i].model);
      }
      */
      // Search for the blue object
      int blueObjectIndex = -1;
      for (int i = 0; i < num_objects; i++)
      {
        if (objects[i].id == BLUE_OBJECT_ID || objects[i].id == 330)
        {
          blueObjectIndex = i;
          break;
        }
      }

      // printf("index: %i\n",blueObjectIndex);

      if (blueObjectIndex != -1)
      {
        // Get the ID of the blue object
        int blueObjectId = objects[blueObjectIndex].id;

        // Get the position and size of the blue object
        int positionOnImageX = objects[blueObjectIndex].position_on_image[0];
        int positionOnImageY = objects[blueObjectIndex].position_on_image[1];
        int sizeOnImageX = objects[blueObjectIndex].size_on_image[0];
        int sizeOnImageY = objects[blueObjectIndex].size_on_image[1];

        // Calculate the center position of the blue object
        int centerX = positionOnImageX + (sizeOnImageX / 2);
        int centerY = positionOnImageY + (sizeOnImageY / 2);

        // Calculate the difference between the center of the image and the center of the blue object
        int diffX = centerX - (img_width / 2);
        int diffY = centerY - (img_height / 2);

        // Calculate the scaling factor based on the camera's field of view
        float scalingFactor = (float)img_width / fieldOfView;

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

        if (leftSpeed > MAX_SPEED)
          leftSpeed = MAX_SPEED;
        if (rightSpeed > MAX_SPEED)
          rightSpeed = MAX_SPEED;
        if (leftSpeed < 0)
          leftSpeed = 0;
        if (rightSpeed < 0)
          rightSpeed = 0;

        // Set the velocities of the left and right motors
        wb_motor_set_velocity(M_left, -leftSpeed);
        wb_motor_set_velocity(M_right, -rightSpeed);
      }
      else
      {
        // If no blue objects are detected, spin
        wb_motor_set_velocity(M_left, -5);
        wb_motor_set_velocity(M_right, 5);
      }
    }
    else
    {
      // If no blue objects are detected, spin
      wb_motor_set_velocity(M_left, -5);
      wb_motor_set_velocity(M_right, 5);
    }
  }

  wb_robot_cleanup();
  return 0;
}
