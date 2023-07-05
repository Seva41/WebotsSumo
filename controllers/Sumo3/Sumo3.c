#include <webots/camera.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <stdio.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

#define TIME_STEP 32
#define WB_CHANNEL_BROADCAST -1
#define MAX_SPEED 10

#define BLUE_OBJECT_ID 483  // Assuming the blue object has an ID of 472
#define BLUE_COLOR_THRESHOLD 0.1

// Function to calculate the angle to the object based on its position on the image
double calculateObjectAngle(int positionOnImageX, int imageWidth, double fieldOfView) {
  double normalizedX = (double)positionOnImageX / (double)imageWidth;
  return (normalizedX - 0.5) * fieldOfView;
}

// Function to check if a given color is blue
int isBlueColor(double red, double green, double blue) {
  // Normalize the color values to the range [0, 1]
  double normalizedRed = red / 255.0;
  double normalizedGreen = green / 255.0;
  double normalizedBlue = blue / 255.0;
  
  // Check if the color is blue based on the color threshold
  if (normalizedRed < BLUE_COLOR_THRESHOLD && normalizedGreen < BLUE_COLOR_THRESHOLD && normalizedBlue > 1.0 - BLUE_COLOR_THRESHOLD) {
    return 1;  // The color is blue
  }
  
  return 0;  // The color is not blue
}

int main(int argc, char **argv) {
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
  
  // Flag variable to indicate if a blue object has been detected
  int blueObjectDetected = 0;

  while (wb_robot_step(TIME_STEP) != -1) {
      // Get the camera image
      const unsigned char* image = wb_camera_get_image(camera);
      
      // Get camera recognition data
      int num_objects = wb_camera_get_number_of_recognition_objects(camera);
      
      int blueObjectIndex = -1;
      
      // Iterate over each recognition object
      for (int i = 0; i < num_objects; i++) {
        const WbCameraRecognitionObject *object = wb_camera_get_recognition_object(camera, i);
        
        // Check if the object is blue based on its colors
        if (isBlueColor(object->colors[0], object->colors[1], object->colors[2])) {
          blueObjectIndex = i;
          break;
        }
      }
      
      if (blueObjectIndex != -1) {
        // Get the position and size of the blue object
        const WbCameraRecognitionObject *blueObject = wb_camera_get_recognition_object(camera, blueObjectIndex);
        
        // Calculate the position of the blue object
        int positionOnImageX = blueObject->position_on_image[0];
        int positionOnImageY = blueObject->position_on_image[1];
        int sizeOnImageX = blueObject->size_on_image[0];
        int sizeOnImageY = blueObject->size_on_image[1];
        
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
        
        // Set the velocities of the left and right motors
        wb_motor_set_velocity(M_left, -leftSpeed);
        wb_motor_set_velocity(M_right, -rightSpeed);
        
        // Set the flag to indicate that a blue object has been detected
        blueObjectDetected = 1;
        } else {
        if (blueObjectDetected) {
          // Stop the robot if it has detected a blue object but no longer detects it
          wb_motor_set_velocity(M_left, 0.0);
          wb_motor_set_velocity(M_right, 0.0);
          blueObjectDetected = 0; // Reset the flag
        } else {
          // If no blue objects are detected, spin in place
          wb_motor_set_velocity(M_left, MAX_SPEED);
          wb_motor_set_velocity(M_right, -MAX_SPEED);
        }
      }
  }


  wb_robot_cleanup();
  return 0;
}
