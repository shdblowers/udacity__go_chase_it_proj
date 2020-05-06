#include <sensor_msgs/Image.h>
#include "ball_chaser/DriveToTarget.h"
#include "ros/ros.h"

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
  ball_chaser::DriveToTarget srv;
  srv.request.linear_x = lin_x;
  srv.request.angular_z = ang_z;

  if (!client.call(srv))
    ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
  int white_pixel = 255;

  bool found_white_pixel = false;
  int found_pixel_col = 0;

  for (int i = 0; i < img.height * img.step; i++)
  {
    if (img.data[i] == white_pixel)
    {
      found_white_pixel = true;

      // find position in that row of i
      found_pixel_col = i % img.step;

      // already found white pixel, so don't bother looking at rest of image
      break;
    }
  }

  if (found_white_pixel)
  {
    if (found_pixel_col < (img.step / 3))
    {
      // in left third of image, so turn left
      drive_robot(0.0, 0.5);

      return;
    }
    else if (found_pixel_col >= ((img.step / 3) * 2))
    {
      // in right third, so turn right
      drive_robot(0.0, -0.5);

      return;
    }
    else
    {
      // in centre third, so drive forward
      drive_robot(0.5, 0.0);

      return;
    }
  }
  else
  {
    drive_robot(0.0, 0.0);

    return;
  }
}

int main(int argc, char **argv)
{
  // Initialize the process_image node and create a handle to it
  ros::init(argc, argv, "process_image");
  ros::NodeHandle n;

  // Define a client service capable of requesting services from command_robot
  client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

  // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
  ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

  // Handle ROS communication events
  ros::spin();

  return 0;
}