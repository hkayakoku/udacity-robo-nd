#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
  ball_chaser::DriveToTarget srv;
  srv.request.linear_x  = lin_x;
  srv.request.angular_z = ang_z;
  
  if (!client.call(srv))
    ROS_ERROR("Failed to call service safe_move");

}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
  int white_pixel = 255;

  // std_msgs/Header header
  //   uint32 seq
  //   time stamp
  //   string frame_id
  // uint32 height
  // uint32 width
  // string encoding
  // uint8 is_bigendian
  // uint32 step
  // uint8[] data

  int leftCount = 0;
  int rightCount = 0;
  int forwardCount = 0;

  // No need to process whole image bacause ball will be in floor.
  // So i skip image's top area. Only 20 pixel slice will be process

  int top  = img.height / 2 - 3;
  int bottom = img.height / 2 + 3;

  for(int i = 3*top*img.width; i < 3*bottom*img.width; i++)
    {
      // search from rows
      // Get three consequtive array elem (RGB Channels)
      if( img.data[i] == white_pixel && img.data[i+1] == white_pixel && img.data[i+2] == white_pixel )
        {
          int column = (i / 3) % img.width;

          // Determine area with respect to following formula:
          // -----------x-------------------3x---------4x
          // |          |                    |          |
          // |          |                    |          |
          // |  LEFT    |     FORWARD        |  RIGHT   |
          // |          |                    |          |
          // |          |                    |          |
          // |          |                    |          |


          if(column >= (img.width/3) * 2)
            rightCount++;
          else if(column > (img.width/3) && column < (img.width/3) * 2)
            forwardCount++;
          else
            leftCount++;
        }
      i = i+2;
    }

  ROS_INFO("left: %d right: %d forward: %d", leftCount, rightCount, forwardCount);  

  int processedPixelSize = (bottom-top) * img.width;

  // This a threshold value for sending stop request. Means when white pixel count
  // so much, robot is close enough to ball.
 
  int maxForwardWhitePixel = (processedPixelSize / 2 ) / 2;

  // There is an inverse proportion between white pixel count and forward velocity
  // if forward area  has only 1 pixel white, means robot will go forward with 
  // maximum speed which is 1.0. if forward area has maximum pixel (which is denoted
  // by maxForwardWhitePixel variable) means robot will stop in forward direction
  // this is a linear equation.

  // Formula: (max -y) / max = x
  float forwardVel = 0.0;
  float angularVel = 0.0;
   
  if(forwardCount > 0 && forwardCount < maxForwardWhitePixel)
    forwardVel =(float) (maxForwardWhitePixel - forwardCount) / (float)(maxForwardWhitePixel * 2);
   
  if((forwardCount == 0) && (rightCount != 0 || leftCount != 0))
    {
      if(rightCount > leftCount)
        angularVel = -0.1;
      else
        angularVel = 0.1;
    }

  drive_robot(forwardVel, angularVel);

  // ros::Duration(3).sleep();

}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "process_image");
  ros::NodeHandle n;

  // Define a client service cabaple of requesting services from command_robot
  client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

  // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
  ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

  // Handle ROS communication events
  ros::spin();

  return 0;    
}
