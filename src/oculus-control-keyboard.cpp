#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include <thread>
#include <chrono>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <cmath>


/** @brief Publisher for the desired trajectory of the drone
*/
ros::Publisher pub_traj;
/** @brief The message variable that is published
*/
trajectory_msgs::MultiDOFJointTrajectory traj_msg;
/** @brief The speed at which the drone can move at
*/
float speed = 0.1;
/** @brief The position the drone is expected to go to
*/
Eigen::Vector3d desired_position(0.0, 0.0, 1.0);
/** @brief The value 90 degrees holds in radians (rounded)
*/
float ninetyDegInRad = 1.5708;

/** @brief The yaw rotation expected of the drone to turn to (in radians)
*/
double desired_yaw = 0.0;



/**  @brief The Main Function that runs the keyboard loop
*
*    The terminal is cleared. When the terminal is active, it reads
*    the key being pressed by the user. Based on what key is pressed
*    the change needed in the desired trajectory will be calculated.
*    This desired trajectory will then be published to the firefly drone.
*
*/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "oculus_keycontrol");
  ros::NodeHandle nh;
  traj_msg.header.stamp = ros::Time::now();

  pub_traj = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/firefly/command/trajectory", 1);

  int ch, oldch;
  char cheese;
  int kfd = 0;
  struct termios cooked, raw;
  float speed = 0.05;
  int active = 0;


  while(1) {

    /// get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    /// Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("----Press X to activate/deactivate control-----------------");
    puts("Use W,A,S,D to move the drone. Use Q,E to adjust height of the drone. R,F to rotate.");

    if(read(kfd, &cheese, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }


    switch (cheese) {
      case 'x':
      case 'X':
        if(active == 0) {
          active = 1;
        } else {
          active = 0;
        }
      case 'a':
      case 'A':
        ROS_INFO("A");
        desired_position.x() = desired_position.x() - speed*cos(desired_yaw-ninetyDegInRad);
        desired_position.y() = desired_position.y() - speed*sin(desired_yaw-ninetyDegInRad);
        goto PUB;
        break;
      case 's':
      case 'S':
        ROS_INFO("S");
        desired_position.x() = desired_position.x() - speed*cos(desired_yaw);
        desired_position.y() = desired_position.y() - speed*sin(desired_yaw);
        goto PUB;
        break;
      case 'd':
      case 'D':
        ROS_INFO("D");
        desired_position.x() = desired_position.x() + speed*cos(desired_yaw-ninetyDegInRad);
        desired_position.y() = desired_position.y() + speed*sin(desired_yaw-ninetyDegInRad);
        goto PUB;
        break;
      case 'w':
      case 'W':
        ROS_INFO("W");
        desired_position.x() = desired_position.x() + speed*cos(desired_yaw);
        desired_position.y() = desired_position.y() + speed*sin(desired_yaw);
        goto PUB;
        break;
      case 'q':
      case 'Q':
        ROS_INFO("Q");
        desired_position.z() = desired_position.z() + speed;
        goto PUB;
        break;
      case 'e':
      case 'E':
        ROS_INFO("E");
        desired_position.z() = desired_position.z() - speed;
        goto PUB;
        break;
      case 'r':
      case 'R':
        ROS_INFO("R");
        desired_yaw = desired_yaw + speed;
        goto PUB;
        break;
      case 'f':
      case 'F':
        ROS_INFO("F");
        desired_yaw = desired_yaw - speed;
        goto PUB;
        break;

      PUB:
        if (active) {
          ROS_INFO("publishing");
              mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
                  desired_yaw, &traj_msg);
          pub_traj.publish(traj_msg);
        }
      }
    }
}
