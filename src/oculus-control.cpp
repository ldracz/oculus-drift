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

/** @brief Callback Function for Oculus Orientation Information
*
*   The orientation information is translated into a desired trajectory that is published to the drone.
*
*   Tilting in the y-axis when past a threshold will turn the drone left or right.
*   Tilting in the x-axis when past a threshold will move the drone forward based
*   on the current orientation of the drone.
*   Tilting in the z-axis when past a threshold will move the drone up and down.
*
*   @param msg The orientation quaternion that provides the x,y,z,w orientation information of the oculus
*/

void orientationCallback(const geometry_msgs::Quaternion::ConstPtr msg)
{
  ROS_INFO("Callback Success");

  if (msg->y < -0.2) {
    desired_yaw = desired_yaw - speed;
  } else if (msg->y > 0.2) {
    desired_yaw = desired_yaw + speed;
  }

  if (msg->x < -0.2) {
    desired_position.x() = desired_position.x() + speed*cos(desired_yaw);
    desired_position.y() = desired_position.y() + speed*sin(desired_yaw);
  } else if (msg->x > 0.2) {
    desired_position.x() = desired_position.x() - speed*cos(desired_yaw);
    desired_position.y() = desired_position.y() - speed*sin(desired_yaw);
  }

  if (msg->z < -0.25) {
    desired_position.z() = desired_position.z() + speed;
  } else if (msg->z > 0.25) {
    desired_position.z() = desired_position.z() - speed;
  }

  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
      desired_yaw, &traj_msg);
  pub_traj.publish(traj_msg);

}

/** @brief Main function
*
*   That setups up the subscription to the oculus's orientation
*   and the publisher to the firefly drone.
*/


int main(int argc, char** argv)
{
  ros::init(argc, argv, "oculus_control");
  ros::NodeHandle nh;

  traj_msg.header.stamp = ros::Time::now();

  ros::Subscriber sub_orient = nh.subscribe("/oculus/orientation", 1, orientationCallback);

  pub_traj = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/firefly/command/trajectory", 1);

  ros::Rate loop_rate(5);
  while (nh.ok()) {

    ros::spinOnce();
    loop_rate.sleep();
  }
}
