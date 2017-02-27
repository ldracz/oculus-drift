#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

/*! \mainpage Oculus Drift Package
 *
 * @author Lukasz Dracz
 * @author Simone Mazzola
 * @author Suraj Pattar
 *
 * \section intro_sec Introduction
 *
 * The Oculus Drift provides three control schemes for controlling
 * a firefly asctec drone using an oculus rift and keyboard.
 * The Oculus Drift consists of four nodes that can be run.
 *
 * Oculus-drift republishes images from the firefly stereo camera to the oculus
 *
 * Oculus-control provides the first control scheme using the Oculus orientation info
 *
 * With this node it is possible to move into any position using the proper sequence of
 * commands.
 *
 * oculus-control-alternative provides a similar control scheme as oculus-control but limits
 *
 * the oculus to the xy plane it is currently located and changes tilting in the z axis to movement
 *
 * in the left or right of the robot.
 *
 * oculus-control-keyboard allows the control of the firefly drone through keyboard commands in the
 *
 * terminal (that needs to be active).
 *
 */

/** @brief An Image Transport Publisher to the left oculus eye
*
*/
image_transport::Publisher pub_left;
/** @brief An Image Transport Publisher to the right oculus eye
*
*/
image_transport::Publisher pub_right;

/** @brief Callback Function for left stereo camera
*
*   Publishes the image received from the left stereo camera to the left oculus image
*   @param msg the raw image for left stereo camera
*/
void imageLeftCallback(const sensor_msgs::ImageConstPtr& msg)
{
  pub_left.publish(msg);
}

/** @brief Callback Function for right stereo camera
*
*   Publishes the image received from the right stereo camera to the right oculus image
*   @param msg the raw image for right stereo camera
*/
void imageRightCallback(const sensor_msgs::ImageConstPtr& msg)
{
  pub_right.publish(msg);
}

/** @brief Main function that setups up the subscribers and publishers
*
*   It sets up the subscribers to the left and right stereo camera's of the firefly drone.
*   The publishers set up are published and used by oculus-stereo node
*
*/

int main(int argc, char** argv)
{
  ros::init(argc, argv, "oculus_drift");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  pub_left = it.advertise("left/image_raw", 1);
  pub_right = it.advertise("right/image_raw", 1);
  image_transport::Subscriber sub_left = it.subscribe("/firefly/vi_sensor/left/image_raw", 1, imageLeftCallback);
  image_transport::Subscriber sub_right = it.subscribe("/firefly/vi_sensor/right/image_raw", 1, imageRightCallback);
  cv::waitKey(30);

  ros::Rate loop_rate(5);
  while (nh.ok()) {

    ros::spinOnce();
    loop_rate.sleep();
  }
}
