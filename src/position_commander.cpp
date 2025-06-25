#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "position_commander");
  ros::NodeHandle nh;

  ros::Publisher pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/pose_ref_uav1", 10);

  ros::Rate rate(10.0);  // 10 Hz

  while (ros::ok())
  {
    mavros_msgs::PositionTarget msg;

    msg.header.stamp = ros::Time::now();
    msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

    // Máscara: usar posición + yaw
    msg.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                    mavros_msgs::PositionTarget::IGNORE_VY |
                    mavros_msgs::PositionTarget::IGNORE_VZ |
                    mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY |
                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::PositionTarget::FORCE |
                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    // Posición deseada
    msg.position.x = 3.0;
    msg.position.y = 1.0;
    msg.position.z = 2.7;

    // Yaw en radianes
    msg.yaw = 0.7;

    pos_pub.publish(msg);
    rate.sleep();
  }

  return 0;
}