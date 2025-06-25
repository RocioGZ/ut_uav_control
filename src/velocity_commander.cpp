#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "velocity_commander");
  ros::NodeHandle nh;

  ros::Publisher vel_pub = nh.advertise<mavros_msgs::PositionTarget>("/pose_ref_uav1", 10);

  ros::Rate rate(10.0);  // 10 Hz

  while (ros::ok())
  {
    mavros_msgs::PositionTarget msg;

    msg.header.stamp = ros::Time::now();
    msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

    // Máscara: ignorar posición y aceleración, usar velocidad y yaw
    msg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                    mavros_msgs::PositionTarget::IGNORE_PY |
                    mavros_msgs::PositionTarget::IGNORE_PZ |
                    mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY |
                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::PositionTarget::FORCE |
                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    // Velocidades deseadas (m/s)
    msg.velocity.x = 0.1;   // Adelante
    msg.velocity.y = 0.0;
    msg.velocity.z = 0.0;

    // Yaw deseado (radianes)
    msg.yaw = 0.5;

    vel_pub.publish(msg);
    rate.sleep();
  }

  return 0;
}