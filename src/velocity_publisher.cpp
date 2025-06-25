#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <cmath>

class VelocityToPoseRef {
private:
  ros::Subscriber vel_sub_;
  ros::Publisher pose_ref_pub_;
  mavros_msgs::PositionTarget pose_ref_msg_;
  geometry_msgs::Vector3 prev_velocity_;
  
  bool first_msg_;
  double threshold_;
  double max_velocity_;

public:
  VelocityToPoseRef(ros::NodeHandle *nh) : first_msg_(true), threshold_(0.1), max_velocity_(0.75){
    vel_sub_ = nh->subscribe("/mavros/local_position/velocity_local", 10, &VelocityToPoseRef::velocityCallback, this);
    pose_ref_pub_ = nh->advertise<mavros_msgs::PositionTarget>("/pose_ref_uav1", 10);

    // Inicializar el mensaje con valores constantes para posición y máscara
    pose_ref_msg_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    pose_ref_msg_.type_mask = (
        mavros_msgs::PositionTarget::IGNORE_PX |
        mavros_msgs::PositionTarget::IGNORE_PY |
        mavros_msgs::PositionTarget::IGNORE_PZ |
        mavros_msgs::PositionTarget::IGNORE_AFX |
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ |
        mavros_msgs::PositionTarget::FORCE |
        mavros_msgs::PositionTarget::IGNORE_YAW |
        mavros_msgs::PositionTarget::IGNORE_YAW_RATE
    );
  }

  geometry_msgs::Vector3 saturateVelocity(const geometry_msgs::Vector3& vel) {
    geometry_msgs::Vector3 saturated = vel;
    double norm = std::sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z);

    if (norm > max_velocity_) {
      double scale = max_velocity_ / norm;
      saturated.x *= scale;
      saturated.y *= scale;
      saturated.z *= scale;
    }

    return saturated;
  }

  void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    geometry_msgs::Vector3 current_velocity = msg->twist.linear;
    
    double delta_norm = std::sqrt(
      std::pow(current_velocity.x - prev_velocity_.x, 2) +
      std::pow(current_velocity.y - prev_velocity_.y, 2) +
      std::pow(current_velocity.z - prev_velocity_.z, 2)
    );

    pose_ref_msg_.header.stamp = ros::Time::now();

    if (first_msg_) {
      pose_ref_msg_.velocity.x = 0.0;
      pose_ref_msg_.velocity.y = 0.0;
      pose_ref_msg_.velocity.z = 0.0;
    } else if ( delta_norm > threshold_){
      pose_ref_msg_.velocity = saturateVelocity(current_velocity);
      //prev_velocity_ = current_velocity;
    } else {
      pose_ref_msg_.velocity = saturateVelocity(prev_velocity_);
    }

    pose_ref_pub_.publish(pose_ref_msg_);
    prev_velocity_ = current_velocity;
    first_msg_ = false;
  }

  void spin() {
    ros::Rate rate(50);  // 50 Hz
    while (ros::ok()) {
      ros::spinOnce();
      rate.sleep();
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "velocity_publisher");
  ros::NodeHandle nh;

  VelocityToPoseRef v2p(&nh);
  v2p.spin();

  return 0;
}
