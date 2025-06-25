#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/PositionTarget.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

class pose_controller {
private:
  mavros_msgs::PositionTarget sp;
  geometry_msgs::PoseStamped local_pose_;
  geometry_msgs::Twist vel_cmd_;
  ros::Publisher set_pose_pub_;
  ros::Subscriber local_pose_sub_, pose_ref_sub_, offboard_sub_, rc_in_sub_;
  bool isInit_, isOffboard_;
  bool isAligmentMode_;

public:
  pose_controller(ros::NodeHandle *nh) {
    set_pose_pub_ = nh->advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
    local_pose_sub_ = nh->subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &pose_controller::localPoseCallback, this);
    pose_ref_sub_ = nh->subscribe<mavros_msgs::PositionTarget>("/pose_ref_uav1", 1, &pose_controller::poseRefCallback, this);

    offboard_sub_ = nh->subscribe<mavros_msgs::State>("/mavros/state", 1, &pose_controller::offboardCallback, this);
    
    isInit_ = false;
    isOffboard_ = false;
  }
  // Función de callback para la posición local
  void localPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    local_pose_.pose = msg->pose;
    if (!isOffboard_){
      sp.type_mask = 0; // 0 set the type mask as needed  msg->type_mask
      sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
      sp.position.x = local_pose_.pose.position.x;
      sp.position.y = local_pose_.pose.position.y;
      sp.position.z = local_pose_.pose.position.z;
      sp.velocity.x = 0.0;
      sp.velocity.y = 0.0;
      sp.velocity.z = 0.0;
      sp.acceleration_or_force.x = 0.0;
      sp.acceleration_or_force.y = 0.0;
      sp.acceleration_or_force.z = 0.0;

      double quatx= local_pose_.pose.orientation.x;
      double quaty= local_pose_.pose.orientation.y;
      double quatz= local_pose_.pose.orientation.z;
      double quatw= local_pose_.pose.orientation.w;

      tf::Quaternion q(quatx, quaty, quatz, quatw);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      //ROS_INFO("Yaw: [%f]",yaw*180/3.1415);

      sp.yaw = yaw;
      sp.yaw_rate = 0.0;
    }
    isInit_ = true;
  }
   
  void offboardCallback(const mavros_msgs::State::ConstPtr &msg) {
    if (msg->mode == "OFFBOARD") {
      isOffboard_ = true;
      ROS_WARN_STREAM_THROTTLE(5.0, "WE ARE IN OFFBOARD");
    } else {
      ROS_ERROR_STREAM_THROTTLE(5.0, "NO OFFBOARD");
      isOffboard_ = false;
    }
  }

  // Función de callback para la pose de referencia
  void poseRefCallback(const mavros_msgs::PositionTarget::ConstPtr &msg) {

    sp.type_mask = msg->type_mask; 
    sp.coordinate_frame = msg->coordinate_frame;
    sp.position = msg->position;
    sp.velocity = msg->velocity;
    sp.acceleration_or_force = msg->acceleration_or_force;
    sp.yaw = msg->yaw;
    sp.yaw_rate = msg->yaw_rate;
    ROS_ERROR_STREAM_THROTTLE(1.0, "Traj OFFBOARD");

  }
  
  void commandPose() {
    // Configura la frecuencia del bucle principal
    ros::Rate loop_rate(50.0);
    while (ros::ok() && !isInit_) {
      ros::Duration(0.05).sleep();
      ros::spinOnce();
    }
    ROS_WARN_STREAM("Got local pose!");

    while (ros::ok()) {
      // Publica la posicion de referencia que lee del nodo
      sp.header.stamp = ros::Time::now();
      set_pose_pub_.publish(sp);

    
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "Offboard_Node_UAV1");
  ros::NodeHandle nh;
  pose_controller controller = pose_controller(&nh);

  controller.commandPose();
}