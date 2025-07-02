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
  geometry_msgs::TwistStamped local_vel_;

  geometry_msgs::Twist vel_cmd_;
  ros::Publisher set_pose_pub_;
  ros::Subscriber local_pose_sub_, local_vel_sub_, offboard_sub_, rc_in_sub_;
  bool isInit_, isOffboard_;
  bool isAligmentMode_;

  double hold_z_;
  double hold_yaw_;
  bool free_floating;

  std::vector<geometry_msgs::Vector3> dir_buffer_;
  std::vector<double> yaw_rate_buffer_;
  bool dir_mean_computed_ = false;
  geometry_msgs::Vector3 dir_mean_;
  double yaw_rate_mean_;


public:
  pose_controller(ros::NodeHandle *nh) {
    set_pose_pub_ = nh->advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
    local_pose_sub_ = nh->subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &pose_controller::localPoseCallback, this);
    local_vel_sub_ = nh->subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 1, &pose_controller::velocityCallback, this);
    offboard_sub_ = nh->subscribe<mavros_msgs::State>("/mavros/state", 1, &pose_controller::offboardCallback, this);
    
    isInit_ = false;
    isOffboard_ = false;
    free_floating = false;
    yaw_rate_mean_ = 0.0;

  }

  void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg) {
    local_vel_.twist = msg->twist;
  }


  // Función de callback para la posición local
  void localPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    local_pose_.pose = msg->pose;

    double vel_x = local_vel_.twist.linear.x;
    double vel_y = local_vel_.twist.linear.y;
    double norm_vel = std::sqrt(vel_x * vel_x + vel_y * vel_y);

    double vel_yaw = local_vel_.twist.angular.z;

    double v_max = 0.5;     // velocidad máxima que quieres aplicar
    double vel_thresh = 0.15; // umbral de activación mínima

    double yaw_rate_max = 0.5;     // velocidad máxima que quieres aplicar RAD/s
    double yaw_rate_thresh = 0.2; // umbral de activación mínima

    if (!isOffboard_){
      sp.type_mask = 0; // 0 set the type mask as needed  msg->type_mask
      sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
      sp.position.x = local_pose_.pose.position.x;
      sp.position.y = local_pose_.pose.position.y;
      hold_z_ = local_pose_.pose.position.z;
      sp.position.z = hold_z_;
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
      hold_yaw_ = yaw;
      sp.yaw = hold_yaw_;
      sp.yaw_rate = 0.0;
      free_floating = false;

    }else {

      sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

      // Type mask: ignora TODO excepto Z (posición) y Yaw
      // Binario: 0b011111111000 = 0x7F8 = 2040
      // Posición: ignorar x,y (bit 0,1), mantener z (bit 2)
      // Velocidad: ignorar x,y,z (bits 3,4,5)
      // Aceleración: ignorar x,y,z (bits 6,7,8)
      // Fuerza: ignorada porque usamos aceleración_or_force
      // Yaw: mantener (bit 9 = 0), Yaw_rate: ignorar (bit 10 = 1)
      sp.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                     mavros_msgs::PositionTarget::IGNORE_PY |
                     mavros_msgs::PositionTarget::IGNORE_VZ |
                     mavros_msgs::PositionTarget::IGNORE_AFX |
                     mavros_msgs::PositionTarget::IGNORE_AFY |
                     mavros_msgs::PositionTarget::IGNORE_AFZ |
                     mavros_msgs::PositionTarget::IGNORE_YAW;

       if (norm_vel > vel_thresh && !dir_mean_computed_) {
        geometry_msgs::Vector3 dir_unit;
        dir_unit.x = vel_x / norm_vel;
        dir_unit.y = vel_y / norm_vel;
        dir_unit.z = 0.0;
        dir_buffer_.push_back(dir_unit);
        yaw_rate_buffer_.push_back(vel_yaw);
       
        if (dir_buffer_.size() >= 20) {
          // Calcular media
          dir_mean_.x = 0.0;
          dir_mean_.y = 0.0;
          for (const auto& v : dir_buffer_) {
            dir_mean_.x += v.x;
            dir_mean_.y += v.y;
          }
          dir_mean_.x /= dir_buffer_.size();
          dir_mean_.y /= dir_buffer_.size();
       
          // Normalizar dirección media
          double norm = std::sqrt(dir_mean_.x * dir_mean_.x + dir_mean_.y * dir_mean_.y);
          if (norm > 1e-3) {
            dir_mean_.x /= norm;
            dir_mean_.y /= norm;
          }

          yaw_rate_mean_ = 0.0;
          for (const auto& w : yaw_rate_buffer_) {
            yaw_rate_mean_ += w;
          }
          yaw_rate_mean_ /= yaw_rate_buffer_.size();
       
          dir_mean_computed_ = true;
          ROS_WARN("Vector medio de dirección calculado");
        }
      }

      if (dir_mean_computed_) {
        sp.velocity.x = dir_mean_.x * v_max;
        sp.velocity.y = dir_mean_.y * v_max;

        // If paraa el yaw rate
        if (yaw_rate_mean_ >= yaw_rate_max) {
          sp.yaw_rate = yaw_rate_max; 
        } else if(yaw_rate_mean_ <= -yaw_rate_max){
          sp.yaw_rate = -yaw_rate_max;
        }else if (yaw_rate_mean_ > yaw_rate_thresh || yaw_rate_mean_ < -yaw_rate_thresh){
          sp.yaw_rate = yaw_rate_mean_;
        } else {
          sp.yaw_rate = 0; 
        }
        free_floating = true;
      }

      if (!free_floating){
        sp.velocity.x = 0.0;
        sp.velocity.y = 0.0; 
        sp.yaw_rate = 0.0;
      }
    
      // Mantener altura 
      sp.position.z = hold_z_;      
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
      dir_buffer_.clear();
      dir_mean_computed_ = false;
    }
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
