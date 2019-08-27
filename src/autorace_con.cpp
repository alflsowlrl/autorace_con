#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <ctime>

#define DT 0.1
#define ABS(x) (((x) > 0) ? (x) : (-x))
class Vector3f{
public:
  double x, y, z;
  Vector3f()
    :x(0), y(0), z(0) {}
  Vector3f(double xx, double yy, double zz)
    :x(xx), y(yy), z(zz) {}
  Vector3f(const geometry_msgs::Point& p)
    :x(p.x), y(p.y), z(p.z) {}
  Vector3f(const geometry_msgs::Vector3& p)
    :x(p.x), y(p.y), z(p.z) {}
  Vector3f& operator=(const geometry_msgs::Point& p){
    x = p.x; y = p.y; z = p.z;
    return (*this);
  }
  Vector3f& operator=(const geometry_msgs::Vector3& p){
    x = p.x; y = p.y; z = p.z;
    return (*this);
  }
  
  void print(){
    ROS_INFO("%f, %f, %f", x, y, z);
  }
};

std::ostream& operator<<(std::ostream& os, const Vector3f& v){
  os<<v.x<<", "<<v.y<<", "<<v.z;
  return os;
}

double find_yaw(const geometry_msgs::Pose pose);

double right_wheel_vel = 0;
double left_wheel_vel = 0;

double yaw = 0;
double yaw_msg = 0;
int y_msg = 145;

Vector3f position, angular, linear;

void odomCB(const nav_msgs::Odometry& msg){
  yaw = find_yaw(msg.pose.pose);
  position = msg.pose.pose.position;
  linear = msg.twist.twist.linear;
  angular = msg.twist.twist.angular;

  ROS_INFO("yaw: %f", yaw * 180 / M_PI);
  //ROS_INFO_STREAM("position: "<<position);
  //ROS_INFO_STREAM("linear: "<<linear);
  //ROS_INFO_STREAM("angular: "<<angular);
}

void yawCB(const std_msgs::Float64 msg){
  yaw_msg = msg.data;
  ROS_INFO("line_yaw: %f", yaw_msg);
}

void yCB(const std_msgs::Int32 msg){
  y_msg = msg.data;
  ROS_INFO("line_y: %d", y_msg);
 
}

double find_yaw(const geometry_msgs::Pose pose){
  double w = pose.orientation.w;
  double x = pose.orientation.x;
  double y = pose.orientation.y;
  double z = pose.orientation.z;

  double siny_cosp = +2.0 * (w * z + x * y);
  double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);  
  double yaw = atan2(siny_cosp, cosy_cosp);
  return yaw;
}

int main(int argc, char ** argv){
  ros::init(argc, argv, "simple_con");
  ros::NodeHandle nh;
  //ros::Subscriber odom_sub = nh.subscribe("odom", 1, odomCB);
  ros::Subscriber yaw_sub = nh.subscribe("/detect/yaw", 1, yawCB);
  ros::Subscriber y_sub = nh.subscribe("/detect/y", 1, yCB);
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Publisher error_pub = nh.advertise<std_msgs::Float64>("error_x", 1);
  ros::Duration nap(DT);

  double Kx, Ky, Kyaw;
  
  double linear_d = 0;
  double angular_d = 0;
  double x_d = 0; double y_d = 145; double yaw_d = 0;
  double v = 0.17;
  
  double error_x, error_y, error_yaw;
  
  geometry_msgs::Twist msg;
  std_msgs::Float64 error_msg;

  while(ros::ok()){
   
     
    Kx = 1;
    Ky = 5;
    Kyaw = 5;

    
    x_d += v*DT;
    linear_d = v;
    angular_d = 0;
    
    error_x = v*DT;
    error_y = 0.04*(y_d - y_msg);
    error_yaw = 0;
    ROS_INFO("error: %f, %f, %f", error_x, error_y, error_yaw);
 
    double linear_vel = linear_d*cos(error_yaw) + Kx*error_x;
    double angular_vel = angular_d + linear_d*(Ky*error_y + Kyaw*sin(error_yaw));
    msg.linear.x = std::min(linear_vel, 0.22);
    if(angular_vel > 0)
      msg.angular.z = std::min(angular_vel, 2.84);
    else
      msg.angular.z = std::max(angular_vel, -2.84);

    ROS_INFO("control l: %f a:%f", msg.linear.x, msg.angular.z);
    vel_pub.publish(msg);
    
    ros::spinOnce();
    nap.sleep();

  }

  return 0;
}
