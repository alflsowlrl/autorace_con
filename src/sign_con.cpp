#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

#include <cmath>


#define DT 0.1
#define ABS(x) (((x) > 0) ? (x) : (-x))
#define SLOW 0.18
#define FAST 0.18

enum{YELLOW, GREEN, RED, NONE};

class Controller{
private:
  ros::NodeHandle nh;
  ros::Subscriber y_sub;
  ros::Subscriber sign_sub;
  ros::Publisher vel_pub;
  ros::Publisher error_pub;

  double Kx, Ky, Kyaw;
  double linear_d, angular_d;
  double x_d, y_d, yaw_d;
  int sign_code, y_msg;
public:
  Controller(ros::NodeHandle nh);
  void yCB(const std_msgs::Int32 msg);
  void signCB(const std_msgs::Int32 msg);
  void control();
};

Controller::Controller(ros::NodeHandle nh)
  :nh(nh)
{
  y_sub = nh.subscribe("/detect/y", 1, &Controller::yCB, this);
  sign_sub = nh.subscribe("/detect/sign", 1, &Controller::signCB, this);
  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  linear_d = SLOW;
  angular_d = 0;
  x_d = 0; y_d = 145; yaw_d = 0;

  Kx = 1;
  Ky = 5;
  Kyaw = 5;

  y_msg = 145;
  sign_code = GREEN;
}
void Controller::yCB(const std_msgs::Int32 msg){
  y_msg = msg.data;
  //ROS_INFO("line_y: %d", y_msg);
 
}

void Controller::signCB(const std_msgs::Int32 msg){
  static bool was_red = false;
  sign_code = msg.data;
  double K = 1/0.8;
  switch(sign_code){
    case YELLOW:
      if(linear_d > 0.01)
        linear_d *= 0.95;
      ROS_INFO("yellow");
      break;
    case RED:
      linear_d = 0;
      was_red = true;
      ROS_INFO("red");
      break;
    case GREEN:
      if(was_red){
        linear_d = FAST;
        sign_sub.shutdown();
      }
      ROS_INFO("green");
      break;
    default:
      ROS_INFO("none");
    }
}

void Controller::control(){
  angular_d = 0;
  
  geometry_msgs::Twist msg;
  double error_x = linear_d*DT;
  double error_y = 0.04*(y_d - y_msg);
  double error_yaw = 0;
  ROS_INFO("vel: %f", linear_d);
  //ROS_INFO("error: %f, %f, %f", error_x, error_y, error_yaw);
 
  double linear_vel = linear_d*cos(error_yaw) + Kx*error_x;
  double angular_vel = angular_d + linear_d*(Ky*error_y + Kyaw*sin(error_yaw));
  msg.linear.x = std::min(linear_vel, 0.22);
  if(angular_vel > 0)
    msg.angular.z = std::min(angular_vel, 2.84);
  else
    msg.angular.z = std::max(angular_vel, -2.84);

  ROS_INFO("control l: %f a:%f", msg.linear.x, msg.angular.z);
  vel_pub.publish(msg);
}

    

int main(int argc, char ** argv){
  ros::init(argc, argv, "simple_con");
  ros::NodeHandle nh;
  Controller controller(nh);
  ros::Duration nap(DT);

  while(ros::ok()){
    ros::spinOnce();
    controller.control();
    nap.sleep();
  }

  return 0;
}
