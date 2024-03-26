#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>


ros::Publisher pub;
geometry_msgs::Twist vel;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "m2wr_twist", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  while (ros::ok())
  {
    vel.linear.x = 0.3;
    vel.angular.z = 0.1;
    pub.publish(vel);
    ros::spinOnce();
  }
return 0;
}


/*
void moveRobot(double X, double Z) {
  vel.linear.x += X;
  vel.angular.z += Z;
  pub.publish(vel);
}

void stopRobot() {
  vel.linear.x = 0.0;
  vel.angular.z = 0.0;
  pub.publish(vel);
}

void mySigintHandler(int sig) {
  stopRobot();
  ROS_INFO("Sigint Event");
  ros::shutdown();
}

int main(int argc, char **argv)
{
  signal(SIGINT, mySigintHandler);
  ros::init(argc, argv, "m2wr_twist", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  while (ros::ok())
  {
    char ch = getchar();
    if(ch == 'w') {
      moveRobot(0.2, 0.0);
    } else if(ch == 's') {
      moveRobot(-0.2, 0.0);
    } else if(ch == 'd') {
      moveRobot(0.0, -0.1);
    } else if(ch == 'a') {
      moveRobot(0.0, 0.1);
    } else if(ch == 'q') {
      stopRobot();
    }
    ROS_INFO("X: %f", vel.linear.x);
    ROS_INFO("Z: %f", vel.angular.z);
    ros::spinOnce();
  }
  mySigintHandler(1);
  return 0;
}
*/