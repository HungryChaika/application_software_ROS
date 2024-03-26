#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <iostream>

using namespace std;

ros::Publisher pub;

geometry_msgs::Twist messageStop;
geometry_msgs::Twist messageMotion;

double target_X;
double target_Y;
double realPosX;
double realPosY;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    realPosX = msg->pose.pose.position.x;
    realPosY = msg->pose.pose.position.y;
}

void stopRobot()
{
    pub.publish(messageStop);
    ros::Duration(1).sleep();
    ROS_INFO("STOP");
    ros::shutdown();
}

int main(int argc, char *argv[])
{   
    ros::init(argc, argv, "move_to_point_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ros::Subscriber subOdom = nh.subscribe("odom", 100, odomCallback);
    ros::spinOnce();

    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    cout << "Введите координату X: ";
    cin >> target_X;
    cout << "Введите координату Y: ";
    cin >> target_Y;

    double dt = 5;
    double dY = target_Y;
    double dX = target_X;
    double distance = sqrt(dY*dY + dX*dX);
    double angle = atan2(dY, dX);
    double omega = angle / dt;
    double velocity = distance / dt;

    cout << "dY: " << dY << endl;
    cout << "dX: " << dX << endl;
    cout << "distance: " << distance << endl;
    cout << "angle: " << angle * 360 / M_PI << endl;
    cout << "angle_rad: " << angle << endl;
    cout << "omega: " << omega << endl;
    cout << "velocity: " << velocity << endl;

    messageMotion.angular.z = omega;
    pub.publish(messageMotion);
    ros::Duration(dt).sleep();
    ros::spinOnce();

    messageMotion.angular.z = 0;
    messageMotion.linear.x = velocity;
    pub.publish(messageMotion);
    ros::Duration(dt).sleep();
    ros::spinOnce();

    cout << "REALposX: " << realPosX << endl;
    cout << "REALposY: " << realPosY << endl;

    stopRobot();

    return 0;
}
