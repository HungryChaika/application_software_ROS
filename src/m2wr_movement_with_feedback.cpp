#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <signal.h>
#include <tf/transform_broadcaster.h>
#include <vector>

#define PI 3.1415926


ros::Publisher pub;
ros::Publisher pubTheorTrajectory;
ros::Publisher pubRealLinVel;
ros::Publisher pubRealAngVel;

geometry_msgs::Twist messageStop;
geometry_msgs::Twist messageMotion;
geometry_msgs::Vector3 points;

double linVelReal = 0;
double angVelReal = 0;
double xPosReal = 0;
double yPosReal = 0;
double angleReal = 0;

float k_d = 5;
float k_p = 4;
float time_coef = 0.3;

void mySigintHandler(int sig)
{
    pub.publish(messageStop);
    ros::Duration(1).sleep();
    ROS_INFO("I'm stoped sigint");

    ros::shutdown();
}

void setupPublishers(ros::NodeHandle &nh)
{
    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    pubTheorTrajectory = nh.advertise<geometry_msgs::Vector3>("theor_trajectory", 10);
    pubRealLinVel = nh.advertise<geometry_msgs::Vector3>("real_lin_vel", 100);
    pubRealAngVel = nh.advertise<geometry_msgs::Vector3>("real_ang_vel", 100);
}

void getAngle(const nav_msgs::Odometry::ConstPtr &msg) {
    //angleReal = asin(msg->pose.pose.orientation.z) * 2 * 180 / PI;
    tfScalar yaw, pitch, roll;
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
    tf::Matrix3x3 m(q);
    m.getEulerYPR(yaw, pitch, roll);
    angleReal = yaw;// * 180 / PI;
}

void msgCallbackOdom(const nav_msgs::Odometry::ConstPtr &msg)
{
    linVelReal = msg->twist.twist.linear.x;
    angVelReal = msg->twist.twist.angular.z;
    xPosReal = msg->pose.pose.position.x;
    yPosReal = msg->pose.pose.position.y;
    getAngle(msg);
}

void calculatePointsT(int length, float * arr) {
    double step = 2.0 * PI / length;
    double value = 0.0;
    for(int i = 0; i < length; i++) {
        arr[i] = value;
        value += step;
        ROS_INFO("t=%f", arr[i]);
    }
}

float X(float t)
{
    float Shift = -2;
    return cos(t * time_coef) + cos(2*t * time_coef) + Shift;
}

float Xdt(float t)
{
    return -time_coef*sin(t * time_coef) - 2*time_coef*sin(2*t * time_coef);
}

float Xdtdt(float t)
{
    return -time_coef*time_coef*cos(t * time_coef) - 4*time_coef*time_coef*cos(2*t * time_coef);
}

float Y(float t)
{
    return 2*sin(t * time_coef);
}

float Ydt(float t)
{
    return 2*time_coef*cos(t * time_coef);
}

float Ydtdt(float t)
{
    return -2*time_coef*time_coef*sin(t * time_coef);
}

void calculatePointsOfTrajectory(int length, float * x, float * y, float * t) {
    for(int i = 0; i < length; i++)
    {
        x[i] = X(t[i]/time_coef);
        y[i] = Y(t[i]/time_coef);

        points.x = x[i];
        points.y = y[i];
        ROS_INFO("X=%f, Y=%f", x[i], y[i]);
        pubTheorTrajectory.publish(points);
        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }
}

void CalculateMotionVel(double &omega, double &acceleration, double t)
{
    float X_Part =  Xdtdt(t) + 
                    k_d * (Xdt(t) - linVelReal * cos(angleReal)) + 
                    k_p * (X(t) - xPosReal);
    float Y_Part =  Ydtdt(t) + 
                    k_d * (Ydt(t) - linVelReal * sin(angleReal)) +
                    k_p * (Y(t) - yPosReal);
    acceleration = cos(angleReal) * X_Part + sin(angleReal) * Y_Part;
    omega = cos(angleReal) / linVelReal * Y_Part - sin(angleReal) / linVelReal * X_Part;
}

void publishRealLinAngVel(int count, double lin, double ang) {
    geometry_msgs::Vector3 messageLinVel;
    geometry_msgs::Vector3 messageAngVel;
    messageLinVel.x = count;
    messageLinVel.y = lin;
    pubRealLinVel.publish(messageLinVel);
    messageAngVel.x = count;
    messageAngVel.y = ang;
    pubRealAngVel.publish(messageAngVel);
}

int main(int argc, char *argv[])
{
    int numberPoints = 500;

    float t_points[numberPoints];
    float x_points[numberPoints];
    float y_points[numberPoints];

    ros::init(argc, argv, "motion_with_pid_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ros::Subscriber subOdom = nh.subscribe("odom", 10, msgCallbackOdom);
    ros::spinOnce();

    signal(SIGINT, mySigintHandler);

    setupPublishers(nh);

    calculatePointsT(numberPoints, t_points);
    calculatePointsOfTrajectory(numberPoints, x_points, y_points, t_points);

	double omega, acceleration, timeReal, timeRealOld, velocity, dtReal = 0.0;
	double startTime = ros::Time::now().toSec();

    int count = 0;
	while (timeReal < 2.0 * PI / time_coef)
	{
		messageMotion.linear.x = velocity;
		messageMotion.angular.z = omega;
		pub.publish(messageMotion);
        publishRealLinAngVel(count, velocity, omega);

		ros::Duration(0.05).sleep();
		ros::spinOnce();

        CalculateMotionVel(omega, acceleration, timeReal);
        
        count++;
        timeReal = ros::Time::now().toSec() - startTime;
		dtReal = timeReal - timeRealOld;
        timeRealOld = timeReal;
        velocity = velocity + acceleration * dtReal;
        ROS_INFO("V=%d, W=%d", velocity, omega);
	}

    mySigintHandler(1);
    return 0;
}
