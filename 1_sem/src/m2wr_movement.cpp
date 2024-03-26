#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <signal.h>
#include <vector>

#define PI 3.1415926


ros::Publisher pub;
ros::Publisher pubTheorTrajectory;
ros::Publisher pubTheorLinVel;
ros::Publisher pubTheorAngVel;
ros::Publisher pubRealLinVel;
ros::Publisher pubRealAngVel;

geometry_msgs::Twist messageStop;
geometry_msgs::Twist messageMotion;
geometry_msgs::Vector3 points;

double linVelReal = 0;
double angVelReal = 0;

void StopRobot(void);

void mySigintHandler(int sig)
{
    StopRobot();
    ROS_INFO("I'm stoped sigint");
    
    ros::shutdown();
}

void msgCallbackOdom(const nav_msgs::Odometry::ConstPtr &msg)
{
    linVelReal = msg->twist.twist.linear.x;
    angVelReal = msg->twist.twist.angular.z;
}

void CalculateParametrizationFunction(int length, float * arr)
{
    int i = 0;
    float endT = 2*PI;    // for linear

    for(i = 0; i < length; i++)
    {
        float s = ((float)i/(float)length)*endT;
        arr[i] = s;                       // linear
        
        ROS_INFO("i=%d, t=%f", i, arr[i]);    
    }    
}

void CalculatePointsOfTrajectory(int length, float * x, float * y, float * t)
{
    float kTr = 2;//1
    int i =0;
    for(i=0; i < length; i++)
    {
        //trajectory "boomerang"
        x[i] = (cos(t[i]) + cos(2*t[i])) ;//* kTr;
        y[i] = (3*sin(t[i]) - sin(t[i])) ;//* kTr;

        points.x = y[i];
        points.y = (x[i] - kTr) * (-1);
        pubTheorTrajectory.publish(points);
        ros::Duration(0.01).sleep();    
    }
}

float Range(float val, float min, float max)
{
    if(val < min)
        return min;

    if(val>max)
        return max;

    return val;
}

float RemapAngle(float val)
{
    if(val > PI) { val -= 2*PI; }
    if(val < -PI) { val += 2*PI; }

    return val;
}
float GetMax(int length, float *arr)
{
    float max = 0;

    for(int i = 0; i < length; i++)
    {
        if(arr[i] > max)
        {
            max = arr[i];
        }
    }

    return max;
}
float GetMin(int length, float *arr)
{
    float min = arr[0];

    for(int i = 0; i < length; i++)
    {
        if(arr[i] < min)
        {
            min = arr[i];
        }
    }

    return min;
}
void CalculateVelocities(int length, float dt, float thetaStart, float * x, float * y, float * v, float * w)
{
    float dx, dy;
    float tetta[length];
    float dl[length];
    float dw[length];
    
    for(int i = 0; i < length; i++)
    {
        dx = x[i+1] - x[i];
        dy = y[i+1] - y[i];
        dl[i] = sqrt(dx*dx + dy*dy);
        tetta[i] = atan2(y[i+1] - y[i], x[i+1] - x[i]);
        if(tetta[i] < 0)
        {
            tetta[i] += 2*PI;
        }

        if(i==0)
        {
            dw[i] = tetta[i] - thetaStart;
        }
        else
        {
            dw[i] = tetta[i] - tetta[i-1];
        }

        dw[i] = RemapAngle(dw[i]);

        v[i] = dl[i] / dt;
        w[i] = dw[i] / dt;             
    }    
}

void PublishTheoreticalVelocities(int length, float * arr, float coeff, ros::Publisher &pub)
{
    geometry_msgs::Vector3 message;
    for(int i = 0; i < length; i++)
    {
        message.x = i;
        message.y = arr[i]*coeff;
        
        pub.publish(message);
        ros::Duration(0.01).sleep();        
    }
}

void TransmitVelocitiesToRobot(int length, float dt, float k, float * v, float * w,  bool transmit)
{
    geometry_msgs::Vector3 messageLinVel;
    geometry_msgs::Vector3 messageAngVel;


    for(int i = 0; i < length; i++)
    {
        ROS_INFO("i=%d, v=%f, w=%f", i, v[i], w[i]);
                
        messageMotion.linear.x = v[i]*k;
        messageMotion.angular.z = w[i]*k;
        
        if(transmit)
        {   
            pub.publish(messageMotion);
            
            messageLinVel.x = i;
            messageLinVel.y = linVelReal;
            pubRealLinVel.publish(messageLinVel);

            messageAngVel.x = i;
            messageAngVel.y = angVelReal;
            pubRealAngVel.publish(messageAngVel);

            ros::Duration(dt/k).sleep();
        }
        ros::spinOnce();
    }
}

void StopRobot()
{
    pub.publish(messageStop);
    ros::Duration(1).sleep();
}

void SetupPublishers(ros::NodeHandle &nh)
{
    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    pubTheorTrajectory = nh.advertise<geometry_msgs::Vector3>("theor_trajectory", 1000);
    pubTheorLinVel = nh.advertise<geometry_msgs::Vector3>("theor_lin_vel", 1000);
    pubTheorAngVel = nh.advertise<geometry_msgs::Vector3>("theor_ang_vel", 1000);
    pubRealLinVel = nh.advertise<geometry_msgs::Vector3>("real_lin_vel", 1000);
    pubRealAngVel = nh.advertise<geometry_msgs::Vector3>("real_ang_vel", 1000);
}

int main(int argc, char* argv[])
{
    const int numberPoints = 500;
    const bool transmit = true;
    const float maxLinVelocityReal = 1.5;
    const float maxAngVelocityReal = 1.5;
    const float thetaStart = PI/2;

    // init arrays
    float t[numberPoints];
    float x[numberPoints];
    float y[numberPoints];
    
    float v[numberPoints-1];
    float w[numberPoints-1];

    float dt = 0.05;

    // init ROS
    ros::init(argc, argv, "motion_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    // init of subscriber to get real velocities while motion
    ros::Subscriber subOdom = nh.subscribe("odom", 100, msgCallbackOdom);
    ros::spinOnce();

    // init sigint event, this event occurs when node exit (CTRL+C)
    signal(SIGINT, mySigintHandler);   
    
    // init publishers
    SetupPublishers(nh);
        
    // calculate parameters for motion
    CalculateParametrizationFunction(numberPoints, t);
    CalculatePointsOfTrajectory(numberPoints, x, y, t);
    CalculateVelocities(numberPoints-1, dt, thetaStart, x, y, v, w);

    float maxLinVelocity = GetMax(numberPoints-1, v);
    float maxAngVelocity = GetMax(numberPoints-1, w);

    ROS_INFO("maxV = %f, maxW = %f", maxLinVelocity, maxAngVelocity);

    ros::spinOnce();

    // calculate coefficient of scale velocities
    float k1,k2,k;
    float coeffSafety = 1;
    k1 = maxLinVelocityReal/maxLinVelocity * coeffSafety;
    k2 = maxAngVelocityReal/maxAngVelocity * coeffSafety;

    k = GetMin(2, new float[2] {k1, k2});
    k = Range(k, 0.1, coeffSafety);
    
    ROS_INFO("k=%f", k);

    // publish calculated linear and angular velocities
    PublishTheoreticalVelocities(numberPoints-1, v, k, pubTheorLinVel);
    PublishTheoreticalVelocities(numberPoints-1, w, k, pubTheorAngVel);

    TransmitVelocitiesToRobot(numberPoints-1, dt, k, v, w, transmit);

    StopRobot();
    
    mySigintHandler(1);
    return 0;
    
}