#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

//PID algorithm
const float kp = 1;//TODO
const float kd = 0.1f;//TODO
const float ki = 0.0f;//TODO

float error, last_error = 0;
double last_time;
double Dt, Dt1;

//distance to wall
const float distance_to_wall=1.3;

// laserScans
int a, b; 
const auto L=1;//TODO
const auto theta=30/180*M_PI;//theta em radianos TODO monhe falou de 30 graus nas palestras

ros::Subscriber sub;
ros::Publisher pub;

void callback_scan(sensor_msgs::LaserScan scan);
void publish_nav(ackermann_msgs::AckermannDriveStamped msg);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wall_follow");
    ros::NodeHandle nh;

    //init pub and subs
    sub = nh.subscribe("/scan", 1, callback_scan);
    pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 1);

    ros::spin();

    last_time=ros::Time::now().toSec();
}


int getIndex(double angulo, sensor_msgs::LaserScan scan) {
    double angulo_real = (3*M_PI/2 - angulo); // 270 graus menos o angulo da o angulo 0 no eixo dos x (lado direito do carro)
    int idx =  angulo_real / scan.angle_increment; // obter o indice dividindo o angulo pelo incremento
    return idx;
}


void callback_scan(sensor_msgs::LaserScan scan)
{
    ROS_INFO("Received scan!");
    ROS_INFO("%f increments with %llu total scans!", scan.angle_increment, scan.ranges.size());
    //ranges vai de menos pi até pi (sentido do relogio... ou seja,... para tras temos o -pi, à esquerda o --pi+pi/2 em frente o 0 (indice do meio)  e por ai fora)

    a=scan.ranges[getIndex(theta, scan)]; // scan at theta degrees
    b=scan.ranges[getIndex(0, scan)]; // scan at 0 degrees (3*pi/2 rads do scan)

    auto alpha = atan(a*cos(theta)-b / a*sin(theta));
    
    Dt = b*cos(alpha);
    Dt1 = Dt+L*sin(alpha);

    error = Dt1 - distance_to_wall;

    ackermann_msgs::AckermannDriveStamped msg;
    auto dt = last_time - ros::Time::now().toSec();

    //PID without integral
    msg.drive.steering_angle = kp * error + kd * (error - last_error) / dt;

    if(msg.drive.steering_angle < 10/180*M_PI )
    {
        msg.drive.speed = 1.5;
    }
    else if(msg.drive.steering_angle < 20/180*M_PI )
    {
        msg.drive.speed = 1;
    }
    else
    {
        msg.drive.speed = 0.5;
    }

    publish_nav(msg);
    last_error = error;
    last_time = ros::Time::now().toSec();
}

void publish_nav(ackermann_msgs::AckermannDriveStamped msg)
{
    pub.publish(msg);
}