#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

ros::Publisher odom_pub;
geometry_msgs::Twist cmd_vel;
nav_msgs::Odometry robot_position;

void robotpositionCbk(const nav_msgs::Odometry& _robot_position) {
  robot_position = _robot_position;

  nav_msgs::Odometry robot_odom;
  robot_odom.pose = robot_position.pose;
  robot_odom.twist.twist = cmd_vel;
  odom_pub.publish(robot_odom);
};

void cmd_velCbk(const geometry_msgs::Twist& _cmd_vel) 
{ 
    cmd_vel = _cmd_vel; 
    nav_msgs::Odometry robot_odom;
    robot_odom.pose = robot_position.pose;
    robot_odom.twist.twist = cmd_vel;
    odom_pub.publish(robot_odom);
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd2robotodom");
    ros::NodeHandle n;
    odom_pub = n.advertise<nav_msgs::Odometry>("/pose", 1000); 
    ros::Subscriber robotpos_sub =
        n.subscribe("/pedsim_simulator/robot_position", 1000, robotpositionCbk);
    ros::Subscriber cmd_sub =
        n.subscribe("/pedbot/control/cmd_vel", 1000, cmd_velCbk);
    ros::spin();
    return 0;
}

