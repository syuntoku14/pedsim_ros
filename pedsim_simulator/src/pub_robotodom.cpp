#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

ros::Publisher odom_pub, pose_pub;
geometry_msgs::Twist cmd_vel;
std::string odom_frame, map_frame, base_frame;

void cmd_velCbk(const geometry_msgs::Twist& _cmd_vel) 
{   
    nav_msgs::Odometry robot_odom;
    robot_odom.header.frame_id = odom_frame;
    robot_odom.header.stamp = ros::Time(0);

    tf::StampedTransform transform;
    tf::TransformListener listener;

    try{
      listener.waitForTransform(map_frame, base_frame, ros::Time(), ros::Duration(1.0));
      listener.lookupTransform(map_frame, base_frame, ros::Time(0), transform);
      // construct a pose message

      geometry_msgs::PoseWithCovarianceStamped robot_pose_cov;
      robot_pose_cov.header.frame_id = map_frame;
      robot_pose_cov.header.stamp = ros::Time(0);

      robot_pose_cov.pose.pose.orientation.x = transform.getRotation().getX();
      robot_pose_cov.pose.pose.orientation.y = transform.getRotation().getY();
      robot_pose_cov.pose.pose.orientation.z = transform.getRotation().getZ();
      robot_pose_cov.pose.pose.orientation.w = transform.getRotation().getW();

      robot_pose_cov.pose.pose.position.x = transform.getOrigin().getX();
      robot_pose_cov.pose.pose.position.y = transform.getOrigin().getY();
      robot_pose_cov.pose.pose.position.z = transform.getOrigin().getZ();

      robot_odom.pose = robot_pose_cov.pose;
      robot_odom.twist.twist = _cmd_vel;

      pose_pub.publish(robot_pose_cov);
      odom_pub.publish(robot_odom);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd2robotodom");
    ros::NodeHandle n;
    ros::NodeHandle nh_priv("~");

    nh_priv.param<std::string>("odom_frame",odom_frame,"odom");
    nh_priv.param<std::string>("map_frame",map_frame,"map");
    nh_priv.param<std::string>("base_frame",base_frame,"base_link");

    odom_pub = n.advertise<nav_msgs::Odometry>("/robot_odom", 1000); 
    pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/robot_pose", 1000); 

    ros::Subscriber cmd_sub =
        n.subscribe("/pedbot/control/cmd_vel", 1000, cmd_velCbk);
    ros::spin();
    return 0;
}

