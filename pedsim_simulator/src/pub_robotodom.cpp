#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

ros::Publisher odom_pub, pose_pub;
geometry_msgs::Twist cmd_vel;
geometry_msgs::PoseWithCovarianceStamped robot_pose_cov;
nav_msgs::Odometry robot_odom;
std::string odom_frame, map_frame, base_frame;
double publish_frequency;

void cmd_velCbk(const geometry_msgs::Twist& _cmd_vel) 
{   
    robot_odom.twist.twist = _cmd_vel;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd2robotodom");
    ros::NodeHandle n;
    ros::NodeHandle nh_priv("~");
    nh_priv.param<std::string>("odom_frame",odom_frame,"odom");
    nh_priv.param<std::string>("map_frame",map_frame,"map");
    nh_priv.param<std::string>("base_frame",base_frame,"base_link");
    nh_priv.param<double>("publish_frequency",publish_frequency,20);

    odom_pub = n.advertise<nav_msgs::Odometry>("/robot_odom", 1000); 
    pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/robot_pose", 1000); 

    ros::Subscriber cmd_sub =
        n.subscribe("/cmd_vel", 1000, cmd_velCbk);

    ros::Rate rate(publish_frequency);

    while (n.ok())
    {
      tf::StampedTransform transform;
      tf::TransformListener listener;
      try{
            listener.waitForTransform(map_frame, base_frame, ros::Time(), ros::Duration(1.0));
            listener.lookupTransform(map_frame, base_frame, ros::Time(0), transform);
            // construct a pose message
            robot_pose_cov.header.frame_id = map_frame;
            robot_pose_cov.header.stamp = ros::Time(0);

            robot_pose_cov.pose.pose.orientation.x = transform.getRotation().getX();
            robot_pose_cov.pose.pose.orientation.y = transform.getRotation().getY();
            robot_pose_cov.pose.pose.orientation.z = transform.getRotation().getZ();
            robot_pose_cov.pose.pose.orientation.w = transform.getRotation().getW();

            robot_pose_cov.pose.pose.position.x = transform.getOrigin().getX();
            robot_pose_cov.pose.pose.position.y = transform.getOrigin().getY();
            robot_pose_cov.pose.pose.position.z = transform.getOrigin().getZ();
            pose_pub.publish(robot_pose_cov);

            robot_odom.header.frame_id = odom_frame;
            robot_odom.header.stamp = ros::Time(0);
            robot_odom.pose = robot_pose_cov.pose;
            odom_pub.publish(robot_odom);
          } catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
          }
      ros::spinOnce();
      rate.sleep();
    }
    return 0;
}

