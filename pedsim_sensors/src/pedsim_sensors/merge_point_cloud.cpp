#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

ros::Publisher pub;
ros::Subscriber ped_sub, obs_sub;
sensor_msgs::PointCloud2 ped_pcd, obs_pcd, output;

void ped_clb(const sensor_msgs::PointCloud2ConstPtr& pcd) {
  ped_pcd = *pcd;
  pcl::concatenatePointCloud(obs_pcd, ped_pcd, output);
  pub.publish(output);
}

void obs_clb(const sensor_msgs::PointCloud2ConstPtr& pcd) {
  obs_pcd = *pcd;
  pcl::concatenatePointCloud(obs_pcd, ped_pcd, output);
  pub.publish(output);
}

int main (int argc, char **argv)
{
    ros::init (argc, argv, "merge_ped_obs_pcds");
    ros::NodeHandle nh;

    obs_sub = nh.subscribe(
        "/pedsim_obstacle_sensor/point_cloud_local", 1, ped_clb);

    ped_sub = nh.subscribe(
        "/pedsim_people_sensor/point_cloud_local", 1, obs_clb);

    pub = nh.advertise<sensor_msgs::PointCloud2>("/pedsim_merged_sensor/point_cloud_local", 1);
    ros::spin ();

    return 0;
}