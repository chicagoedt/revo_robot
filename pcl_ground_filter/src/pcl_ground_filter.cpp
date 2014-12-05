#include <string>
#include <math.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/passthrough.h>

using std::string;
using sensor_msgs::PointCloud;
using sensor_msgs::PointCloud2;

namespace my_modes {
typedef enum {
NONE = 0,
SACMODEL_PERPENDICULAR_PLANE = 1
} modes_t;
}
using namespace my_modes;

typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;
typedef pcl::PCLPointCloud2            PCL2;

class GroundFilter {
private:
  ros::NodeHandle n_;
  tf::TransformListener listener_;
  ros::Subscriber pcl2_sub_;
  ros::Publisher obstacles_pub_;
  ros::Publisher nonobstacles_pub_;
  int mode_;
  string vehicle_frame_;
  double min_z, max_z;
  PCLPointCloud last_pc;
  bool paused, first_cloud, z_filter, downsample;
  double resolution;
  int max_iterations;
  double distance_threshold, eps_angle;
public:
  GroundFilter(ros::NodeHandle n) :
    n_(n),
    mode_(SACMODEL_PERPENDICULAR_PLANE),
    vehicle_frame_("odom"),
    min_z(0.0f),
    max_z(1.2f),
    paused(false),
    first_cloud(true),
    z_filter(true),
    downsample(false),
    resolution(0.001f),
    max_iterations(400),
    distance_threshold(0.1),
    eps_angle(15.0)
  {
    pcl2_sub_ = n_.subscribe("/stereo_camera/points2", 10, &GroundFilter::pcl2Callback, this);
    obstacles_pub_ = n_.advertise<PointCloud2>("/ground_filter/obstacles", 1);
    n_.getParam(string("vehicle_frame"), vehicle_frame_);
    n_.getParam("resolution", resolution);
    n_.getParam("min_z", min_z);
    n_.getParam("max_z", max_z);
    n_.getParam("filtering_mode", mode_);
    n_.getParam("max_iterations", max_iterations);
    n_.getParam("distance_threshold", distance_threshold);
    n_.getParam("eps_angle", eps_angle);
  }

void publishPCLPointCloud(PCL2 &pc, ros::Publisher &pub) {
  PointCloud2 pc_out;
  pcl_conversions::fromPCL(pc, pc_out);
  pc_out.header.frame_id = vehicle_frame_;
  
  pub.publish(pc_out);
}


void processPC(PCLPointCloud &pc) {       
  // Remove points with z-values above max_z and below min_z
  filterByHeight(pc, min_z, max_z);
  PCL2 nongroundPCL2;

  pcl::toPCLPointCloud2(pc,nongroundPCL2); // convert from pcl::PointCloud to pcl::PCLPointCloud2

  // Publish the nonground pc to obstacles
  publishPCLPointCloud(nongroundPCL2, obstacles_pub_);
}
/* Takes in sensor_msgs::PointCloud2, converts it to sensor_msgs::PointCloud2, performs transform,
// Then re-converts it back to a sensor_msgs::PointCloud2 object, then converts it into a 
// pcl::PCLPointCloud2 object then to a pcl::PointCloud object. Finally it processes the pc
*/
void pcl2Callback(const sensor_msgs::PointCloud2ConstPtr& pc2_in){//void pcl2Callback(const sensor_msgs::PointCloud2ConstPtr& cloud)

  sensor_msgs::PointCloud pc_tf;
  sensor_msgs::PointCloud pc_in;
  sensor_msgs::convertPointCloud2ToPointCloud(*pc2_in, pc_in);
  // First transform the point cloud to the vehicle frame
  try {
    listener_.waitForTransform(vehicle_frame_, pc_in.header.frame_id, pc_in.header.stamp, ros::Duration(0.2));
    listener_.transformPointCloud(vehicle_frame_, pc_in, pc_tf);
  }
  catch (tf::TransformException& e) {
    ROS_WARN("Failed to transform PointCloud: %s", e.what());
    return;
  }
  sensor_msgs::PointCloud2 pc2_intermediate;
  sensor_msgs::convertPointCloudToPointCloud2(pc_tf, pc2_intermediate);

  PCL2 pcl_pc;
  pcl_conversions::toPCL(pc2_intermediate,pcl_pc); // convert from sensor_msgs::PointCloud2 to pcl::PCLPointCloud2
  PCLPointCloud pc;
  pcl::fromPCLPointCloud2(pcl_pc, pc); // convert from pcl::PCLPointCloud2 to pcl::PointCloud

  last_pc = pc;


  processPC(pc);
}

void filterByHeight(PCLPointCloud &pc, double &min, double &max) {
  // set up filter for height range, also removes NANs:
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setFilterFieldName("z");
  pass.setFilterLimits(min, max);
  // Filter the pc
  pass.setInputCloud(pc.makeShared());
  pass.filter(pc);
}
};

 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_ground_filter");

  ros::Time::init();

  ros::NodeHandle nh;
  GroundFilter gf(nh);

  ros::Rate rate(10);
  while(ros::ok())
  {

    ros::spinOnce();

    rate.sleep();

  }

  return 0;
}
