#include <string>
#include <math.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/extract_indices.h>
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
  //dynamic_reconfigure::Server<automow_filtering::automow_filteringConfig> server_;
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
    vehicle_frame_("base_link"),
    min_z(0.1f),
    max_z(0.75f),
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
    //nonobstacles_pub_ = n_.advertise<PointCloud2>("/ground_filter/non_obstacles", 1);
    //server_.setCallback(boost::bind(&GroundFilter::onConfigure, this, _1, _2));
    n_.getParam(string("vehicle_fame"), vehicle_frame_);
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
  pc_out.header.frame_id = "bumblebee_mount_link";
  
  pub.publish(pc_out);
}

void segmentWithRANSAC(PCLPointCloud &pc, PCLPointCloud &ground, PCLPointCloud &nonground) {
  //int max_iterations = 400;
  //float distance_threshold = 0.1;
  //float eps_angle = 0.15;
  // Remove data way below or above the ground
  filterByHeight(pc, min_z, max_z);
  // Set the headers
  ground.header = pc.header;
  nonground.header = pc.header;

  // Create the coefficients and inliers storage
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl:: SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations(max_iterations); // 400
  seg.setDistanceThreshold (distance_threshold);
  seg.setAxis(Eigen::Vector3f(0,0,1));
  seg.setEpsAngle(eps_angle*M_PI/180.0); // 15 degrees (must convert to rads)
  
  // Filter until the cloud is too small or I "find" the ground plane
  pcl::PointCloud<pcl::PointXYZ> filter_pc(pc);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  bool found_ground = false;
  while (filter_pc.size() > 10 && !found_ground) {
    seg.setInputCloud(filter_pc.makeShared());
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
      ROS_WARN("No plane found in the pointcloud!");
      break;
    }
    // if (std::abs(coefficients->values.at(3)) < 0.07) {
    // ROS_INFO("True");
    // } else {
    // ROS_INFO("False");
    // }
    // Get the indices using the extractor
    extract.setInputCloud(filter_pc.makeShared());
    extract.setIndices(inliers);
    extract.setNegative(false);
    pcl::PointCloud<pcl::PointXYZ> cloud_out;
    extract.filter(cloud_out);
    ground += cloud_out;
    if(inliers->indices.size() != filter_pc.size()){
      extract.setNegative(true);
      pcl::PointCloud<pcl::PointXYZ> cloud_out2;
      extract.filter(cloud_out2);
      nonground += cloud_out2;
      filter_pc = cloud_out2;
    }
    found_ground = true;
  }
}

void processPC(PCLPointCloud &pc) {
  //float min_z = 0.1;
  //float max_z = 0.75;        

  filterByHeight(pc, min_z, max_z);
  PCLPointCloud ground, nonground;
  PCL2 nongroundPCL2;
  segmentWithRANSAC(pc, ground, nonground);

  // Krystian added this line to check if we even get anything. We do, but theres something wrong with transform.
  pcl::toPCLPointCloud2(pc,nongroundPCL2); // convert from pcl::PointCloud to pcl::PCLPointCloud2

  // pcl::toPCLPointCloud2(nonground,nongroundPCL2); // convert from pcl::PointCloud to pcl::PCLPointCloud2
  // Publish the nonground pc to obstacles
  publishPCLPointCloud(nongroundPCL2, obstacles_pub_);
  // Publish ground to the nonobstacles
  //publishPCLPointCloud(ground, nonobstacles_pub_);
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

  // Subscribe to the messages we need
  //ros::Subscriber pcl2Sub = nh.subscribe("stereo_camera/points2", 10, &pcl2Callback);


  ros::Rate rate(10);
  while(ros::ok())
  {

    ros::spinOnce();

    rate.sleep();

  }

  return 0;
}
