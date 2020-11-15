#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/pcl_search.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <visualization_msgs/MarkerArray.h>
#include <vision_msgs/Detection3DArray.h>
#include "geometry_msgs/Point.h"
#include <vision_msgs/Detection3D.h>
#include <vision_msgs/BoundingBox3D.h>

ros::Publisher pub;

void messageReceived(const sensor_msgs::PointCloud2ConstPtr& msg){
	/////////////////////////
	// GROUND SEGMENTATION //
	/////////////////////////
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segmented (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::ModelCoefficients coefficients;
	// Transforms ROS message /point_cloud to PCL cloud
	pcl::fromROSMsg(*msg,cloud);
  	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  	// Create the segmentation object
  	pcl::SACSegmentation<pcl::PointXYZ> seg;
  	// Optional
  	seg.setOptimizeCoefficients (true);
  	// Mandatory
  	seg.setModelType (pcl::SACMODEL_PLANE);
  	seg.setMethodType (pcl::SAC_RANSAC);
  	seg.setDistanceThreshold (0.3);
  	seg.setInputCloud (cloud.makeShared());
  	seg.segment (*inliers, coefficients);

	////////////////////////
	// CLUSTER EXTRACTION //
	////////////////////////
	// Extract inliers
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud.makeShared());
	extract.setIndices(inliers);
	// Don't keep the ground
	extract.setNegative(true);
	// Segmented cloud 
	extract.filter(*cloud_segmented);

  	// Creating the KdTree object for the search method of the extraction
  	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  	tree->setInputCloud (cloud_segmented);

  	std::vector<pcl::PointIndices> cluster_indices;
  	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  	ec.setClusterTolerance (0.5);
  	ec.setMinClusterSize (10);
  	ec.setMaxClusterSize (25000);
  	ec.setSearchMethod (tree);
 	ec.setInputCloud (cloud_segmented);
  	ec.extract (cluster_indices);

	vision_msgs::Detection3DArray detections;
	detections.header = msg->header;
  	
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
                //Create a cloud for the individual cluster
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

                for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
                {
                        cloud_cluster->points.push_back (cloud_segmented->points[*pit]);

                }
		/////////////////////
		// POST PROCESSING //
		/////////////////////
                // Compute the size of each box
                Eigen::Vector4f min;
		Eigen::Vector4f max;
                pcl::getMinMax3D (*cloud_cluster, min, max);

                //Compute the location of the centroid of each box
                Eigen::Vector4f centroid;
                pcl::compute3DCentroid (*cloud_cluster, centroid);

                // Now assign the parameters of the box to an object called
                // detection. This is the information that we are going to publish
                vision_msgs::Detection3D detection;
                detection.header = msg->header;
                detection.bbox.center.position.x = centroid[0];
                detection.bbox.center.position.y = centroid[1];
                detection.bbox.center.position.z = centroid[2];
                detection.bbox.size.x =  (max[0]-min[0]);
                detection.bbox.size.y =  (max[1]-min[1]);
                detection.bbox.size.z =  (max[2]-min[2]);
                detections.detections.push_back(detection);

               // publish detections
                pub.publish(detections);
        }
	std::cout << "haha publisher goes brrrr" << std::endl;
}	

int main(int argc, char **argv){
	ros::init(argc, argv, "pcl_solution_node");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("/point_cloud", 1000, &messageReceived);
	pub = nh.advertise<vision_msgs::Detection3DArray>("/pcl_solution_node/detections",1000);
	ros::spin();
}
