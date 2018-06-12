#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/impl/point_types.hpp>
#include <sensor_msgs/ChannelFloat32.h>
#include <pcl/common/projection_matrix.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pointcloud_msgs/PointCloud2_Segments.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


ros::Publisher pub;

float x, y, z ;
double clusterTolerance, distanceThreshold;

int maxIterations;
int minClusterSize, maxClusterSize;


void cloud_callback (const pointcloud_msgs::PointCloud2_Segments& c_)
{

    pcl::PCLPointCloud2 cloud2;


    pcl_conversions::toPCL( c_.clusters[0] , cloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::fromPCLPointCloud2(cloud2, *cloud);

    //std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);


    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (clusterTolerance);// 2cm
    ec.setMinClusterSize (minClusterSize); //100
    ec.setMaxClusterSize (maxClusterSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    int j = 0;

    pointcloud_msgs::PointCloud2_Segments msg_;



    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            cloud_cluster->points.push_back (cloud->points[*pit]); 
        }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        j++;

        sensor_msgs::PointCloud2 msgout;
        pcl::PCLPointCloud2 cloud2;
        pcl::toPCLPointCloud2(*cloud_cluster, cloud2);

        pcl_conversions::fromPCL(cloud2, msgout);

        msg_.clusters.push_back(msgout);
        msg_.factor = c_.factor;
        msg_.overlap = c_.overlap;
        msg_.first_stamp = c_.first_stamp; 
        msg_.num_scans = c_.num_scans ;


    }
    pub.publish(msg_);

}

int main (int argc, char** argv){
    ros::init (argc, argv, "pointcloud2_clustering");
    ros::NodeHandle n_;


    n_.param("pointcloud2_clustering/setMaxIterations", maxIterations, 100);
    n_.param("pointcloud2_clustering/setDistanceThreshold", distanceThreshold, 0.01);
    n_.param("pointcloud2_clustering/setClusterTolerance", clusterTolerance, 0.4);
    n_.param("pointcloud2_clustering/setMinClusterSize", minClusterSize, 10);
    n_.param("pointcloud2_clustering/setMaxClusterSize", maxClusterSize, 25000);

    std::string topic;
    std::string out_topic;
    n_.param("pointcloud2_clustering/cloud_topic", topic, std::string("laserscan_stacker/scans"));
    n_.param("pointcloud2_clustering/output_cloud_topic", out_topic, std::string("pointcloud2_clustering/clusters"));

    ros::Subscriber sub = n_.subscribe (topic, 1, cloud_callback);

    pub = n_.advertise<pointcloud_msgs::PointCloud2_Segments> (out_topic, 1);

    ros::spin ();
}