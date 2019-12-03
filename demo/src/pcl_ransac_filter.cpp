#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

  /** \brief Get the distance from a point to a plane (signed) defined by ax+by+cz+d=0
    * \param p a point
    * \param plane_coefficients the normalized coefficients (a, b, c, d) of a plane
    * \ingroup sample_consensus
    */
template <typename Point> inline double
pointToPlaneDistanceSigned (const Point &p, const Eigen::Vector4f &plane_coefficients) {
    eturn ( plane_coefficients[0] * p.x + plane_coefficients[1] * p.y + plane_coefficients[2] * p.z + plane_coefficients[3] );
}

class FilterNode {
      FilterNode() {
        //Topic you want to publish
        pub_ = n_.advertise<sensor_msgs::PointCloud2>("/filtered_cloud", 1);

        //Topic you want to subscribe
        sub_ = n_.subscribe("/kinect2/sd/points", 1, &FilterNode::pointCloudCb, this);
    }

    void pointCloudCb(const sensor_msgs::PointCloud2::ConstPtr &msg) {
        // Convert to pcl point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_msg (new pcl::PointCloud<pcl::PointXYZ>);

        pcl::fromROSMsg(*msg,*cloud_msg);
        ROS_DEBUG("%s: new ponitcloud (%i,%i)(%zu)",_name.c_str(),cloud_msg->width,cloud_msg->height,cloud_msg->size());

        // Filter cloud depth filter
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_msg);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits(0.001,10000);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pass.filter (*cloud);


        //DECIMATE CLOUD
        // Create the filtering object
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>::Ptr );
        pcl::VoxelGrid<pcl::PointCloud<pcl::PointXYZ> sor;
        sor.setInputCloud (cloud);
        sor.setLeafSize (0.01f, 0.01f, 0.01f);
        sor.filter (*cloud_filtered);
        //DECIMATE CLOUD


        // Get segmentation ready
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold(_max_distance);

        // Create pointcloud to publish inliers
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pub(new pcl::PointCloud<pcl::PointXYZRGB>);

        // Fit a plane
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        // Display inform
        ROS_INFO("%s: fitted plane %i: %fx%s%fy%s%fz%s%f=0 ",
                _name.c_str(),
                coefficients->values[0],(coefficients->values[1]>=0?"+":""),
                coefficients->values[1],(coefficients->values[2]>=0?"+":""),
                coefficients->values[2],(coefficients->values[3]>=0?"+":""),
                coefficients->values[3],);

        //USE PLANE TO FILTER CLOUD
        //IF CLOSER TO PLANE KEEP ELSE PRUNE
        for (int i=0;i<inliers->cloud.size();i++){
            pt_color = cloud->points[i];
            if(pointToPlaneDistanceSigned(pt_color, coefficients) > 0) {
                pcl::PointXYZRGB pt_color = cloud->points[inliers->indices[i]];
                cloud_pub->points.push_back(pt_color);}
        }
     

        // Publish points
        sensor_msgs::PointCloud2 cloud_publish;
        pcl::toROSMsg(*cloud_pub,cloud_publish);
        cloud_publish.header = msg->header;
        pub_.publish(cloud_publish);
    }

    private:
        os::NodeHandle n_; 
        ros::Publisher pub_;
        os::Subscriber sub_;
}

int main(int argc, char **argv) {
    /Initiate ROS
    ros::init(argc, argv, "rasnsac_filter");

    //Create an object of class SubscribeAndPublish that will take care of everything
    FilterNode filter;

    ros::spin();

    return 0;
}