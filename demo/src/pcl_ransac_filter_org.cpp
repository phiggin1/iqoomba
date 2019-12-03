

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


    // Iterate inliers
    double mean_error(0);
    double max_error(0);
    double min_error(100000);
    std::vector<double> err;
    for (int i=0;i<inliers->indices.size();i++){
        // Get Point
        pcl::PointXYZ pt = cloud->points[inliers->indices[i]];

        // Compute distance
        double d = point2planedistnace(pt,coefficients)*1000;// mm
        err.push_back(d);

        // Update statistics
        mean_error += d;
        if (d>max_error) max_error = d;
        if (d<min_error) min_error = d;

    }
    mean_error/=inliers->indices.size();

    // Compute Standard deviation
    ColorMap cm(min_error,max_error);
    double sigma(0);
    for (int i=0;i<inliers->indices.size();i++){
        sigma += pow(err[i] - mean_error,2);

        // Get Point
        pcl::PointXYZ pt = cloud->points[inliers->indices[i]];

        // Copy point to noew cloud
        pcl::PointXYZRGB pt_color;
        pt_color.x = pt.x;
        pt_color.y = pt.y;
        pt_color.z = pt.z;
        uint32_t rgb;
        if (_color_pc_with_error)
            rgb = cm.getColor(err[i]);
        else
            rgb = colors[n_planes].getColor();
        pt_color.rgb = *reinterpret_cast<float*>(&rgb);
        cloud_pub->points.push_back(pt_color);

    }
    sigma = sqrt(sigma/inliers->indices.size());

    // Extract inliers
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    pcl::PointCloud<pcl::PointXYZ> cloudF;
    extract.filter(cloudF);
    cloud->swap(cloudF);

    // Display infor
    ROS_INFO("%s: fitted plane %i: %fx%s%fy%s%fz%s%f=0 (inliers: %zu/%i)",
            _name.c_str(),n_planes,
            coefficients->values[0],(coefficients->values[1]>=0?"+":""),
            coefficients->values[1],(coefficients->values[2]>=0?"+":""),
            coefficients->values[2],(coefficients->values[3]>=0?"+":""),
            coefficients->values[3],
            inliers->indices.size(),original_size);
    ROS_INFO("%s: mean error: %f(mm), standard deviation: %f (mm), max error: %f(mm)",_name.c_str(),mean_error,sigma,max_error);
    ROS_INFO("%s: poitns left in cloud %i",_name.c_str(),cloud->width*cloud->height);

    // Publish points
    sensor_msgs::PointCloud2 cloud_publish;
    pcl::toROSMsg(*cloud_pub,cloud_publish);
    cloud_publish.header = msg->header;
    _pub_inliers.publish(cloud_publish);
}