#include "./dbscan.h"
#include "./header.h"
#include "./processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "./processPointClouds.cpp"
#include <cstdlib>
#include <ctime>

struct Color {
    int r;
    int g;
    int b;
};

Color getRandomColor() {
    Color color;
    color.r = rand() % 256; // 0~255
    color.g = rand() % 256;
    color.b = rand() % 256;
    return color;
}

// pcl point type
typedef pcl::PointXYZ PointT;
// cluster point type
typedef pcl::PointXYZI clusterPointT;

// ROI parameter
double zMinROI, zMaxROI, xMinROI, xMaxROI, yMinROI, yMaxROI, yCarROI, xCarROIFront, xCarROIRear;
double xMinBoundingBox, xMaxBoundingBox, yMinBoundingBox, yMaxBoundingBox, zMinBoundingBox, zMaxBoundingBox;
// DBScan parameter
int minPoints;
double epsilon, minClusterSize, maxClusterSize;
// VoxelGrid parameter
float leafSize;
// segmentPlane parameter
int maxIterations;
float distanceThreshold;


// publisher
ros::Publisher pubROI;
ros::Publisher pubCluster;
ros::Publisher pubObjectInfo;
ros::Publisher pubPointInfo;
ros::Publisher pubObjectMarkerArray;
ros::Publisher pubPlaneInfo;

//MSG
lidar_object_detection::ObjectInfo objectInfoMsg;
lidar_object_detection::PointInfo pointInfoMsg; // 이미지에 옮길 bbox 각 꼭짓점 정보


void cfgCallback(lidar_object_detection::objectDetectorStaticConfig &config_tunnel_static, int32_t level) {
    xMinROI = config_tunnel_static.xMinROI;
    xMaxROI = config_tunnel_static.xMaxROI;
    yMinROI = config_tunnel_static.yMinROI;
    yMaxROI = config_tunnel_static.yMaxROI;
    yCarROI = config_tunnel_static.yCarROI;
    xCarROIFront = config_tunnel_static.xCarROIFront;
    xCarROIRear = config_tunnel_static.xCarROIRear;
    zMinROI = config_tunnel_static.zMinROI;
    zMaxROI = config_tunnel_static.zMaxROI;

    minPoints = config_tunnel_static.minPoints;
    epsilon = config_tunnel_static.epsilon;
    minClusterSize = config_tunnel_static.minClusterSize;
    maxClusterSize = config_tunnel_static.maxClusterSize;

    xMinBoundingBox = config_tunnel_static.xMinBoundingBox;
    xMaxBoundingBox = config_tunnel_static.xMaxBoundingBox;
    yMinBoundingBox = config_tunnel_static.yMinBoundingBox;
    yMaxBoundingBox = config_tunnel_static.yMaxBoundingBox;
    zMinBoundingBox = config_tunnel_static.zMinBoundingBox;
    zMaxBoundingBox = config_tunnel_static.zMaxBoundingBox;

    leafSize  = config_tunnel_static.leafSize;

    maxIterations = config_tunnel_static.maxIterations;
    distanceThreshold = config_tunnel_static.distanceThreshold;
}

pcl::PointCloud<PointT>::Ptr ROI (const sensor_msgs::PointCloud2ConstPtr& input) {
    // ... do data processing
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    pcl::fromROSMsg(*input, *cloud); // sensor_msgs -> PointCloud 형변환

    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr center(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr outskirt(new pcl::PointCloud<PointT>);

    // pcl::PointCloud<PointT>::Ptr *retPtr = &cloud_filtered;
    // std::cout << "Loaded : " << cloud->width * cloud->height << '\n';




    // X축 ROI 먼저 적용
    pcl::PassThrough<PointT> filter;
    filter.setInputCloud(cloud);
    filter.setFilterFieldName("x");
    filter.setFilterLimits(xMinROI, xMaxROI);
    filter.setFilterLimitsNegative(false);
    filter.filter(*cloud_filtered);

    // 차량 중앙 부분 제거 (앞뒤 비대칭)
    pcl::PointCloud<PointT>::Ptr cloud_car_filtered(new pcl::PointCloud<PointT>);

    for (const auto& point : cloud_filtered->points) {
        bool inCarArea = false;

        // Y축 차량 영역 확인
        if (std::abs(point.y) <= yCarROI) {
            // X축 차량 영역 확인 (앞뒤 비대칭)
            if ((point.x >= 0 && point.x <= xCarROIFront) ||    // 앞쪽 (양수)
                (point.x < 0 && point.x >= -xCarROIRear)) {      // 뒤쪽 (음수)
                inCarArea = true;
            }
        }

        // 차량 영역이 아닌 점만 추가
        if (!inCarArea) {
            cloud_car_filtered->push_back(point);
        }
    }

    *cloud_filtered = *cloud_car_filtered;

    // Y축 ROI
    filter.setInputCloud(cloud_filtered);
    filter.setFilterFieldName("y");
    filter.setFilterLimits(yMinROI, yMaxROI);
    filter.setFilterLimitsNegative(false);
    filter.filter(*cloud_filtered);

    // Z축 ROI
    filter.setInputCloud(cloud_filtered);
    filter.setFilterFieldName("z");
    filter.setFilterLimits(zMinROI, zMaxROI);
    filter.setFilterLimitsNegative(false);
    filter.filter(*cloud_filtered);

    // 포인트수 출력
    // std::cout << "ROI Filtered :" << cloud_filtered->width * cloud_filtered->height  << '\n';

    sensor_msgs::PointCloud2 roi_raw;
    pcl::toROSMsg(*cloud_filtered, roi_raw);
    roi_raw.header.frame_id = "velodyne";
    roi_raw.header.stamp = ros::Time::now();

    pubROI.publish(roi_raw);

    return cloud_filtered;
}

pcl::PointCloud<PointT>::Ptr segmentPlane(pcl::PointCloud<PointT>::Ptr input) {
    ProcessPointClouds<PointT> pointProcessor;
    std::pair<pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<PointT>::Ptr> segmentCloud = pointProcessor.SegmentPlane(input, maxIterations, distanceThreshold);

    sensor_msgs::PointCloud2 pointCloudSegmentPlane;
    pcl::toROSMsg(*segmentCloud.first, pointCloudSegmentPlane);
    pubPlaneInfo.publish(pointCloudSegmentPlane);

    return segmentCloud.first;
}

pcl::PointCloud<PointT>::Ptr voxelGrid(pcl::PointCloud<PointT>::Ptr input) {
    //Voxel Grid를 이용한 DownSampling
    pcl::VoxelGrid<PointT> vg;    // VoxelGrid 선언
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>); //Filtering 된 Data를 담을 PointCloud 선언
    vg.setInputCloud(input);             // Raw Data 입력
    vg.setLeafSize(leafSize, leafSize, leafSize); // 사이즈를 너무 작게 하면 샘플링 에러 발생
    vg.filter(*cloud_filtered);          // Filtering 된 Data를 cloud PointCloud에 삽입

    // std::cout << "After Voxel Filtered :" << cloud_filtered->width * cloud_filtered->height  << '\n'; 

    return cloud_filtered;
}

void cluster(pcl::PointCloud<PointT>::Ptr input) {
    if (input->empty()) {
        sensor_msgs::PointCloud2 cluster_point;
        pcl::PointCloud<clusterPointT> totalcloud_clustered;
        pcl::toROSMsg(totalcloud_clustered, cluster_point);
        cluster_point.header.frame_id = "velodyne";
        pubCluster.publish(cluster_point);

        objectInfoMsg.objectCounts = 0;
        // pubObjectInfo.publish(objectInfoMsg);
        return;
    }


    //KD-Tree
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    pcl::PointCloud<clusterPointT>::Ptr clusterPtr(new pcl::PointCloud<clusterPointT>);
    tree->setInputCloud(input);

    //Segmentation
    std::vector<pcl::PointIndices> cluster_indices;

    //DBSCAN with Kdtree for accelerating
    DBSCANKdtreeCluster<PointT> dc;
    dc.setCorePointMinPts(minPoints);   //Set minimum number of neighbor points
    dc.setClusterTolerance(epsilon); //Set Epsilon 
    dc.setMinClusterSize(minClusterSize);
    dc.setMaxClusterSize(maxClusterSize);
    dc.setSearchMethod(tree);
    dc.setInputCloud(input);
    dc.extract(cluster_indices);

    pcl::PointCloud<clusterPointT> totalcloud_clustered;
    int cluster_id = 0;

    //각 Cluster 접근
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++, cluster_id++) {
        pcl::PointCloud<clusterPointT> eachcloud_clustered;
        float cluster_counts = cluster_indices.size();

        //각 Cluster내 각 Point 접근
        for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {

            clusterPointT tmp;
            tmp.x = input->points[*pit].x; 
            tmp.y = input->points[*pit].y;
            tmp.z = input->points[*pit].z;
            tmp.intensity = cluster_id % 100; // 상수 : 예상 가능한 cluster 총 개수
            eachcloud_clustered.push_back(tmp);
            totalcloud_clustered.push_back(tmp);
        }

        //minPoint와 maxPoint 받아오기
        clusterPointT minPoint, maxPoint;
        pcl::getMinMax3D(eachcloud_clustered, minPoint, maxPoint);

        objectInfoMsg.lengthX[cluster_id] = maxPoint.x - minPoint.x; // 
        objectInfoMsg.lengthY[cluster_id] = maxPoint.y - minPoint.y; // 
        objectInfoMsg.lengthZ[cluster_id] = maxPoint.z - minPoint.z; // 
        objectInfoMsg.centerX[cluster_id] = (minPoint.x + maxPoint.x)/2; //직육면체 중심 x 좌표
        objectInfoMsg.centerY[cluster_id] = (minPoint.y + maxPoint.y)/2; //직육면체 중심 y 좌표
        objectInfoMsg.centerZ[cluster_id] = (minPoint.z + maxPoint.z)/2; //직육면체 중심 z 좌표

        if (xMinBoundingBox <= objectInfoMsg.lengthX[cluster_id] && objectInfoMsg.lengthX[cluster_id] <= xMaxBoundingBox &&
            yMinBoundingBox <= objectInfoMsg.lengthY[cluster_id] && objectInfoMsg.lengthY[cluster_id] <= yMaxBoundingBox &&
            zMinBoundingBox <= objectInfoMsg.lengthZ[cluster_id] && objectInfoMsg.lengthZ[cluster_id] <= zMaxBoundingBox) {
            if (objectInfoMsg.centerY[cluster_id] >= 0) {
                pointInfoMsg.xMini[cluster_id] = maxPoint.x; 
                pointInfoMsg.yMini[cluster_id] = minPoint.y;
                pointInfoMsg.zMini[cluster_id] = minPoint.z;

                pointInfoMsg.xMaxi[cluster_id] = minPoint.x;
                pointInfoMsg.yMaxi[cluster_id] = maxPoint.y;
                pointInfoMsg.zMaxi[cluster_id] = maxPoint.z;
            }
            else if (objectInfoMsg.centerY[cluster_id] < 0) {
                pointInfoMsg.xMini[cluster_id] = minPoint.x; 
                pointInfoMsg.yMini[cluster_id] = minPoint.y;
                pointInfoMsg.zMini[cluster_id] = minPoint.z;

                pointInfoMsg.xMaxi[cluster_id] = maxPoint.x;
                pointInfoMsg.yMaxi[cluster_id] = maxPoint.y;
                pointInfoMsg.zMaxi[cluster_id] = maxPoint.z;
            }

        }
        else {
            cluster_id--;
        }

    }

    objectInfoMsg.objectCounts = cluster_id;
    pubObjectInfo.publish(objectInfoMsg);

    pointInfoMsg.bboxCounts = cluster_id;
    pubPointInfo.publish(pointInfoMsg);

    sensor_msgs::PointCloud2 cluster_point;
    pcl::toROSMsg(totalcloud_clustered, cluster_point);
    cluster_point.header.frame_id = "velodyne";
    pubCluster.publish(cluster_point);
}

void visualizeObject() {
    visualization_msgs::MarkerArray objectMarkerArray;
    visualization_msgs::Marker objectMarker;

    objectMarker.header.frame_id = "velodyne"; 
    objectMarker.ns = "object_shape";
    objectMarker.type = visualization_msgs::Marker::CUBE;
    objectMarker.action = visualization_msgs::Marker::ADD;

    for (int i = 0; i < objectInfoMsg.objectCounts; i++) {

        if (xMinBoundingBox <= objectInfoMsg.lengthX[i] && objectInfoMsg.lengthX[i] <= xMaxBoundingBox &&
            yMinBoundingBox <= objectInfoMsg.lengthY[i] && objectInfoMsg.lengthY[i] <= yMaxBoundingBox &&
            zMinBoundingBox <= objectInfoMsg.lengthZ[i] && objectInfoMsg.lengthZ[i] <= zMaxBoundingBox) {

            // Set the namespace and id for this marker.  This serves to create a unique ID
            // Any marker sent with the same namespace and id will overwrite the old one
            objectMarker.header.stamp = ros::Time::now();
            objectMarker.id = 100+i; // 

            // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
            objectMarker.pose.position.x = objectInfoMsg.centerX[i];
            objectMarker.pose.position.y = objectInfoMsg.centerY[i];
            objectMarker.pose.position.z = objectInfoMsg.centerZ[i];
            objectMarker.pose.orientation.x = 0.0;
            objectMarker.pose.orientation.y = 0.0;
            objectMarker.pose.orientation.z = 0.0;
            objectMarker.pose.orientation.w = 1.0;

            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            objectMarker.scale.x = objectInfoMsg.lengthX[i];
            objectMarker.scale.y = objectInfoMsg.lengthY[i];
            objectMarker.scale.z = objectInfoMsg.lengthZ[i];

            // Set the color -- be sure to set alpha to something non-zero!
            objectMarker.color.r = 0.0;
            objectMarker.color.g = 1.0;
            objectMarker.color.b = 0.0;
            objectMarker.color.a = 0.5;

            objectMarker.lifetime = ros::Duration(0.1);
            objectMarkerArray.markers.emplace_back(objectMarker);
        }
    }

    // Publish the marker
    pubObjectMarkerArray.publish(objectMarkerArray);
}

void mainCallback(const sensor_msgs::PointCloud2ConstPtr& input) {
    pcl::PointCloud<PointT>::Ptr cloudPtr;

    // main process method
    cloudPtr = ROI(input);
    // cloudPtr = voxelGrid(cloudPtr);
    // cloudPtr = segmentPlane(cloudPtr);
    cluster(cloudPtr);

    // visualize method
    visualizeObject();
}

int main (int argc, char** argv) {
    // Initialize ROS
    ros::init (argc, argv, "lidar_object_detection_static");
    ros::NodeHandle nh;

    dynamic_reconfigure::Server<lidar_object_detection::objectDetectorStaticConfig> server;
    dynamic_reconfigure::Server<lidar_object_detection::objectDetectorStaticConfig>::CallbackType f;

    f = boost::bind(&cfgCallback, _1, _2);
    server.setCallback(f);
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("velodyne_points", 1, mainCallback);

    // Create a ROS publisher for the output point cloud
    pubROI = nh.advertise<sensor_msgs::PointCloud2> ("roi_raw_static", 1);
    pubCluster = nh.advertise<sensor_msgs::PointCloud2>("cluster_static", 1);
    pubObjectInfo = nh.advertise<lidar_object_detection::ObjectInfo>("obstacle_info_static", 1);
    pubPointInfo = nh.advertise<lidar_object_detection::PointInfo>("bbox_point_info_static", 1);
    pubObjectMarkerArray = nh.advertise<visualization_msgs::MarkerArray>("bounding_box_static", 1);
    pubPlaneInfo = nh.advertise<sensor_msgs::PointCloud2> ("plane_static", 1);

    // Spin
    ros::spin();
}