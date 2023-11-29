#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/filters/voxel_grid.h>

#include <pcl_ros/transforms.hpp>  // PCL 변환 관련 헤더 (ROS2에서 사용 가능)
#include <pcl_conversions/pcl_conversions.h>  // PCL 메시지 변환을 위한 헤더

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>  // TF2 버퍼를 사용하기 위한 헤더

#include <nav_msgs/msg/path.hpp>


using namespace std;
typedef pcl::PointXYZI PointType;
std::vector<std::vector<int>> pixel_classes;

int pixel_class_value;

pcl::PointCloud<PointType>::Ptr depthCloud(new pcl::PointCloud<PointType>());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr outside_image_RGB_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

// distortion_parameters:
  double k1 = -0.07253575;
  double k2 = 0.06272369;  
  double k3 = 0.0;
  double p1 = -0.00102158;
  double p2 = 0.00360962;  
// projection_parameters: 
  double fx = 539.12184959;
  double fy = 537.93396235;  
  double cx = 477.46520972; 
  double cy = 270.09177125; 


Eigen::Matrix4f camera_lidar_tf;

cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
cv::Mat dist_coeffs = (cv::Mat_<double>(5,1) << k1, k2, p1, p2, k3);
double L_C_TX;
double L_C_TY;
double L_C_TZ;
double L_C_RX;
double L_C_RY;
double L_C_RZ;
Eigen::Affine3f transOffset;
// Eigen::Affine3f transOffset = pcl::getTransformation(L_C_TX, L_C_TY, L_C_TZ, L_C_RX, L_C_RY, L_C_RZ);




void publishRGBCloud(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& thisPub,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr thisCloud,
    rclcpp::Time thisStamp,
    std::string thisFrame)
{
    if (thisPub->get_subscription_count() == 0)
        return;

    sensor_msgs::msg::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    thisPub->publish(tempCloud);
}

void setCameraLidarTf()  
{
  pixel_classes.resize(540, std::vector<int>(960));

  L_C_TX = 0.143;
  L_C_TY = -0.30;
  L_C_TZ = -0.012;
  L_C_RX = 0.0;
  L_C_RY = -1.5707963;
  L_C_RZ = 1.5707963;

  transOffset = pcl::getTransformation(L_C_TX, L_C_TY, L_C_TZ, L_C_RX, L_C_RY, L_C_RZ);


}

cv::Point2f project3DTo2D(float x, float y, float z, const cv::Mat& camera_matrix)
{
  cv::Point2f pixel;
  pixel.x = (x * camera_matrix.at<double>(0, 0) / z) + camera_matrix.at<double>(0, 2);
  pixel.y = (y * camera_matrix.at<double>(1, 1) / z) + camera_matrix.at<double>(1, 2);
  return pixel;
}


class SubscribeAndPublish : public rclcpp::Node
{
public:
  SubscribeAndPublish() : Node("subscribe_and_publish")
  {
    pub_1 = this->create_publisher<sensor_msgs::msg::PointCloud2>("transformed_velodyne", 10);
    path_pub = this->create_publisher<nav_msgs::msg::Path>("path", 10);

    sub_img = this->create_subscription<sensor_msgs::msg::Image>(
        "/yonsei_rs/semseg/prediction", 10, std::bind(&SubscribeAndPublish::imageCallback, this, std::placeholders::_1));
    sub_cloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/velodyne_points", 10, std::bind(&SubscribeAndPublish::pointCloudCallback, this, std::placeholders::_1));

    tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    int image_height = msg->height;
    int image_width = msg->width;

    for (int row = 0; row < image_height; ++row)
    {
        for (int col = 0; col < image_width; ++col)
        {
            // setCameraLidarTf 에서 pixel class resize도 image width, height에 맞춰서 해줌
            pixel_classes[row][col] = static_cast<int>(msg->data[row * image_width + col]);
        }
    }
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
  {
    outside_image_RGB_cloud->clear();
    pcl::PointCloud<PointType>::Ptr laser_cloud_in(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*cloud_msg, *laser_cloud_in);

    cout << "laser_cloud_in_size : " << laser_cloud_in->size() << endl;
    // 2. downsample new cloud (save memory)
    pcl::PointCloud<PointType>::Ptr laser_cloud_in_ds(new pcl::PointCloud<PointType>());
    static pcl::VoxelGrid<PointType> downSizeFilter;
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.setInputCloud(laser_cloud_in);
    downSizeFilter.filter(*laser_cloud_in_ds);
    *laser_cloud_in = *laser_cloud_in_ds; // downsize filter 된 point cloud : laser_cloud_in_ds, 이걸 다시 laser_cloud_in에 넣음
    cout << "downsampled laser_cloud_in_size : " << laser_cloud_in->size() << endl;

    // 3. filter lidar points (only keep points in camera view)
    pcl::PointCloud<PointType>::Ptr laser_cloud_in_filter(new pcl::PointCloud<PointType>());
    for (int i = 0; i < (int)laser_cloud_in->size(); ++i)
    {
        PointType p = laser_cloud_in->points[i];
        if (p.x >= 0 && abs(p.y / p.x) <= 10 && abs(p.z / p.x) <= 10)
            {
                laser_cloud_in_filter->push_back(p);
            }
    }   // laser_cloud_in에서 Camera view 안에 있는 포인트 클라우드들만 남김 laser_cloud_in_filter
    *laser_cloud_in = *laser_cloud_in_filter;   // laser_cloud_in_filter를 다시 laser_cloud_in에 넣음
    cout << "filtered laser_cloud_in_size : " << laser_cloud_in->size() << endl;



    // Transform the point cloud to the camera frame
    pcl::PointCloud<PointType>::Ptr transformed_cloud(new pcl::PointCloud<PointType>);
    // pcl::PointCloud<PointType>::Ptr inv_transformed_cloud(new pcl::PointCloud<PointType>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inv_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    // Eigen::Affine3f transOffset = pcl::getTransformation(L_C_TX, L_C_TY, L_C_TZ, L_C_RX, L_C_RY, L_C_RZ);
    pcl::transformPointCloud(*laser_cloud_in, *transformed_cloud, transOffset);
    cout << "transformed_cloud_size : " << transformed_cloud->size() << endl;
     
    pcl::PointXYZRGB point_;

    // cout << " pixel_class_value : " << pixel_class_value << endl;
    // cout << "transformed_cloud_size : " << transformed_cloud->size() << endl;
    // Project each point in the point cloud to the image plane
    for (int i = 0; i < (int)transformed_cloud->size(); ++i)
    {     

      point_.x = transformed_cloud->points[i].x;
      point_.y = transformed_cloud->points[i].y;
      point_.z = transformed_cloud->points[i].z;
      // std::cout << "for loop start" << std::endl ;

      // Using the camera matrix, project 3D points to 2D image plane
      cv::Point2f pixel = project3DTo2D(point_.x, point_.y, point_.z, camera_matrix);
      if ((int)pixel.x < 0 || (int)pixel.x >= 960 || (int)pixel.y < 0 || (int)pixel.y >= 540)
      {
        // std::cout << "FOV" << std::endl ;
        // point_.r = 255;  // Set the red channel value
        // point_.g = 0;    // Set the green channel value
        // point_.b = 0;    // Set the blue channel value
        // outside_image_RGB_cloud->push_back(point_); 

      }
      else
      {
        // std::cout << "FOV_out" << std::endl ;
        // std::cout << (int)pixel.x << " , " << (int)pixel.y << std::endl ;
        pixel_class_value  = pixel_classes[(int)pixel.y][(int)pixel.x];
        // pixel class value debug
        // std::cout << pixel_class_value << std::endl ;
        // point_.r = 0;  // Set the red channel value
        // point_.g = 0;    // Set the green channel value
        // point_.b = 255;    // Set the blue channel value
        if(pixel_class_value == 0) // 분홍
        {
            point_.r = 255;  // Set the red channel value
            point_.g = 191;    // Set the green channel value
            point_.b = 190;    // Set the blue channel value
        }
        else if(pixel_class_value == 1) // 회색
        {
            point_.r = 200;  // Set the red channel value
            point_.g = 200;    // Set the green channel value
            point_.b = 200;    // Set the blue channel value
        }
        else if(pixel_class_value == 2) // 노랑
        {
            point_.r = 245;  // Set the red channel value
            point_.g = 243;    // Set the green channel value
            point_.b = 131;    // Set the blue channel value
        }
        else if(pixel_class_value == 3) // 탁한 분홍(?)
        {
            point_.r = 190;  // Set the red channel value
            point_.g = 153;    // Set the green channel value
            point_.b = 153;    // Set the blue channel value
        }
        else if(pixel_class_value == 4) // 파랑
        {
            point_.r = 0;  // Set the red channel value
            point_.g = 0;    // Set the green channel value
            point_.b = 255;    // Set the blue channel value
        }
        else if(pixel_class_value == 5) // 하늘
        {
            point_.r = 0;  // Set the red channel value
            point_.g = 224;    // Set the green channel value
            point_.b = 228;    // Set the blue channel value
        }
        else if(pixel_class_value == 6) // 주황
        {
            point_.r = 250;  // Set the red channel value
            point_.g = 170;    // Set the green channel value
            point_.b = 31;    // Set the blue channel value
        }
        else 
        {
            point_.r = 255;  // Set the red channel value
            point_.g = 0;    // Set the green channel value
            point_.b = 0;    // Set the blue channel value
        }
        outside_image_RGB_cloud->push_back(point_); 
      }

      // Now pixel corresponds to the point in the image
    }

        // L_C_TX = 0.143;     
        // L_C_TY = -0.30;
        // L_C_TZ = -0.012;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_outside_image_RGB_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    Eigen::Affine3f transOffset = pcl::getTransformation(-L_C_TX,-L_C_TY,-L_C_TZ,M_PI/2,-M_PI,M_PI/2);
    pcl::transformPointCloud(*outside_image_RGB_cloud, *transformed_outside_image_RGB_cloud, transOffset);

    cout << "outsinde_images_RGB_size : " << outside_image_RGB_cloud->size() << endl;
    publishRGBCloud(pub_1, transformed_outside_image_RGB_cloud, rclcpp::Time(cloud_msg->header.stamp), "velodyne");
    outside_image_RGB_cloud->clear();

  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_1;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    setCameraLidarTf();
    auto node = std::make_shared<SubscribeAndPublish>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}