/**
 * @file publish_pcl.cpp
 * @author your name (you@domain.com)
 * @brief ROS node to convert all pixels from an incoming image into an intersection with a ground plane and publish as a point cloud object
 * @version 0.1
 * @date 2021-07-28
 * 
 * @copyright Copyright (c) 2021
 * 
 */

// Imports from Image Geometry tutorial
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>


// own imports
#include <sensor_msgs/PointCloud.h>

class PCL_Converter
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::CameraSubscriber sub_;
    ros::Publisher pcl_pub_;
    tf::TransformListener tf_listener_;
    image_geometry::PinholeCameraModel cam_model_;
    std::string world_frame;

public:
    PCL_Converter(/* args */);
    void imageCb(const sensor_msgs::ImageConstPtr&, const sensor_msgs::CameraInfoConstPtr&);
    std::vector<cv::Point3d> convertPixelsToRays(int, int);
    cv::Point2d homogenisePoint(int, int, double, double, double, double);

    cv::Point3f transformPointInverse(const cv::Point3f, const cv::Matx44f);
};

// Member functions
PCL_Converter::PCL_Converter(/* args */) : it_(nh_)
{
    std::string image_topic;
    std::string pcl_topic;
    image_topic = nh_.resolveName("image");
    nh_.param("pcl_topic", pcl_topic, std::string("pcl_plane"));
    nh_.param("world_frame", this->world_frame, std::string("map"));
    sub_ = it_.subscribeCamera(image_topic, 1, &PCL_Converter::imageCb, this);
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud>("pcl", 1);

    // world plane points
    // TODO: see if this throws an error
    cv::Vec3d plane_pt1, plane_pt2, plane_pt3;
    nh_.param("plane/point1", plane_pt1, {0.f, 0.f, 0.f});
    nh_.param("plane/point1", plane_pt2, {0.f, 1.f, 0.f});
    nh_.param("plane/point1", plane_pt3, {1.f, 0.f, 0.f});

    // World plane vector
    std::vector<cv::Point3d> plane;
    plane.push_back(cv::Point3f(plane_pt1));
    plane.push_back(cv::Point3f(plane_pt2));
    plane.push_back(cv::Point3f(plane_pt3));
}

// Image Callback
void PCL_Converter::imageCb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg){

    // Convert image
    cv::Mat image;
    int width, height;
    cv_bridge::CvImagePtr input_bridge;
    try{
        input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        image = input_bridge->image;
        // TODO: check whether these two are the right way around
        width = image.size[0];
        height = image.size[1];
    } catch(cv_bridge::Exception& ex){
        ROS_ERROR("Failed to convert image");
        return;
    }
    
    // create a camera model from the camera info - apparently cheap
    cam_model_.fromCameraInfo(info_msg);

    // look up the transform of the camera in the world frame
    tf::StampedTransform transform;
    try{
        ros::Time image_time = info_msg->header.stamp;
        ros::Duration timeout(1.0 / 30);
        tf_listener_.waitForTransform(cam_model_.tfFrame(), this->world_frame, image_time, timeout);
        tf_listener_.lookupTransform(cam_model_.tfFrame(), this->world_frame, image_time, transform);
    } catch(tf::TransformException& ex){
        ROS_WARN("Failed to get transform from %s to %s", cam_model_.tfFrame(), this->world_frame);
        return;
    }

    // Get a ray for each pixel
    // TODO: Preallocate vector for speed
    // TODO: theoretically, this can be done once
    std::vector<cv::Point3d> rays = convertPixelsToRays(width, height);

    // use the transform and the points of the plane to convert between coordinate frames

    // calculate a new plane from the points

    // Calculate the intersection of plane and points

    // turn into pointcloud message

}

// Converting single pixel to rays
std::vector<cv::Point3d> PCL_Converter::convertPixelsToRays(int width, int height){
    std::vector<cv::Point3d> output_vec;
    double cx = this->cam_model_.cx();
    double cy = this->cam_model_.cy();
    double fx = this->cam_model_.fx();
    double fy = this->cam_model_.fy();

    for (int x=0; x<height; x++){
        for (int y=0; y<width; y++){
            cv::Point2d uv_rect = homogenisePoint(x, y, cx, cy, fx, fy);
            cv::Point3d ray = this->cam_model_.projectPixelTo3dRay(uv_rect);
            output_vec.push_back(ray);
        }
    }
    return output_vec;
}

// Converting image coordinate to rectified Point
cv::Point2d PCL_Converter::homogenisePoint(int x, int y, double cx, double cy, double fx, double fy){
    cv::Point2d rect;
    rect.x = (x - cx) / fx;
    rect.y = (y - cy) / fy;
    return rect;
}

// Transforming a point into another coordinate frame 
cv::Point3f PCL_Converter::transformPointInverse(const cv::Point3f pt, const cv::Matx44f T){
    cv::Mat homogenous_3dPt = (cv::Mat_<double>(4, 1) << pt.x, pt.y, pt.z, 1.0);
    cv::Mat tfPt = T * homogenous_3dPt;
    return cv::Point3f(tfPt.at<double>(0),  tfPt.at<double>(1), tfPt.at<double>(2));
}

// Computing the intersection of a plane and a ray


int main(int argc, char **argv){
    ros::init(argc, argv, "test_node");
    PCL_Converter convert();
    ros::spin();
    return 0;
}