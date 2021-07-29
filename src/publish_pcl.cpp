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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/TransformStamped.h>     // should already have QUaternion by inheritance
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
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tf_listener_;
    image_geometry::PinholeCameraModel cam_model_;
    std::string world_frame;
    std::vector<cv::Point3d> plane;

public:
    PCL_Converter(/* args */);
    void imageCb(const sensor_msgs::ImageConstPtr&, const sensor_msgs::CameraInfoConstPtr&);
    std::vector<cv::Point3d> convertPixelsToRays(int, int);
    cv::Point2d homogenisePoint(int, int, double, double, double, double);

    cv::Point3f transformPointInverse(const cv::Point3f, const cv::Matx44f);
    void QtoRot(const geometry_msgs::Quaternion, cv::Mat&);
    void TransformToT(const geometry_msgs::TransformStamped, cv::Mat&);
    std::vector<cv::Point3d> convertPts(const std::vector<cv::Point3d>,const cv::Mat);
    void calculatePlane(std::vector<cv::Point3d>, std::vector<float>&);
    void linesPlaneIntersection(std::vector<cv::Point3d> rays, std::vector<float> plane, std::vector<geometry_msgs::Point32> &intersections, cv::Point3d pointOnPlane);
    void linePlaneIntersection(cv::Point3f& contact, cv::Point3d ray, cv::Point3d rayOrigin, cv::Point3d normal, cv::Point3d coord);
};

// Member functions
PCL_Converter::PCL_Converter() : it_(nh_), tf_listener_(tfBuffer)
{
    std::string image_topic;
    std::string pcl_topic;
    image_topic = nh_.resolveName("hbv_1615/image_color");
    nh_.param("pcl_topic", pcl_topic, std::string("pcl_plane"));
    nh_.param("world_frame", this->world_frame, std::string("map"));
    sub_ = it_.subscribeCamera(image_topic, 1, &PCL_Converter::imageCb, this);
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud>("pcl", 1);

    // world plane points
    // TODO: see if this throws an error
    cv::Vec3d plane_pt1, plane_pt2, plane_pt3;
    std::vector<float> pt1, pt2, pt3;
    nh_.param("plane/point1", pt1, {0.f, 0.f, 0.f});
    nh_.param("plane/point1", pt2, {0.f, 1.f, 0.f});
    nh_.param("plane/point1", pt3, {1.f, 0.f, 0.f});

    // World plane vector
    plane.push_back(cv::Point3f(pt1.at(0), pt1.at(1), pt1.at(2)));
    plane.push_back(cv::Point3f(pt2.at(0), pt2.at(1), pt2.at(2)));
    plane.push_back(cv::Point3f(pt3.at(0), pt3.at(1), pt3.at(2)));
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
        width = image.size[1];
        height = image.size[0];
    } catch(cv_bridge::Exception& ex){
        ROS_ERROR("Failed to convert image");
        return;
    }
    
    // create a camera model from the camera info - apparently cheap
    cam_model_.fromCameraInfo(info_msg);

    // look up the transform of the camera in the world frame
    geometry_msgs::TransformStamped transform;
    try{
        ros::Time image_time = info_msg->header.stamp;
        ros::Duration timeout(1.0 / 30);
        transform = tfBuffer.lookupTransform(cam_model_.tfFrame(), this->world_frame, image_time, timeout);
    } catch(tf2::TransformException& ex){
        ROS_WARN("Failed to get transform from %s to %s", cam_model_.tfFrame().c_str(), this->world_frame.c_str());
        return;
    }

    // Get a ray for each pixel
    // TODO: Preallocate vector for speed
    // TODO: theoretically, this can be done once -> only the 3 points for the plane need to be converted anew
    std::vector<cv::Point3d> rays = convertPixelsToRays(width, height);

    // Convert OpenCV and ROS representations
    cv::Mat T = cv::Mat_<double>(4,4);
    TransformToT(transform, T);

    // Turn the plane points into the new coordinate frame
    std::vector<cv::Point3d> newpts = convertPts(this->plane, T);

    // Calculate the new plane equation
    std::vector<float> planecoeff(4);
    calculatePlane(newpts, planecoeff);

    // Calculate the intersection of plane and rays
    std::vector<geometry_msgs::Point32> planePoints(rays.size());
    linesPlaneIntersection(rays, planecoeff, planePoints, newpts.at(0));

    // turn into pointcloud message
    sensor_msgs::PointCloud msg;
    msg.points = planePoints;
    msg.header = info_msg->header; // TODO: potentially change the frame_id

    this->pcl_pub_.publish(msg);        
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

// Converting a Quaternion into a Rotation Matrix - Alternative
void PCL_Converter::QtoRot(const geometry_msgs::Quaternion q, cv::Mat& rot){
// from: https://automaticaddison.com/how-to-convert-a-quaternion-to-a-rotation-matrix/
    // cv::Mat rot;
    rot.at<double>(0,0) = 2 * (pow(q.w, 2) + pow(q.x, 2)) - 1.0;
    rot.at<double>(0,1) = 2 * ((q.x * q.y) - (q.w * q.z));
    rot.at<double>(0,2) = 2 * ((q.x * q.z) + (q.w * q.y));
    rot.at<double>(1,0) = 2 * ((q.x * q.y) + (q.w * q.z));
    rot.at<double>(1,1) = 2 * (pow(q.w, 2) + pow(q.y, 2)) - 1.0;
    rot.at<double>(1,2) = 2 * ((q.y * q.z) - (q.w * q.x));
    rot.at<double>(2,0) = 2 * ((q.x * q.z) - (q.w * q.y));
    rot.at<double>(2,1) = 2 * ((q.y * q.z) + (q.w * q.x));
    rot.at<double>(2,2) = 2 * (pow(q.w, 2) + pow(q.z, 2)) - 1.0;

    /*
        Alternative
        #include <tf2/LinearMath/Matrix3x3.h>
        tf2::Quaternion q;
        tf2::Matrix3x3 m;
        m.setRotation(q);


    Another alternative here: https://answers.ros.org/question/358188/converting-cvmat-rotation-matrix-to-quaternion/
    */
}

// Converting a transformStamped into a homogenous transform matrix
void PCL_Converter::TransformToT(const geometry_msgs::TransformStamped transform, cv::Mat& T){
    cv::Mat rot = cv::Mat_<double>(3, 3);
    QtoRot(transform.transform.rotation, rot);
    // Alternative inside code snippet
    T.at<double>(0,0) = rot.at<double>(0,0);
    T.at<double>(0,1) = rot.at<double>(0,1);
    T.at<double>(0,2) = rot.at<double>(0,2);
    T.at<double>(0,3) = transform.transform.translation.x;
    T.at<double>(1,0) = rot.at<double>(1,0);
    T.at<double>(1,1) = rot.at<double>(1,1);
    T.at<double>(1,2) = rot.at<double>(1,2);
    T.at<double>(0,3) = transform.transform.translation.y;
    T.at<double>(2,0) = rot.at<double>(2,0);
    T.at<double>(2,1) = rot.at<double>(2,1);
    T.at<double>(2,2) = rot.at<double>(2,2);
    T.at<double>(2,3) = transform.transform.translation.z;
    T.at<double>(3,0) = 0.0;
    T.at<double>(3,1) = 0.0;
    T.at<double>(3,2) = 0.0;
    T.at<double>(3,3) = 1.0;
}

// Transforming 3 points of a plane into a new coordinate frame
std::vector<cv::Point3d> PCL_Converter::convertPts(const std::vector<cv::Point3d> plane, const cv::Mat T){
    std::vector<cv::Point3d> out(3);
    for (int i=0; i<plane.size(); i++)
    {
        cv::Mat homPt = cv::Mat(plane.at(i));
        homPt.push_back(1.0);
        cv::Mat transformedPt = T * homPt;
        cv::Point3d tfpt(transformedPt.at<double>(0), transformedPt.at<double>(1), transformedPt.at<double>(2));
        out.at(i) = tfpt;
    }
    return out;
}

// Calculating a plane equation
void PCL_Converter::calculatePlane(std::vector<cv::Point3d> points, std::vector<float> &coeff){
    cv::Point3f vec1, vec2, norm;
    vec1 = points.at(1) - points.at(0);
    vec2 = points.at(2) - points.at(0);
    // normal
    norm = vec1.cross(vec2);
    
    // plane equation
    float a, b, c, d;

    a = norm.x;
    b = norm.y;
    c = norm.z;
    d = -(a * vec1.x + b * vec1.y + c * vec1.z);

    // Double check whether this is correct
    float fac = sqrt(norm.dot(norm));
    coeff.at(0) = a / fac;
    coeff.at(1) = b / fac;
    coeff.at(2) = c / fac;
    coeff.at(3) = d / fac;
    
}

// Calculating the intersection of a plane and a set of lines
void PCL_Converter::linesPlaneIntersection(std::vector<cv::Point3d> rays, std::vector<float> plane, std::vector<geometry_msgs::Point32>& intersections, cv::Point3d pointOnPlane){
    cv::Point3f intersection;
    geometry_msgs::Point32 msg;
    cv::Point3d normal(plane.at(0), plane.at(1), plane.at(2));
    for (int i=0; i < rays.size(); i++){
        linePlaneIntersection(intersection, rays.at(i), cv::Point3d(0.0, 0.0, 0.0), normal, pointOnPlane);
        msg.x = intersection.x;
        msg.y = intersection.y;
        msg.z = intersection.z;
        // msg = geometry_msgs::Point32(intersection.x, intersection.y, intersection.z);
        intersections.at(i) = msg; 
    }
}

// calculating the intersection of a single line and a plane
// From: https://stackoverflow.com/questions/7168484/3d-line-segment-and-plane-intersection
void PCL_Converter::linePlaneIntersection(cv::Point3f& contact, cv::Point3d ray, cv::Point3d rayOrigin, cv::Point3d normal, cv::Point3d coord){

    // getting the d - value
    float d = normal.dot(coord);
    
    // computing the scale value x for the ray
    float x = (d - normal.dot(rayOrigin)) / normal.dot(ray);
    // TODO: since the ray origin is always at 0, the first dot product could be omitted

    contact = rayOrigin + x * ray;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "plane_node");
    PCL_Converter convert;
    ros::spin();
    return 0;
}