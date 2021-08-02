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
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <omp.h>

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
    Eigen::MatrixXd world_plane;
    std::vector<Eigen::Vector3d> rays;

public:
    PCL_Converter(/* args */);
    void imageCb(const sensor_msgs::ImageConstPtr&, const sensor_msgs::CameraInfoConstPtr&);
    void convertPixelsToRays(int width, int height, std::vector<Eigen::Vector3d>& rays);
    Eigen::Vector3d pixelToRay(int u, int v, double cx, double cy, double fx, double fy);

    void QtoRot(geometry_msgs::Quaternion q, Eigen::Matrix3d& R);
    void TransformToT(geometry_msgs::TransformStamped, Eigen::Matrix4d&);
    Eigen::MatrixXd convertPts(Eigen::MatrixXd , Eigen::Matrix4d);
    Eigen::Vector4d calculatePlane(Eigen::Matrix3d);
    void linesPlaneIntersection(Eigen::Vector4d plane, std::vector<geometry_msgs::Point32> &intersections, Eigen::Vector3d pointOnPlane, Eigen::Vector3d rayOrigin);
    void linePlaneIntersection(Eigen::Vector3d& contact, Eigen::Vector3d ray, Eigen::Vector3d rayOrigin, Eigen::Vector3d normal, Eigen::Vector3d coord);
    Eigen::MatrixXd convertVecToMat(std::vector<Eigen::Vector3d>);

    // debugging utilties
    void logTransform(geometry_msgs::TransformStamped);
    void printMat(Eigen::MatrixXd, std::string);
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
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud>(pcl_topic, 1);

    // world plane points
    // TODO: see if this throws an error
    cv::Vec3d plane_pt1, plane_pt2, plane_pt3;
    std::vector<float> pt1, pt2, pt3;
    nh_.param("plane/point1", pt1, {0.f, 0.f, 0.f});
    nh_.param("plane/point1", pt2, {0.f, 1.f, 0.f});
    nh_.param("plane/point1", pt3, {1.f, 0.f, 0.f});

    // World plane vector
    std::vector<Eigen::Vector3d> plane;
    plane.push_back(Eigen::Vector3d(pt1.at(0), pt1.at(1), pt1.at(2)));
    plane.push_back(Eigen::Vector3d(pt2.at(0), pt2.at(1), pt2.at(2)));
    plane.push_back(Eigen::Vector3d(pt3.at(0), pt3.at(1), pt3.at(2)));
    
    // Convert the World plane into a cv::matrix - COLUMN VECTORS - so that the matrix multiplication can happen naturally
    this->world_plane = convertVecToMat(plane);

    // Getting the Camera model - preallocation
    sensor_msgs::CameraInfoConstPtr info;
    info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/hbv_1615/camera_info", ros::Duration(3.0));
    this->cam_model_.fromCameraInfo(info);

    // Assigning the rays - preallocation
    this->rays = std::vector<Eigen::Vector3d>(info->width * info->height);
    convertPixelsToRays(info->width, info->height, this->rays);
    ROS_INFO("Initialized point cloud class");
}

// Image Callback
void PCL_Converter::imageCb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg){

    // Convert image
    cv::Mat image;
    // int width, height;
    cv_bridge::CvImagePtr input_bridge;
    try{
        input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        image = input_bridge->image;
    } catch(cv_bridge::Exception& ex){
        ROS_ERROR("Failed to convert image");
        return;
    }
    
    // create a camera model from the camera info - apparently cheap
    // cam_model_.fromCameraInfo(info_msg);

    // look up the transform of the camera in the world frame
    geometry_msgs::TransformStamped transform;
    try{
        ros::Time image_time = info_msg->header.stamp;
        ros::Duration timeout(1.0 / 30);
        transform = tfBuffer.lookupTransform(this->cam_model_.tfFrame(), this->world_frame, image_time, timeout);
    } catch(tf2::TransformException& ex){
        ROS_WARN("Failed to get transform from %s to %s", cam_model_.tfFrame().c_str(), this->world_frame.c_str());
        return;
    }

    // Convert representations
    Eigen::Matrix4d T;
    TransformToT(transform, T);

    // Turn the plane points into the new coordinate frame
    Eigen::MatrixXd newplane = convertPts(this->world_plane, T);
    Eigen::Matrix3d nplane = newplane.topLeftCorner<3,3>();

    // Calculate the new plane equation
    Eigen::Vector4d planecoeff = calculatePlane(nplane);

    // Calculate the intersection of plane and rays
    std::vector<geometry_msgs::Point32> planePoints(rays.size());
    Eigen::Vector3d coordinate = nplane.col(0);
    Eigen::Vector3d rayOrigin(0.0, 0.0, 0.0);
    linesPlaneIntersection(planecoeff, planePoints, coordinate, rayOrigin);

    // turn into pointcloud message
    sensor_msgs::PointCloud msg;
    msg.points = planePoints;
    msg.header = info_msg->header;

    // Publish
    this->pcl_pub_.publish(msg);        
}


// Converting a single pixel to a ray - with Eigen
void PCL_Converter::convertPixelsToRays(int width, int height, std::vector<Eigen::Vector3d> &rays){
    double cx = this->cam_model_.cx();
    double cy = this->cam_model_.cy();
    double fx = this->cam_model_.fx();
    double fy = this->cam_model_.fy();

    for (int y=0; y < height; y++){
        for (int x=0; x < width; x++){
            Eigen::Vector3d ray = pixelToRay(x, y, cx, cy, fx, fy);
            int index = y * width + x;
            rays.at(index) = ray;
        }
    }
}

// Converting a single pixel to a ray - following the definition of the python class
Eigen::Vector3d PCL_Converter::pixelToRay(int u, int v, double cx, double cy, double fx, double fy){
    double x = (u - cx) / fx;
    double y = (v - cy) / fy;
    float norm = sqrt(x*x + y*y);
    x /= norm;
    y /= norm;
    double z = 1.0 / norm;
    Eigen::Vector3d point(x, y, z);
    return point;
}

// COnverting between Quaternion and Rotation Matrix
void PCL_Converter::QtoRot(geometry_msgs::Quaternion q, Eigen::Matrix3d& R){
    Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
    R = quat.toRotationMatrix();
}

// Eigen implementation of the transformation
void PCL_Converter::TransformToT( geometry_msgs::TransformStamped transform, Eigen::Matrix4d& T){
    Eigen::Matrix3d rot;
    QtoRot(transform.transform.rotation, rot);
    Eigen::Vector3d t;
    t << transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z;
    Eigen::RowVector4d zeros(0.0, 0.0, 0.0, 1.0);
    T << rot, t, zeros;
}

Eigen::MatrixXd PCL_Converter::convertPts(Eigen::MatrixXd pts, Eigen::Matrix4d T){
    Eigen::MatrixXd out;
    out = T * pts;
    return out;
}


// Calculating plane equation using eigen
Eigen::Vector4d PCL_Converter::calculatePlane(Eigen::Matrix3d newplane){
    // The last row should be full of ones
    // only takes the first 3 columns as points
    // Eigen::Matrix3d pts = newplane.block<3,3>(0,0);
    Eigen::Vector3d vec1 = newplane.col(1) - newplane.col(0);
    Eigen::Vector3d vec2 = newplane.col(2) - newplane.col(0);
    
    Eigen::Vector3d norm = vec1.cross(vec2);
    
    double d = -(norm(0) * vec1(0) + norm(1) *  vec1(1) + norm(2) * vec1(2)); 
    // double f = -(vec1.dot(norm));          // should be equivalent

    double fac  = sqrt(norm.dot(norm));
    Eigen::Vector4d factors(norm(0), norm(1), norm(2), d);
    factors /= fac;
    return factors;
}


void PCL_Converter::linesPlaneIntersection(Eigen::Vector4d plane, std::vector<geometry_msgs::Point32> &intersections, Eigen::Vector3d pointOnPlane, Eigen::Vector3d rayOrigin){
    Eigen::Vector3d normal(plane(0), plane(1), plane(2));
    #pragma omp parallel for
    for (int i=0 ; i < this->rays.size(); i++){
        Eigen::Vector3d intersection;
        geometry_msgs::Point32 msg;
        linePlaneIntersection(intersection, this->rays.at(i), rayOrigin, normal, pointOnPlane);
        msg.x = intersection(0);
        msg.y = intersection(1);
        msg.z = intersection(2);
        intersections.at(i) = msg;
    }
}

void PCL_Converter::linePlaneIntersection(Eigen::Vector3d& contact, Eigen::Vector3d ray, Eigen::Vector3d rayOrigin, Eigen::Vector3d normal, Eigen::Vector3d coord){
    double d = normal.dot(coord);
    double x = (d - normal.dot(rayOrigin)) / normal.dot(ray);

    contact = rayOrigin + x * ray;
}


// calculating the intersection of a single line and a plane
// From: https://stackoverflow.com/questions/7168484/3d-line-segment-and-plane-intersection
// Converting the vector of points into a - HOMOGENISED matrix - for better multiplication
Eigen::MatrixXd PCL_Converter::convertVecToMat(std::vector<Eigen::Vector3d> plane_pts){
    Eigen::MatrixXd mat(4, plane_pts.size());
    for (int col = 0; col<plane_pts.size(); col++){
        mat(0,col) = plane_pts.at(col)(0);
        mat(1,col) = plane_pts.at(col)(1);
        mat(2,col) = plane_pts.at(col)(2);
        mat(3,col) = 1;
    }
    return mat;
}


// Debugging functions:
void PCL_Converter::logTransform( geometry_msgs::TransformStamped transform)  {
    ROS_INFO("Transform from %s to %s\n\tx: %f\n\ty: %f\n\tz: %f\n\tw: %f\n\trx: %f\n\try: %f\n\trz: %f\n",
                transform.header.frame_id.c_str(),
                transform.child_frame_id.c_str(),
                transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z,
                transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z
            );
}

void PCL_Converter::printMat(Eigen::MatrixXd mat, std::string name = ""){
    std::cout << "Matrix: " << name << std::endl <<  mat << std::endl;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "plane_node");
    PCL_Converter convert;
    ros::spin();
    return 0;
}