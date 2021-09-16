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

// own imports
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <sensor_msgs/CompressedImage.h>
#include <message_filters/subscriber.h>
#include <tf2_ros/message_filter.h>

class PCL_Converter
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_;
    ros::Publisher pcl_pub_;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tf_listener_;
    image_geometry::PinholeCameraModel cam_model_;
    std::string world_frame;
    Eigen::MatrixXd world_plane;
    std::vector<Eigen::Vector3d> rays;

    // Message Filter
    message_filters::Subscriber<sensor_msgs::CompressedImage> filt_sub;
    tf2_ros::MessageFilter<sensor_msgs::CompressedImage> tf2_filt_;

public:
    PCL_Converter(/* args */);
    void imageCb(const sensor_msgs::CompressedImageConstPtr&);
    void convertPixelsToRays(int width, int height, std::vector<Eigen::Vector3d>& rays);
    Eigen::Vector3d pixelToRay(int u, int v, double cx, double cy, double fx, double fy);

    void QtoRot(geometry_msgs::Quaternion q, Eigen::Matrix3d& R);
    void TransformToT(geometry_msgs::TransformStamped, Eigen::Matrix4d&);
    Eigen::MatrixXd convertPts(Eigen::MatrixXd , Eigen::Matrix4d);
    Eigen::Vector4d calculatePlane(Eigen::Matrix3d);
    void linesPlaneIntersection(Eigen::Vector4d plane, std::vector<geometry_msgs::Point32> &intersections, std::vector<float> &color_vals, Eigen::Vector3d pointOnPlane, Eigen::Vector3d rayOrigin, cv::Mat &image);
    void linePlaneIntersection(Eigen::Vector3d& contact, Eigen::Vector3d ray, Eigen::Vector3d rayOrigin, Eigen::Vector3d normal, Eigen::Vector3d coord);
    Eigen::MatrixXd convertVecToMat(std::vector<Eigen::Vector3d>);

    // PCL color conversion
    float rgbtofloat(uint8_t r, uint8_t g, uint8_t b);

    // debugging utilties
    void logTransform(geometry_msgs::TransformStamped);
    void printMat(Eigen::MatrixXd, std::string);
};

// Member functions
PCL_Converter::PCL_Converter() : it_(nh_), tf_listener_(tfBuffer), tf2_filt_(tfBuffer, world_frame, 10, nh_)
{
    std::string image_topic, pcl_topic, info_topic;
    // image_topic = nh_.resolveName("hbv_1615/image_color");
    nh_.param("camera_info_topic", info_topic, std::string("/scouter_vision/camera_info"));
    nh_.param("image_topic", image_topic, std::string("/scouter_vision/image_raw"));
    nh_.param("pcl_topic", pcl_topic, std::string("pcl_plane"));
    nh_.param("world_frame", this->world_frame, std::string("map"));
    
    // Logging
    ROS_INFO("Subscribing to %s for camera info.", info_topic.c_str());
    ROS_INFO("Subscribing to %s for images. Using compressed transport", image_topic.c_str());
    ROS_INFO("Publishing to %s", pcl_topic.c_str());    
    
    // std::string image_topic = camera + "/image_raw";
    // Making the compressed image explicit
    image_transport::TransportHints hints("compressed");

    // Subscribing to the image
    // sub_ = it_.subscribe(image_topic, 1, &PCL_Converter::imageCb, this, hints);
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud>(pcl_topic, 1);

    // Adding the message filter
    filt_sub.subscribe(nh_, image_topic, 1);
    // in case target frame does not work
    // tf2_filt_->registerCallback(boost::bind(&PCL_Converter::imageCb, this, _1));
    tf2_filt_.setTargetFrame(world_frame);
    tf2_filt_.registerCallback(boost::bind(&PCL_Converter::imageCb, this, _1));
    ROS_INFO("Converting to %s, world frame: %s", tf2_filt_.getTargetFramesString().c_str(), world_frame.c_str());


    // world plane points
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
    info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(info_topic, ros::Duration(30.0));
    this->cam_model_.fromCameraInfo(info);
    ROS_INFO("Looking up point conversion from: %s to %s", this->cam_model_.tfFrame().c_str(), this->world_frame.c_str());

    // Assigning the rays - preallocation
    this->rays = std::vector<Eigen::Vector3d>(info->width * info->height);
    convertPixelsToRays(info->width, info->height, this->rays);
    ROS_INFO("Initialized point cloud class");
}

// Image Callback
void PCL_Converter::imageCb(const sensor_msgs::CompressedImageConstPtr& image_msg){

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
        ros::Time image_time = image_msg->header.stamp;
        ros::Duration timeout(1.0 / 1.);
        transform = tfBuffer.lookupTransform(this->cam_model_.tfFrame(), this->world_frame, image_time, timeout);
    } catch(tf2::TransformException& ex){
        ROS_WARN("Failed to get transform from %s to %s: %s", cam_model_.tfFrame().c_str(), this->world_frame.c_str(), ex.what());
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

    // Create the color channel part of the message
    std::vector<sensor_msgs::ChannelFloat32> channels(1);
    std::vector<float> col_values(rays.size());

    // Calculate the intersection of plane and rays
    std::vector<geometry_msgs::Point32> planePoints(rays.size());
    Eigen::Vector3d coordinate = nplane.col(0);
    Eigen::Vector3d rayOrigin(0.0, 0.0, 0.0);
    // include the color calculation in here
    linesPlaneIntersection(planecoeff, planePoints, col_values, coordinate, rayOrigin, image);

    // turn into pointcloud message
    sensor_msgs::PointCloud msg;
    msg.points = planePoints;
    msg.header = image_msg->header;
    // Assigning the color value
    channels.front().values = col_values;
    channels.front().name = "rgb";
    msg.channels = channels;

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


void PCL_Converter::linesPlaneIntersection(Eigen::Vector4d plane, std::vector<geometry_msgs::Point32> &intersections, std::vector<float> &color_vals, Eigen::Vector3d pointOnPlane, Eigen::Vector3d rayOrigin, cv::Mat &image){
    Eigen::Vector3d normal(plane(0), plane(1), plane(2));

    // Image stuff - from: https://stackoverflow.com/questions/7899108/opencv-get-pixel-channel-value-from-mat-image
    uint8_t* pixelPtr = (uint8_t*)image.data;
    int channels = image.channels();
    int row, col;
    uint8_t r, g, b;

    for (int i=0 ; i < this->rays.size(); i++){
        Eigen::Vector3d intersection;
        geometry_msgs::Point32 msg;
        linePlaneIntersection(intersection, this->rays.at(i), rayOrigin, normal, pointOnPlane);
        msg.x = intersection(0);
        msg.y = intersection(1);
        msg.z = intersection(2);
        intersections.at(i) = msg;

        // Getting the RGB out of the image and projecting it to the pointcloud
        row = floor(i / image.cols);
        col = i % image.cols;
        b = pixelPtr[row * channels * image.cols + col * channels + 0];
        g = pixelPtr[row * channels * image.cols + col * channels + 1];
        r = pixelPtr[row * channels * image.cols + col * channels + 2];

        color_vals.at(i) = rgbtofloat(r, g, b);
        // Do the PCL color conversion here
        // sensor_msgs::ChannelFloat32 fl;
        // fl.name = "rgb";
        // fl.
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


// PCL color conversion - see book
// "A systematic approach to learning robot programming with ROS p. 339"
float PCL_Converter::rgbtofloat(uint8_t r, uint8_t g, uint8_t b){
    uint32_t rgb = (static_cast<uint32_t> (r) << 16 | static_cast<uint32_t> (g) << 8 | static_cast<uint32_t> (b));
    float rgb_float = *reinterpret_cast<float*> (&rgb);
    return rgb_float;
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