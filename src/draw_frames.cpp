#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv4/opencv2/opencv.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <boost/foreach.hpp>

/**
 * @brief 
 * Coming from [here](http://wiki.ros.org/image_geometry/Tutorials/ProjectTfFrameToImage)
 * use with [this](http://docs.ros.org/en/api/image_geometry/html/c++/classimage__geometry_1_1PinholeCameraModel.html#a40ad5d3b964d63f41232428fb96376fe)
 * TODO: COmbine with line 318 in [here](http://docs.ros.org/en/api/image_geometry/html/c++/pinhole__camera__model_8cpp_source.html)
 * TODO: Combine with this code [here, OpenCV Forum](https://answers.opencv.org/question/67008/can-i-get-2d-world-coordinates-from-a-single-image-uv-coords/)
 * Stackoverflow comment [#1](https://stackoverflow.com/questions/14972490/ray-tracer-in-c-ray-plane-intersection?rq=1)
 * Stackoverflow comment [#2](https://stackoverflow.com/questions/12977980/in-opencv-converting-2d-image-point-to-3d-world-unit-vector)
 * original [OpenCV comment](https://answers.opencv.org/question/117354/back-projecting-a-2d-point-to-a-ray/)
 * 
 */


class FrameDrawer{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::CameraSubscriber sub_;
    image_transport::Publisher pub_;
    tf::TransformListener tf_listener_;
    image_geometry::PinholeCameraModel cam_model_;
    std::vector<std::string> frame_ids;
    int hz = 30;
    // CvFont font_;

    public:
        FrameDrawer(const std::vector<std::string>& frame_ids) : it_(nh_), frame_ids(frame_ids){
            std::string image_topic = nh_.resolveName("image");
            sub_ = it_.subscribeCamera(image_topic, 1, &FrameDrawer::imageCb, this);
            pub_ = it_.advertise("image_out" , 1);
        }

        // image callback function
        void imageCb(const sensor_msgs::ImageConstPtr&, const sensor_msgs::CameraInfoConstPtr&);
};

void FrameDrawer::imageCb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg){
    cv::Mat image;
    cv_bridge::CvImagePtr input_bridge;
    
    // Standard image conversion
    try
    {
        input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        image = input_bridge->image;
    }
    catch(cv_bridge::Exception& ex)
    {
        ROS_ERROR("Failed to convert image");
        return;
    }

    // Creating a model from the camera info
    cam_model_.fromCameraInfo(info_msg);

    // iterating over the frame_ids
    BOOST_FOREACH(const std::string& frame_id, frame_ids){
        tf::StampedTransform transform;

        // transform lookup
        try{
            // Time prelims
            ros::Time acquisition_time = info_msg->header.stamp;
            ros::Duration timeout(1.0 / this->hz);

            // Lookup
            tf_listener_.waitForTransform(cam_model_.tfFrame(), frame_id, acquisition_time, timeout);
            tf_listener_.lookupTransform(cam_model_.tfFrame(), frame_id, acquisition_time, transform);
        } catch(tf::TransformException& ex){
            ROS_WARN("Transform exception\n%s",ex.what());
            return;
        }

        // Actual processing
        // 3D point in the camera frame
        tf::Point pt = transform.getOrigin();       
        cv::Point3d pt_cv(pt.x(), pt.y(), pt.z());  
        
        // 2D point in the camera frame
        cv::Point2d uv;
        uv = cam_model_.project3dToPixel(pt_cv);

        // Some other bullshit processing - drawing a circle
        static const int RADIUS = 3;
        cv::circle(image, uv, RADIUS, CV_RGB(255, 0, 0), -1);
        // omitted the text processing here
    }
        
    // ! publishing with an empty msg?
    pub_.publish(input_bridge->toImageMsg());
    
}

int main(int argc, char** argv){
    ros::init(argc, argv, "draw_frames");
    std::vector<std::string> frame_ids(argv + 1, argv + argc);
    FrameDrawer drawer(frame_ids);
    ros::spin();
    return 0;
}