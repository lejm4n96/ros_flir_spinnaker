#include "ros/ros.h"
#include <image_transport/image_transport.h>


#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/CameraInfo.h>

sensor_msgs::CameraInfo _camera_info;

image_transport::CameraPublisher*       _camera_info_pub     = NULL;
camera_info_manager::CameraInfoManager* _camera_info_manager = NULL;


void imageCallback(const sensor_msgs::ImageConstPtr& image)
{   
    if((_camera_info_pub->getNumSubscribers() == 0))
        return;

    sensor_msgs::Image img = *image;

    _camera_info = _camera_info_manager->getCameraInfo();
    _camera_info.header = img.header;
    _camera_info_pub->publish(img, _camera_info);

}

int main (int argc, char* argv[])
{
    ros::init (argc, argv, "calibration_publisher_node");

    ros::NodeHandle n_("~");
    ros::NodeHandle n;

    image_transport::ImageTransport it(n);

    std::string thermal_topic;
    ros::param::param<std::string>("~thermal_topic", thermal_topic, "thermal_camera/image");

    image_transport::Subscriber sub_thermal = it.subscribe(thermal_topic, 1, imageCallback);

    std::string camera_name;
    std::string camera_info_url;
    n_.getParam("camera_name", camera_name);
    n_.getParam("camera_info_url", camera_info_url);

    camera_info_manager::CameraInfoManager cam_info_manager(n);
    _camera_info_manager = &cam_info_manager;

    if (!_camera_info_manager->setCameraName(camera_name))
    {
        ROS_WARN_STREAM("Invalid camera name: " << camera_name << " for camera_info_manager");
    }

    if (_camera_info_manager->validateURL(camera_info_url))
    {
        if (!_camera_info_manager->loadCameraInfo(camera_info_url))
        {
            ROS_WARN("Camera calibration data not found in URL: %s", camera_info_url.c_str());
        }
        else if (!_camera_info_manager->isCalibrated())
        {
            ROS_WARN("Camera is not calibrated. Using default values.");
        }
    }
    else
    {
        ROS_ERROR_STREAM_ONCE("Invalid calibration URL syntax for CameraInfoManager.");
    }

    image_transport::CameraPublisher cam_info_pub = it.advertiseCamera("image_raw", 1);
    _camera_info_pub = &cam_info_pub;

    ros::spin();
}