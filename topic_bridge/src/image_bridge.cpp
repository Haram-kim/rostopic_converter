#include <ros/ros.h>

// opencv
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

// messages
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

struct ImageData{
    double time_stamp;
    cv::Mat image;
    uint32_t seq;
};

class ImageGrabber
{
public:

    ImageGrabber(std::string filename): save_dir(filename){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    void WriteImage(const ImageData& imageData);

    std::string save_dir;
};

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    static int lastseq = msg->header.seq-1;
    lastseq = msg->header.seq;
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    ImageData imageData;
    imageData.image = cv_ptr->image.clone();
    imageData.seq =  msg->header.seq;
    imageData.time_stamp = cv_ptr->header.stamp.toSec();

    WriteImage(imageData);    
}

void ImageGrabber::WriteImage(const ImageData& imageData)
{
    std::string img_name;
    img_name.append(save_dir);
    img_name.append(std::to_string(imageData.time_stamp));
    img_name.append(".png");
    cv::imwrite(img_name, imageData.image);
    std::cout << "Write image "<< img_name << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_bridge");
    ros::start();

    std::string filename = "~/";

    ros::NodeHandle nodeHandler("~");
    nodeHandler.param<std::string>("filename", filename, "");

    ImageGrabber imageGrabber(filename);

    ros::Subscriber image_sub = nodeHandler.subscribe("/image", 10, &ImageGrabber::GrabImage, &imageGrabber);

    ros::spin();

    ros::shutdown();
    return 0;
}
