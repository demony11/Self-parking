#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

int iLowH = 170;
int iHighH = 179;

int iLowS = 230; 
int iHighS = 255;

int iLowV = 234;
int iHighV = 255;

void imageCB(const sensor_msgs::ImageConstPtr& msg);

int main(int argc, char** argv) {

    
    ros::init(argc, argv, "image_subscribe");
    ros::start();
    ros::NodeHandle n;


    /*这句话非常重要！使用ros::Subscriber订阅图像消息，并将消息传入回调函数imageCB*/
    ros::Subscriber imageSub = n.subscribe("/MVCamera/image_raw", 1, &imageCB);
    // ros::Subscriber imageSub = n.subscribe("/usb_cam/image_raw", 1, &imageCB);


    ros::Rate loop_rate(30);
    while (ros::ok())
	{
        ros::spinOnce();
		loop_rate.sleep();
    }

    return 0;
}

void imageCB(
    const sensor_msgs::ImageConstPtr& msg
    )
{
    /*这句话非常重要！使用cv_bridge将图像消息转换为cv::Mat*/
    cv::Mat img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;


    if(img.empty())
    {
        printf("img empty!!!!\t");
        assert(!img.empty());
    }
    cv::Mat img2;
    img.copyTo(img2);

    cv::Mat imgHSV;
    std::vector<cv::Mat> hsvSplit;
    cv::cvtColor(img, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

    //因为我们读取的是彩色图，直方图均衡化需要在HSV空间做
    cv::split(imgHSV, hsvSplit);
    cv::equalizeHist(hsvSplit[2],hsvSplit[2]);
    cv::merge(hsvSplit,imgHSV);
    cv::Mat imgThresholded;

    cv::inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
    
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15)); 
    //闭操作 (连接一些连通域)
    cv::morphologyEx(imgThresholded, imgThresholded, cv::MORPH_CLOSE, element);

    cv::Mat element1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)); 
    //开操作 (去除一些噪点)
    cv::morphologyEx(imgThresholded, imgThresholded, cv::MORPH_OPEN, element1);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    // 查找轮廓，对应连通域
    cv::findContours( imgThresholded, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );

    // 寻找最大连通域
    double maxArea = 0;
    std::vector<cv::Point> maxContour;
    for(size_t i = 0; i < contours.size(); i++)
    {
        double area = cv::contourArea(contours[i]);
        if (area > maxArea)
        {
            maxArea = area;
            maxContour = contours[i];
        }
    }

    // 将轮廓转为矩形框
    cv::Rect maxRect = boundingRect(maxContour);
    cv::rectangle(img, maxRect, cv::Scalar(0,255,0),5);
    cv::cvtColor(imgThresholded, imgThresholded, cv::COLOR_GRAY2BGR); 
    cv::imshow("imgThresholded", imgThresholded); //show the thresholded image

    cv::imshow("img", img); //show the original image
    cv::waitKey(1);
    
}