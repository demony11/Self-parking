#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sstream>
#include <iostream>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Bool.h>

using namespace std;
using namespace cv;
Size large_size = Size(640, 512);
Size small_size = Size(640, 480);
Size dist_size=small_size;
string video_source="/media/chris/My Passport/abc.mp4";
Mat img_show;
void get_is_large(const std_msgs::BoolConstPtr &is_large_resolution)
{
  if(is_large_resolution->data==true)
  {
    dist_size=large_size;
  }else
  {
    dist_size=small_size;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"video_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);// useful when publish imgs
  image_transport::Publisher pub = it.advertise("/MVCamera/image_raw", 1);
  ros::Rate loop_rate(15);
  VideoCapture cap(video_source);//open video in the path
  if(!cap.isOpened())
  {
    std::cout<<"open video failed!"<<std::endl;
    return -1;
  }
  else
    std::cout<<"open video success!!!!!!!!"<<std::endl;

  Mat frame,img_show;
  bool isSuccess = true;

  while(nh.ok())
  {
    isSuccess = cap.read(frame);
    if(!isSuccess)//if the video ends, then break
    {
      std::cout<<"video ends"<<std::endl;
      break;
    }
    ///preprocess: opencv version
    Size src_size=Size(frame.cols,frame.rows);
    if(src_size!=dist_size)   // resize to 640x512
      resize(frame, frame, dist_size);
    std_msgs::Header header;
    header.stamp=ros::Time::now();
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
    pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
