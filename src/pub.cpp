#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/UInt8MultiArray.h>



using namespace cv;
using namespace std;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "opencv_pub");

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::UInt8MultiArray>("camera/image", 1);

    VideoCapture cap(0);
    Mat frame;


    while(nh.ok())
    {
        cap >> frame;

        if(!frame.empty())
        {

            // Encode, Decode image example
            vector<uchar> encode;

            vector<int> encode_param;

            encode_param.push_back(CV_IMWRITE_JPEG_QUALITY);
            encode_param.push_back(20);

            imencode(".jpg", frame, encode, encode_param);

            Mat decode = imdecode(encode, 1);

            imshow("decode", decode);

            //imshow("img_mask", img_mask);
            // Convert encoded image to ROS std_msgs format
            std_msgs::UInt8MultiArray msgArray;
            msgArray.data.clear();
            msgArray.data.resize(encode.size());
            copy(encode.begin(), encode.end(), msgArray.data.begin());


            msgArray.data.resize(encode.size());
            copy(encode.begin(), encode.end(), msgArray.data.begin());
            // Publish msg
            pub.publish(msgArray);

            waitKey(1);

        }
        ros::Rate loop_rate(100);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}
