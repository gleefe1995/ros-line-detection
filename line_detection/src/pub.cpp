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

//Hough Transform 파라미터
float rho = 2; // distance resolution in pixels of the Hough grid
float theta = 1 * CV_PI / 180; // angular resolution in radians of the Hough grid
float hough_threshold = 90;	 // minimum number of votes(intersections in Hough grid cell)
float minLineLength = 120; //minimum number of pixels making up a line
float maxLineGap = 150;	//maximum gap in pixels between connectable line segments

//Region - of - interest vertices, 관심 영역 범위 계산시 사용
//We want a trapezoid shape, with bottom edge at the bottom of the image
float trap_bottom_width = 0.85;  // width of bottom edge of trapezoid, expressed as percentage of image width
float trap_top_width = 0.07;     // ditto for top edge of trapezoid
float trap_height = 0.4;         // height of the trapezoid expressed as percentage of image height



int main(int argc, char** argv)
{
    ros::init(argc, argv, "opencv_pub");
    
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::UInt8MultiArray>("camera/image", 1);

    VideoCapture cap(0);
    Mat frame;

    Mat img_edges;

    
    while(nh.ok())
    {
        cap >> frame;

        if(!frame.empty())
        {

            imshow("frame", frame);
            
            Mat canny;
            Mat hough;
            Mat img_filtered;
            Mat img_gray;


            cvtColor(frame, img_gray, COLOR_BGR2GRAY);
            GaussianBlur(img_gray, img_gray, Size(3, 3), 0, 0);
            Canny( img_gray , img_edges, 30, 127);

            int width = img_edges.cols; //this was changed
            int height = img_edges.rows;

            float trap_bottom_width = 0.85;  // width of bottom edge of trapezoid, expressed as percentage of image width
            float trap_top_width = 0.07;     // ditto for top edge of trapezoid
            float trap_height = 0.4;         // height of the trapezoid expressed as percentage of image height

            Point points[4];
            points[0] = Point((width*(1-trap_bottom_width))/2, height);
            points[1] = Point((width * (1 - trap_top_width)) / 2, height - height * trap_height);
            points[2] = Point(width - (width * (1 - trap_top_width)) / 2, height - height * trap_height);
            points[3] = Point(width - (width * (1 - trap_bottom_width)) / 2, height);


            Mat img_mask = Mat::zeros(img_edges.rows, img_edges.cols, CV_8UC1);
            Scalar ignore_mask_color = Scalar(255, 255, 255);
            const Point* ppt[1] = {points};
            int npt[] = {4};

            fillPoly(img_mask, ppt, npt, 1, Scalar(255,255,255), 8);

            Mat img_masked;
            bitwise_and(img_edges, img_mask, img_masked);


            UMat uImage_edges;
            img_masked.copyTo(uImage_edges);

            vector<Vec4i> lines;

            HoughLinesP(uImage_edges, lines, rho, theta, hough_threshold, minLineLength, maxLineGap);
            //hough = canny.clone();

            Mat img_line = Mat::zeros(img_edges.rows, img_edges.cols, CV_8UC3);



            for(size_t i=0;i<lines.size();i++){
                Vec4i l = lines[i];
                line(img_line,Point(l[0],l[1]), Point(l[2],l[3]), Scalar(0,0,255), 1, CV_AA);

            }
            Mat img_result;
            addWeighted(frame, 0.0, img_line, 1.0, 0.0, img_result);


            // Encode, Decode image example            
            vector<uchar> encode;
            vector<int> encode_param;
            
            encode_param.push_back(CV_IMWRITE_JPEG_QUALITY);
            encode_param.push_back(20);
            
            imencode(".jpg", img_result, encode, encode_param);
            Mat decode = imdecode(encode, 1);
            
            imshow("decode", decode);
            //imshow("img_mask", img_mask);
            // Convert encoded image to ROS std_msgs format
            std_msgs::UInt8MultiArray msgArray;
            msgArray.data.clear();
            msgArray.data.resize(encode.size());
            copy(encode.begin(), encode.end(), msgArray.data.begin());

            // Publish msg
            pub.publish(msgArray);

            waitKey(1);
          
        }

        ros::spinOnce();
    }

    return 0;
    
}
