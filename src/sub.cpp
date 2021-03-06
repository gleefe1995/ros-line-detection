#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <std_msgs/UInt8MultiArray.h>
#include "line_detection/direction.h"
using namespace cv;
using namespace std;
ros::Publisher pub;

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

int dir;
Point pt1, pt2, pt3, pt4;

Mat img_line;
Mat img_edges;

void imageCallback(const std_msgs::UInt8MultiArray::ConstPtr& array)
{
  try
  {
    Mat frame = imdecode(array->data, 1);
    Mat canny;
    Mat hough;
    Mat img_filtered;
    Mat img_gray;
    Mat img_hsv;


    GaussianBlur(frame, img_gray, Size(3, 3), 0, 0);

    cvtColor(img_gray, img_hsv, COLOR_BGR2HSV);
    Scalar LowerBlack = {0,0,0};
    Scalar UpperBlack = {180,255,30};
    inRange(img_hsv, LowerBlack, UpperBlack, img_filtered);


    Canny( img_filtered , img_edges, 30, 127);


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

/*
    UMat uImage_edges;
    img_masked.copyTo(uImage_edges);

    vector<Vec4i> lines;
*/
    //HoughLinesP(uImage_edges, lines, rho, theta, hough_threshold, minLineLength, maxLineGap);
    //hough = canny.clone();
    img_line = Mat::zeros(img_edges.rows, img_edges.cols, CV_8UC3);



/*
    for(size_t i=0;i<lines.size();i++){
        Vec4i l = lines[i];
        line(img_line,Point(l[0],l[1]), Point(l[2],l[3]), Scalar(0,0,255), 1, CV_AA);
    }
    */
    Mat img_result;
    Mat img_mask2;
 // addWeighted(frame, 0.0, img_line, 1.0, 0.0, img_result);
   // addWeighted(frame, 0.0, img_masked, 1.0, 0.0, img_result);

    //img_mask2(Rect(0,0, img_edges.cols, 0.8*img_edges.rows)) = 0;



    vector<vector<Point>> v;
    findContours(img_masked, v, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
    if(v.size() !=0){
        auto area=0;
        auto idx=0;
        auto count=0;
        while (count <v.size()){
            if (area < v[count].size()){
                idx = count;
                area = v[count].size();
            }
            count++;
        }
        Rect rect = boundingRect(v[idx]);

        pt1.x = rect.x;
        pt1.y = rect.y;
        pt2.x = rect.x + rect.width;
        pt2.y = rect.y + rect.height;
        pt3.x = pt1.x+5;
        pt3.y = pt1.y-5;
        pt4.x = pt2.x-5;
        pt4.y = pt2.y+5;
        rectangle(img_line, pt1, pt2, CV_RGB(255,0,0),2);
        putText(img_line, "Line Detected", pt3, CV_FONT_HERSHEY_COMPLEX, 1, CV_RGB(255, 0, 0));
    }

    Moments M = moments(img_masked);
    if (M.m00 > 0) {
        Point p1(M.m10/M.m00, M.m01/M.m00);
        circle(img_line, p1, 5, Scalar(155, 200, 0), -1);
      }
      int c_x = M.m10/M.m00;
      // Tolerance to chooise directions
      auto tol = 15;
      auto count = countNonZero(img_masked);
      int w = img_edges.cols;
      // Turn left if centroid is to the left of the image center minus tolerance
      // Turn right if centroid is to the right of the image center plus tolerance
      // Go straight if centroid is near image center
      line_detection::direction dir_data;
      if (c_x < w/2-tol) {
        dir = 0;
        dir_data.dir=dir;
        putText(img_line, "0", pt4, CV_FONT_HERSHEY_COMPLEX, 1, CV_RGB(255, 0, 0));
      } else if (c_x > w/2+tol) {
        dir = 2;
        dir_data.dir=dir;
        putText(img_line, "2", pt4, CV_FONT_HERSHEY_COMPLEX, 1, CV_RGB(255, 0, 0));
      } else {
        dir = 1;
        dir_data.dir=dir;
        putText(img_line, "1", pt4, CV_FONT_HERSHEY_COMPLEX, 1, CV_RGB(255, 0, 0));
      }
      // Search if no line detected
      if (count == 0) {
        dir = 3;
        dir_data.dir=dir;
        putText(img_line, "3", pt4, CV_FONT_HERSHEY_COMPLEX, 1, CV_RGB(255, 0, 0));
      }

    pub.publish(dir_data);

    imshow("view", img_line);
    imshow("canny",img_masked);
    waitKey(1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cannot decode image");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "opencv_sub");

  namedWindow("view");


  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("camera/image", 5, imageCallback);
  pub = nh.advertise<line_detection::direction>("direction",1);



  ros::spin();
  destroyWindow("view");
  return 0;
}
