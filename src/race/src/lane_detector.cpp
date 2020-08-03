// ros package

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int16.h"
#include "race/lane_info.h"

// opencv package
#include "opencv2/opencv.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

// std package
#include <iostream>
#include <vector>
#include <cstring>
#include <string>
#include <time.h>
#include <math.h>
#include <algorithm>
#include <cv.h>

using namespace std;
using namespace cv;

enum MODE{
  GO_STRAIGHT,
  GO_RIGHT,
  GO_LEFT
};

MODE GO_MODE = GO_STRAIGHT;

const int WIDTH = 680;
const int HEIGHT = 480;

bool is_left = false;
bool is_right = false;

double slope_threshold = 90;

Point left_pos_upper = Point(0,0);
Point left_pos_lower =  Point(0,HEIGHT);

Point right_pos_upper = Point(WIDTH,0);
Point right_pos_lower = Point(WIDTH,HEIGHT);

const Point default_left_vertices[4] =
{
  Point(0,0),
  Point(WIDTH/5, 0),
  Point(WIDTH/5,HEIGHT),
  Point(0,HEIGHT)
};

Point left_vertices[4] =
{
  Point(left_pos_upper.x-60,0),
  Point(left_pos_upper.x+60, 0),
  Point(left_pos_lower.x+60,HEIGHT),
  Point(left_pos_lower.x-60,HEIGHT)
};

const Point default_right_vertices[4] =
{
  Point(WIDTH/6*4,0),
  Point(WIDTH, 0),
  Point(WIDTH,HEIGHT),
  Point(WIDTH/6*4,HEIGHT)
};

Point right_vertices[4] =
{
  Point(right_pos_upper.x-60,0),
  Point(right_pos_upper.x+60, 0),
  Point(right_pos_lower.x+60,HEIGHT),
  Point(right_pos_lower.x-60,HEIGHT)
};

// Setting vertices for bird eyes view
Point2f src_vertices[4];

Point2f dst_vertices[4];


void Callback(const std_msgs::Int16)
{
  std_msgs::Int16 return_msg;

  ROS_INFO("I heard: [%d]", return_msg.data);
}

void set_left_roi(Mat& img, Mat& img_ROI){

  vector <Point> Left_Point;

  for(int i = 0; i < 4; i++)
  {
    Left_Point.push_back(default_left_vertices[i]);
  }

  Mat roi(img.rows, img.cols, CV_8U, Scalar(0));

  fillConvexPoly(roi, Left_Point, Scalar(255));

  Mat filteredImg_Left;
  img.copyTo(filteredImg_Left, roi);

  img_ROI = filteredImg_Left.clone();

}

void set_dynamic_left_roi(Mat& img, Mat& img_ROI){

  vector <Point> Left_Point;

  for(int i = 0; i < 4; i++)
  {
    Left_Point.push_back(left_vertices[i]);

    // circle(img,left_vertices[i],15,Scalar(255,255,255),5);
  }

  Mat roi(img.rows, img.cols, CV_8U, Scalar(0));

  fillConvexPoly(roi, Left_Point, Scalar(255));

  Mat filteredImg_Left;
  img.copyTo(filteredImg_Left, roi);

  img_ROI = filteredImg_Left.clone();

}



void set_right_roi(Mat& img, Mat& img_ROI){

  vector <Point> Right_Point;

  for(int i = 0; i < 4; i++)
  {
    Right_Point.push_back(default_right_vertices[i]);
  }

  Mat roi(img.rows, img.cols, CV_8U, Scalar(0));

  fillConvexPoly(roi, Right_Point, Scalar(255));

  Mat filteredImg_Right;

  img.copyTo(filteredImg_Right, roi);

  img_ROI = filteredImg_Right.clone();

}

void set_dynamic_right_roi(Mat& img, Mat& img_ROI){

  vector <Point> Right_Point;

  for(int i = 0; i < 4; i++)
  {
    Right_Point.push_back(right_vertices[i]);
    // circle(img,right_vertices[i],15,Scalar(255,255,255),5);
  }

  Mat roi(img.rows, img.cols, CV_8U, Scalar(0));

  fillConvexPoly(roi, Right_Point, Scalar(255));

  Mat filteredImg_Right;

  img.copyTo(filteredImg_Right, roi);

  img_ROI = filteredImg_Right.clone();

}


void transform(Point2f* src_vertices, Point2f* dst_vertices, Mat& src, Mat &dst){
    Mat M = getPerspectiveTransform(src_vertices, dst_vertices);
    warpPerspective(src, dst, M, dst.size(), INTER_LINEAR, BORDER_CONSTANT);
}

Mat convert_hsl(Mat &src)
{
  Mat hls_img;
  //Mat hsv_img;

  cvtColor(src,hls_img,COLOR_BGR2HLS);
  //cvtColor(src,hsv_img,COLOR_BGR2HSV);

  // vector<Mat> hsv_plane;
  // split(hsv_img,hsv_plane);

  //imshow("s_channel",hsv_plane[1]);
  //Mat s_thresh;
  Mat wImgMask, yImgMask, imgMask;
  Mat R_wImgMask, g_ImgMask;
  //threshold(hsv_plane[1],s_thresh,125,255,CV_THRESH_BINARY);
  //imshow("s_thresh",s_thresh);

  //RGB WHITE MASK
  inRange(src,Scalar(100,100,200),Scalar(255,255,255),R_wImgMask);

  // WHITE MASK
  inRange(hls_img, Scalar(0, 150, 0), Scalar(255, 255, 255), wImgMask);
  // inRange(hls_img, Scalar(0, 0, 225), Scalar(180, 30, 255), wImgMask);
  // YELLOW MASK
  // inRange(hls_img, Scalar(15, 30, 30), Scalar(65, 105, 255), yImgMask);
  // inRange(hls_img, Scalar(15, 30, 115), Scalar(35, 204, 255), yImgMask);
  inRange(hls_img, Scalar(15, 100, 115), Scalar(35, 204, 255), yImgMask);
  // inRange(hls_img, Scalar(0, 0, 0), Scalar(255, 255, 255), yImgMask);

  // inRange(hls_img,Scalar(15, 15, 20),Scalar(140, 100, 80),g_ImgMask);


  // imshow("RGB White Mask",R_wImgMask);
  // imshow("White Mask", wImgMask);
  // imshow("Yellow Mask", yImgMask);
  // imshow("Green Mask", g_ImgMask);

  imgMask = wImgMask | yImgMask;

  Mat result;

  bitwise_and(src,src,result,imgMask);

  // cvtColor(result,result,COLOR_BGR2HLS);
  // imshow("Result",result);

  return result;
}

int maximum(int straight_num, int left_num,int right_num)
{
  int max = straight_num;

  if(max < left_num)
    max = left_num;
  if(max < right_num)
    max = right_num;

  if(max == straight_num) return 1;
  else if(max == left_num) return 2;
  else if(max == right_num) return 3;
}

bool cmp(const Vec2i &a, const Vec2i &b)
{
  return a[1] < b[1];
}


int data_preprocess(Mat &src,vector<Vec4i> lines, double *real_line_x, double *real_line_y, vector<Vec2i> &lines_vec2i, int dir)
{
  if(lines.size() < 10 && dir == 0)
  {
    // is_left = false;
    printf("No Left Line\n");
    return -1;
  }
  if(lines.size() < 10 && dir == 1)
  {
    // is_right = false;
    printf("No Right Line\n");
    return -1;
  }


  // Number of Sub Point
  int straight_num = 0;
  int right_num = 0;
  int left_num = 0;

  // Array Of GO_MODE
  double straight_x[10000];
  double straight_y[10000];

  double right_x[10000];
  double right_y[10000];

  double left_x[10000];
  double left_y[10000];

  for(int i = 0; i < lines.size(); i++)
  {
    Vec4i line = lines[i];

    int x1 = line[0];
    int y1 = line[1];
    int x2 = line[2];
    int y2 = line[3];


    double dx = x1 - x2;
    double dy = y1 - y2;
    double angle;

    if(dx == 0)
    {
      angle = 90;
    }
    else{

    double r = sqrt(dx*dx + dy*dy); // distance

    angle = atan2(dy,dx);
    angle = angle*180/3.14;
    }

    if(angle < 0)
    {
      angle += 180;
    }

    if(angle >= 80 && angle < 100)
    {
      straight_x[straight_num] = x1;
      straight_y[straight_num++] = y1;

      straight_x[straight_num] = x2;
      straight_y[straight_num++] = y2;

      // circle(src,Point(x1,y1),3,Scalar(0,255,0),5);
      // circle(src,Point(x2,y2),3,Scalar(0,255,0),5);
    }
    else if(angle >= 0 && angle < 80)
    {
      left_x[left_num] = x1;
      left_y[left_num++] = y1;

      left_x[left_num] = x2;
      left_y[left_num++] = y2;

      // circle(src,Point(x1,y1),3,Scalar(255,0,0),5);
      // circle(src,Point(x2,y2),3,Scalar(255,0,0),5);
    }
    else if(angle < 150 && angle >= 100)
    {
      right_x[right_num] = x1;
      right_y[right_num++] = y1;

      right_x[right_num] = x2;
      right_y[right_num++] = y2;

      // circle(src,Point(x1,y1),3,Scalar(0,0,255),5);
      // circle(src,Point(x2,y2),3,Scalar(0,0,255),5);
    }

  }


  // cout << " 어디서 seg?" << endl;

  int biggest = maximum(straight_num, left_num, right_num);
  //cout << straight_num << ' ' << left_num <<  ' ' << right_num << endl;
  //cout << biggest << endl;

  if(GO_MODE == GO_STRAIGHT)
  {
    for(int i = 0; i < straight_num; i++)
    {
      // lines_vec2i[i][0] = straight_x[i];
      // lines_vec2i[i][1] = straight_y[i];
      lines_vec2i.push_back(Vec2i(straight_x[i],straight_y[i]));
    }

    // GO_MODE = GO_STRAIGHT;

    // cout << "GO_STRAIGHT" << endl;

    // return straight_num;
  }

  else if(GO_MODE == GO_LEFT)
  {
    for(int i = 0; i < left_num; i++)
    {
      // lines_vec2i[i][0] = left_x[i];
      // lines_vec2i[i][1] = left_y[i];
      lines_vec2i.push_back(Vec2i(left_x[i],left_y[i]));
    }

    // GO_MODE = GO_LEFT;

    //cout << "GO_LEFT" << endl;

    // return left_num;
  }

  else if(GO_MODE == GO_RIGHT)
  {
    for(int i = 0; i < right_num; i++)
    {
      // lines_vec2i[i][0] = right_x[i];
      // lines_vec2i[i][0] = right_y[i];
      lines_vec2i.push_back(Vec2i(right_x[i],right_y[i]));
    }

    // GO_MODE = GO_RIGHT;

    // cout << "GO_RIGHT" << endl;

    // return right_num;
  }

  // cout << "sort 전" << endl;

  sort(lines_vec2i.begin(), lines_vec2i.end(), cmp);

  // cout << "sort 후" << endl;
  // cout << "lines_vec2i.size() : " << lines_vec2i.size() << endl;

  if(lines_vec2i.size())
  {
    int pre_y = lines_vec2i[0][1];
    int x_min = lines_vec2i[0][0];

    int real_line_idx = 0;
    int pre_y_count = 0;

    for(int i = 0; i < lines_vec2i.size(); i++)
    {
      // cout << "if 전" << endl;
      if(pre_y == lines_vec2i[i][1])
      {
        pre_y_count++;

        if(x_min > lines_vec2i[i][0])
          x_min = lines_vec2i[i][0];
      }
      else if(pre_y != lines[i][1])
      {
        real_line_x[real_line_idx] = x_min;
        real_line_y[real_line_idx++] = pre_y;

      // circle(src,Point(x_min,pre_y),5,Scalar(255,255,0),5);
      //circle(src,Point(x1,y1),5,Scalar(0,0,255),5);
        pre_y = lines_vec2i[i][1];
        x_min = lines_vec2i[i][0];
        pre_y_count = 0;
      }
      // cout << "else if 후" << endl;
    // else if(pre_y != lines[i][1] && pre_y_count <= 0)
    // {
    //   pre_y = lines_vec2i[i][1];
    //   x_min = lines_vec2i[i][0];
    //   pre_y_count = 0;
    // }
    }
    return real_line_idx;
  }
  // cout << "return 직전" << endl;
  return -1;

}

void ransac(Mat &src,Mat &imgResult,int real_line_idx,double *real_lines_x, double *real_lines_y, int dir)
{
  // RANSAC

  // cout << "1" << endl;
  srand(time(NULL));

  double noise_sigma = 100*0.1;

  Mat A(real_line_idx,2,CV_64FC1);
  Mat B(real_line_idx,1,CV_64FC1);

  for( int i=0 ; i < real_line_idx ; i++ )
	{
		A.at<double>(i,0) = real_lines_x[i] ;
	}

	for( int i=0 ; i < real_line_idx ; i++ )
	{
		A.at<double>(i,1) = 1.0 ;
	}

	for( int i=0 ; i < real_line_idx ; i++ )
	{
		B.at<double>(i,0) = real_lines_y[i] ;
	}

  // cout << "2" << endl;

  // imshow("Mat_A",A);
  // imshow("Mat_B",B);

  int n_data = real_line_idx;
  int N = real_line_idx;
  double T = noise_sigma;

  // n_sample : 몇 개의 데이터를 샘플링
  int n_sample = 3;
  int max_cnt = 0;

  Mat best_model(2,1,CV_64FC1);

  for(int i = 0; i < N; i++)
  {
    // random sampling - 3 Point
    int k[3] = {-1, };
    k[0] = floor((rand()%100+1))+1;

    do
		{
			k[1] = floor((rand()%100+1))+1;
		}while(k[1]==k[0] || k[1]<0) ;

    do
		{
			k[2] = floor((rand()%100+1))+1;
		}while(k[2]==k[0] || k[2]==k[1] || k[2]<0);

    cv::Mat AA(3,2,CV_64FC1) ;
		cv::Mat BB(3,1, CV_64FC1) ;

    for( int j=0 ; j<3 ; j++ )
		{
			// AA.at<double>(j,0) = real_left_lines_x[k[j]] * real_left_lines_x[k[j]] ;
			AA.at<double>(j,0) = real_lines_x[k[j]] ;
			AA.at<double>(j,1) = 1.0 ;

			BB.at<double>(j,0) = real_lines_y[k[j]] ;
		}
    // cout << "3" << endl;

    cv::Mat AA_pinv(3,2,CV_64FC1) ;
		invert(AA, AA_pinv, cv::DECOMP_SVD);

    cv::Mat X = AA_pinv * BB ;

    cv::Mat residual(real_line_idx,1,CV_64FC1) ;
		residual = cv::abs(B-A*X) ;

    int ransac_cnt = 0;

    for( int j=0 ; j<real_line_idx ; j++ )
		{
			double data = residual.at<double>(j,0) ;

			if( data < T )
			{
				ransac_cnt++ ;
			}
		}

    if( ransac_cnt > max_cnt )
		{
			best_model = X ;
			max_cnt = ransac_cnt ;
		}

  }
  // cout << "4" << endl;
  // cout << "real_line_idx :" << real_line_idx << endl;

  cv::Mat residual = cv::abs(A*best_model - B) ;
	std::vector<int> vec_index ;

  for( int i=0 ; i<real_line_idx ; i++ )
	{
		double data = residual.at<double>(i, 0) ;

    if( data < T )
		{
      // cout << "vector push_back" << endl;
			vec_index.push_back(i) ;
		}

	}

  // cout << "4-1" << endl;

  cv::Mat A2(vec_index.size(),2, CV_64FC1) ;
	cv::Mat B2(vec_index.size(),1, CV_64FC1) ;

  for( int i=0 ; i<vec_index.size() ; i++ )
	{
		// A2.at<double>(i,0) = real_left_lines_x[vec_index[i]] * real_left_lines_x[vec_index[i]]  ;
		A2.at<double>(i,0) = real_lines_x[vec_index[i]] ;
		A2.at<double>(i,1) = 1.0 ;

		B2.at<double>(i,0) = real_lines_y[vec_index[i]] ;
	}

  // cout << "4-2" << endl;
  if(!vec_index.size() && dir == 0)
  {
    cout << "NO Left LINE" << endl;
    return;
  }
  if(!vec_index.size() && dir == 1)
  {
    cout << "NO Right LINE" << endl;
    return;
  }
  // cout << "5" << endl;
  // cout << vec_index.size() << endl;

  cv::Mat A2_pinv(2,vec_index.size(),CV_64FC1) ;
	invert(A2, A2_pinv, cv::DECOMP_SVD);

  cv::Mat X = A2_pinv * B2 ;

  cv::Mat F = A*X ;

  int interval = 1;

  double dx1,dx2;
  double dy1,dy2;

  double slope;
  double b;

  for( int iy=0 ; iy<real_line_idx ; iy++ )
	{
		cv::circle(imgResult, cv::Point(real_lines_x[iy]*interval, real_lines_y[iy]*interval) ,3, cv::Scalar(0,0,255), CV_FILLED) ;

		double data = F.at<double>(iy,0) ;

    // if(iy == 0)
    // {
    //   dx1 = real_lines_x[iy];
    //   dy1 = data;
    // }
    // else if(iy == 1)
    // {
    //   dx2 = real_lines_x[iy];
    //   dy2 = data;
    // }
		cv::circle(imgResult, cv::Point(real_lines_x[iy]*interval, data*interval) ,5, cv::Scalar(0,255,0), CV_FILLED) ;
	}

  // cout << "6" << endl;
  // if(!(dx1-dx2) && dir == 0)
  // {
  //   slope = (dy1 - dy2)/(dx1 - dx2);
  //   b = dy1 - slope*dx1;
  //
  //   left_pos_upper.x = -b/slope;
  //   left_pos_lower.x = (HEIGHT-b)/slope;
  //
  //   cout << "left_pos_upper.x : " << left_pos_upper.x << endl;
  //   cout << "left_pos_lower.x : " << left_pos_lower.x << endl;
  //
  //
  //   // left_pos_upper = Point((0-b)/slope,0);
  //   // left_pos_lower = Point((HEIGHT-b)/slope,HEIGHT);
  //
  //   // cv::circle(src, left_pos_upper ,10, cv::Scalar(0,0,255), CV_FILLED) ;
  //   // cv::circle(src, left_pos_lower ,10, cv::Scalar(0,0,255), CV_FILLED) ;
  // }
  // else if(!(dx1-dx2) && dir == 0)
  // {
  //   left_pos_upper.x = dx1;
  //   left_pos_lower.x = dx1;
  //
  //   cout << "left_pos_upper.x : " << left_pos_upper.x << endl;
  //   cout << "left_pos_lower.x : " << left_pos_lower.x << endl;
  //
  //   // left_pos_upper = Point(dx1,0);
  //   // left_pos_lower = Point(dx1,HEIGHT);
  // }
  //
  // if(!(dx1-dx2) && dir == 1)
  // {
  //   slope = (dy1 - dy2)/(dx1 - dx2);
  //   b = dy1 - slope*dx1;
  //
  //   right_pos_upper.x = -b/slope;
  //   right_pos_lower.x = (HEIGHT-b)/slope;
  //
  //   cout << "right_pos_upper.x : " << right_pos_upper.x << endl;
  //   cout << "right_pos_lower.x : " << right_pos_lower.x << endl;
  //
  //   // right_pos_upper = Point((0-b)/slope,0);
  //   // right_pos_lower = Point((HEIGHT-b)/slope,HEIGHT);
  //
  //   // cv::circle(src, right_pos_upper ,10, cv::Scalar(255,0,0), CV_FILLED) ;
  //   // cv::circle(src, right_pos_lower ,10, cv::Scalar(255,0,0), CV_FILLED) ;
  // }
  // else if(!(dx1-dx2) && dir == 1)
  // {
  //   right_pos_upper.x = dx1;
  //   right_pos_lower.x = dx1;
  //
  //   cout << "right_pos_upper.x : " << right_pos_upper.x << endl;
  //   cout << "right_pos_lower.x : " << right_pos_lower.x << endl;
  //   // right_pos_upper = Point(dx1,0);
  //   // right_pos_lower = Point(dx1,HEIGHT);
  // }
  //




  // cout << "slope : " << slope << endl;

  // double car_x = 0;
  // double car_y = -1;
  // double edge_x = dx1 - dx2;
  // double edge_y = dy1 - dy2;
  // double r = sqrt(edge_x*edge_x + edge_y*edge_y);
  // double line_x = edge_x/r;
  // double line_y = edge_y/r;
  // double atan_degree = atan(edge_y/edge_x);
  // atan_degree *= 180/3.1416;
  //
  // if(atan_degree < 0)
  //   atan_degree += 180;
  //
  // double result = car_x*line_x + car_y*line_y;
  // cout << "****************************************" << endl;
  //
  //
  // cout << atan_degree << endl;
  //
  // if(atan_degree > 75 && atan_degree < 105)
  //   cout << "Straight" << endl;
  // else if(atan_degree > 105)
  //   cout << "Right Lane" << endl;
  // else
  //   cout << "Left Lane" << endl;
  // // if(cos(20*3.1416/180) > abs(result))
  // // {
  // //   cout << "cos(20*3.1416/180) : " <<  cos(20*3.1416/180) << endl;
  // //   cout << "Result : " << result << endl;
  // //   cout << " Left Curve Line" << endl;
  // // }
  // // else
  // // {
  // //   cout << "Result : " << result << endl;
  // //   cout << " Straight Line" << endl;
  // // }
  // cout << "*************************************************" << endl;

  imshow("img_result",imgResult);
}

void canny(Mat &left_bi, Mat &right_bi)
{
  Canny(left_bi,left_bi,150,270,5);
  Canny(right_bi,right_bi,150,270,5);
}

void houghlinesP(Mat &left_img, Mat &right_img, vector<Vec4i> &left_lines, vector<Vec4i> &right_lines)
{
  HoughLinesP(left_img, left_lines, 1, CV_PI/180, 5, 5, 0 );

  HoughLinesP(right_img, right_lines, 1, CV_PI/180, 5, 5, 0 );
}

void color_threshold(Mat src)
{
  // Create Mat class
  Mat hls_img, lab_img;

  // Convert color space (rgb -> hsl , lab)
  cvtColor(src,hls_img,COLOR_BGR2HLS);
  cvtColor(src,lab_img,COLOR_BGR2Lab);

  // lab_threshold(lab_img);

}

int main(int argc, char **argv)
{

  // Define pub,sub
  ros::Publisher PubToCentralController;
  ros::Publisher PubToModeController;
  ros::Subscriber return_modeController;
  ros::Publisher LaneInfoPub;

  // init node
  ros::init(argc, argv, "lane_detector_node");
  ros::NodeHandle nh;

  // Capture the Video
  VideoCapture video("for_lane3.avi");

  // Dealing Exception
  if(!video.isOpened())
    printf("Failed to open the video\n");

  // FILE *fp;
  //
  // fp = fopen ("filename.txt","r");
  //
  // if (fp!=NULL)
  // {
  //   fscanf(fp,"Some String\n", &var);
  //   fclose (fp);
  // }

  Mat src,dst;

  int quit;

  video >> src;

  src_vertices[0] = Point(src.cols/2 - 200,350);      // upper left
  src_vertices[1] = Point(src.cols/2 + 200, 350);     // upper right
  src_vertices[2] = Point(src.cols, src.rows - 50);   // lower right
  src_vertices[3] = Point(0, src.rows - 50);          // lower left

  dst_vertices[0] = Point(0, 0);                      // upper left
  dst_vertices[1] = Point(src.cols, 0);               // upper right
  dst_vertices[2] = Point(src.cols, src.rows);        // lower right
  dst_vertices[3] = Point(0, src.rows);               // lower left


  // PubToModeController : int
  // PubToCentralController : int
  PubToModeController = nh.advertise<std_msgs::Int16>("return_signal", 1);
  PubToCentralController = nh.advertise<std_msgs::Int16>("return_signal",1);
  LaneInfoPub = nh.advertise<race::lane_info>("lane_info", 1);

  // 예시
  // race::lane_info msg;
  // LaneInfoPub.publish(msg);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {

    return_modeController = nh.subscribe("return_signal",1,Callback);

    // Get frame
    video >> src;


    // GaussianBlur
    Mat gaussian_img;

    GaussianBlur(src,gaussian_img,Size(3,3),0);

    // Bird eyes view transformation
    transform(src_vertices,dst_vertices,gaussian_img,dst);

    // Color Thresholding
    Mat color_filtered_img = convert_hsl(dst);

    // Gray Scaling
    Mat gray_img;

    cvtColor(color_filtered_img,gray_img,COLOR_BGR2GRAY);

    // Binarization
    Mat binary_img;

    threshold(gray_img, binary_img,0,255,THRESH_BINARY | THRESH_OTSU);

    // Dilated Img
    Mat dilated_img;

    dilate(binary_img,dilated_img,Mat());

    // Set ROI
    Mat left_img, right_img;

    if(!is_left)
    {
      // Default ROI
      set_left_roi(dilated_img, left_img);
    }
    else if(is_left)
    {

    }

    if(!is_right)
    {
      set_right_roi(dilated_img, right_img);
    }
    else if(is_right)
    {

    }

    // Operate Canny algorithm
    canny(left_img, right_img);

    // Operate HoughLinesP
    vector<Vec4i> left_lines;
    vector<Vec4i> right_lines;

    houghlinesP(left_img, right_img, left_lines, right_lines);

    // Preprocessed data
    vector<Vec2i> left_lines_vec2i;
    vector<Vec2i> right_lines_vec2i;

    // array of real line
    double real_left_line_x[10000];
    double real_left_line_y[10000];

    int real_left_line_idx = 0;
    int real_right_line_idx = 0;

    double real_right_line_x[10000];
    double real_right_line_y[10000];

    cv::Mat imgResult(src.rows, src.cols,CV_8UC3) ;
    imgResult = cv::Scalar(0);

    // Slope Thresholding & Raw Data Preprocessing
    real_left_line_idx = data_preprocess(dst,left_lines, real_left_line_x, real_left_line_y, left_lines_vec2i,0);
    real_right_line_idx = data_preprocess(dst,right_lines, real_right_line_x, real_right_line_y, right_lines_vec2i,1);

    // ransac
    if(real_left_line_idx > 20)
    {
      is_left = true;
      ransac(dst,imgResult,real_left_line_idx,real_left_line_x, real_left_line_y,0);
    }
    else
    {
      is_left = false;
    }
    if(real_right_line_idx > 20)
    {
      // is_right = true;
      ransac(dst,imgResult,real_right_line_idx,real_right_line_x, real_right_line_y,1);
    }
    else
    {
        // is_right = false;
    }

    // Showing the Img
    imshow("src",src);
    imshow("dst",dst);

    quit = waitKey(27);

    if(quit == 27)
    {
      waitKey(1000000);
      continue;
    }


    std_msgs::Int16 msg;
    // return_modeController : mode_number

    // line_info = {left_slope, right_slope, is_left, is_right,mid_left_point_dis, mid_right_point_dis}

    PubToModeController.publish(msg);
    PubToCentralController.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
