#include <iostream>
#include <vector>
#include <cmath>
#include "opencv2/opencv.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"



////////////////////////////////////////////////////////////////////////////////////////
// g++ Bird_line_detect.cpp -o main $(pkg-config opencv --libs --cflags) -lgsl -lcblas//
// ./main                                                                             //
////////////////////////////////////////////////////////////////////////////////////////
using namespace std;
using namespace cv;

void transform(Point2f* src_vertices, Point2f* dst_vertices, Mat& src, Mat &dst){
    Mat M = getPerspectiveTransform(src_vertices, dst_vertices);
    warpPerspective(src, dst, M, dst.size(), INTER_LINEAR, BORDER_CONSTANT);
}
void cal_radian(Point p1, Point p2){
    float car_x = 0;
    float car_y = -1;
    float edge_x = p2.x-p1.x;
    float edge_y = p2.y-p1.y;
    double r = sqrt(edge_x * edge_x + edge_y * edge_y);
    float line_x = edge_x / r;
    float line_y = edge_y / r;
    double result = car_x * line_x + car_y * line_y;
    if(cos(10 * 3.1416/ 180)>result){
        cout<<"curve"<<endl;
    }
    else{
        cout<<"straight"<<endl;
    }
    
    
}
Mat remove_noise(Mat src)
{
    Mat output;
    
    GaussianBlur(src, output, cv::Size(3, 3), 0, 0);
    
    return output;
}

void enhancer(Mat &src, Mat &dst)
{
    dst = src.clone();
    dst = Scalar(0);
    
    int width = src.cols;
    int height = src.rows;
    
    double max,min;
    
    minMaxLoc(src, &min, &max);
    
    dst = (src - min) * (255.0 / (max-min));
    
    //cout << max << ' ' << min << endl;
}

bool draw_circle(Mat &dst, Point p1, Point p2, Point &s, Point &e)
{
    if(p1.x == p2.x)
    {
        // s = p1;
        // e = p2;
        
        return false;
    }
    double slope = (p1.y-p2.y)/(p1.x-p2.x);
    double b = -slope*p1.x+p1.y;
    
    Point2d start;
    Point2d end;
    
    start.x = -b/slope;
    start.y = 0;
    
    end.x = (-b + dst.rows)/slope;
    end.y = dst.rows;
    
    if(start != end)
    {
        s = start;
        e = end;
    }
    return true;
}

void Left_ROI(Mat& img, Mat& img_ROI){
    Point a = Point(0, 0);
    Point b = Point(img.cols/2, 0);
    Point c = Point(img.cols/2, img.rows);
    Point d = Point(0, img.rows);
    
    vector <Point> Left_Point;
    
    Left_Point.push_back(a);
    Left_Point.push_back(b);
    Left_Point.push_back(c);
    Left_Point.push_back(d);
    
    
    Mat roi(img.rows, img.cols, CV_8U, Scalar(0));
    
    fillConvexPoly(roi, Left_Point, Scalar(255));
    
    Mat filteredImg_Left;
    img.copyTo(filteredImg_Left, roi);
    
    img_ROI = filteredImg_Left.clone();
    
    // imshow("Img_ROI", img_ROI);
}

void Left_ROI(Mat& img, Mat& img_ROI, Point start, Point end){
    Point a = Point(0, 0);
    // Point b = Point(img.cols/3-50, 0);
    // Point c = Point(img.cols/3-50, img.rows);
    Point b = Point(start.x+100, 0);
    Point c = Point(end.x+100, img.rows);
    Point d = Point(0, img.rows);
    
    vector <Point> Left_Point;
    
    Left_Point.push_back(a);
    Left_Point.push_back(b);
    Left_Point.push_back(c);
    Left_Point.push_back(d);
    
    
    Mat roi(img.rows, img.cols, CV_8U, Scalar(0));
    
    fillConvexPoly(roi, Left_Point, Scalar(255));
    
    Mat filteredImg_Left;
    img.copyTo(filteredImg_Left, roi);
    
    img_ROI = filteredImg_Left.clone();
    
    //imshow("Img_left_ROI", img_ROI);
    // imshow("Img_ROI", img_ROI);
}

void Left_ROI2(Mat& img, Mat& img_ROI){
    Point a = Point(0, 0);
    // Point b = Point(img.cols/3-50, 0);
    // Point c = Point(img.cols/3-50, img.rows);
    Point b = Point(img.cols, img.rows);
    Point c = Point(img.cols, img.rows);
    Point d = Point(0, img.rows);
    
    vector <Point> Left_Point;
    
    Left_Point.push_back(a);
    Left_Point.push_back(b);
    Left_Point.push_back(c);
    Left_Point.push_back(d);
    
    
    Mat roi(img.rows, img.cols, CV_8U, Scalar(0));
    
    fillConvexPoly(roi, Left_Point, Scalar(255));
    
    Mat filteredImg_Left;
    img.copyTo(filteredImg_Left, roi);
    
    img_ROI = filteredImg_Left.clone();
    
    //imshow("Img_left_ROI", img_ROI);
}

void Right_ROI(Mat& img, Mat& img_ROI){
    Point a = Point(img.cols/2, 0);
    // Point b = Point(img.cols/3-50, 0);
    // Point c = Point(img.cols/3-50, img.rows);
    Point b = Point(img.cols, 0);
    Point c = Point(img.cols, img.rows);
    Point d = Point(img.cols/2, img.rows);
    
    vector <Point> Right_Point;
    
    Right_Point.push_back(a);
    Right_Point.push_back(b);
    Right_Point.push_back(c);
    Right_Point.push_back(d);
    
    
    Mat roi(img.rows, img.cols, CV_8U, Scalar(0));
    
    fillConvexPoly(roi, Right_Point, Scalar(255));
    
    Mat filteredImg_Right;
    img.copyTo(filteredImg_Right, roi);
    
    img_ROI = filteredImg_Right.clone();
    
    //imshow("Img_ROI", img_ROI);
}
void Right_ROI2(Mat& img, Mat& img_ROI){
    Point a = Point(img.cols/2, 0);
    // Point b = Point(img.cols/3-50, 0);
    // Point c = Point(img.cols/3-50, img.rows);
    Point b = Point(img.cols, 0);
    Point c = Point(img.cols, img.rows);
    Point d = Point(img.cols/2, img.rows);
    
    vector <Point> Right_Point;
    
    Right_Point.push_back(a);
    Right_Point.push_back(b);
    Right_Point.push_back(c);
    Right_Point.push_back(d);
    
    
    Mat roi(img.rows, img.cols, CV_8U, Scalar(0));
    
    fillConvexPoly(roi, Right_Point, Scalar(255));
    
    Mat filteredImg_Right;
    img.copyTo(filteredImg_Right, roi);
    
    img_ROI = filteredImg_Right.clone();
    
    //imshow("Img_ROI2", img_ROI);
}
int main()
{
    VideoCapture video("test.mp4");
    
    Mat src;
    Mat gaussian_img;
    Mat lab_img;
    
    Mat hls_img;
    Mat enhanced_img;
    Mat hls_enhanced_img;
    Mat binary_img;
    Mat hls_binary_img;
    Mat canny_img;
    Mat hls_canny_img;
    
    Mat left_img,right_img;
    //Mat result;
    Point2d temp1,temp2,temp3,temp4;
    Point2d pt1,pt2,pt3,pt4;
    Point left_start,left_end;
    Point temp_left_start,temp_left_end;
    
    Point temp_right_start,temp_right_end;
    Point right_start,right_end;
    
    bool first = true;
    bool rfirst = true;
    bool notfound = false;
    bool isVert = false;
    bool rnotfount = false;
    
    int quit;
    
    Point2f src_vertices[4];
    
    //video >> src;
    
    src = imread("aa.png");
    cout << src.rows << ' ' << src.cols << endl;
    src_vertices[0] = Point(52,62); // 좌상
    src_vertices[1] = Point(266,62); // 우상
    src_vertices[2] = Point(320,97); // 우하
    src_vertices[3] = Point(0,97); // 좌하
    
    Point2f dst_vertices[4];
    
    dst_vertices[0] = Point(0, 0);
    dst_vertices[1] = Point(320, 0);
    dst_vertices[2] = Point(320, 180);
    dst_vertices[3] = Point(0, 180);
    
    while(1)
    {
        //video >> src;
        Mat M = getPerspectiveTransform(src_vertices, dst_vertices);
        Mat dst(180, 320, CV_8UC3);
        int width = dst.cols;
        int height = dst.rows;
        warpPerspective(src, dst, M, dst.size(), INTER_LINEAR, BORDER_CONSTANT);
        gaussian_img = remove_noise(dst);
        
        cvtColor(gaussian_img,lab_img,COLOR_BGR2Lab);
        cvtColor(gaussian_img,hls_img,COLOR_BGR2Lab);
        vector<Mat> lab_plane;
        vector<Mat> hls_plane;
        split(hls_img, hls_plane);
        split(lab_img,lab_plane);
        
        // imshow("B",lab_plane[2]);
        
        enhancer(lab_plane[2],enhanced_img);
        enhancer(hls_plane[2],hls_enhanced_img);
        threshold(enhanced_img,binary_img,120,255,THRESH_BINARY);
        threshold(hls_enhanced_img,hls_binary_img,120,255,THRESH_BINARY);
        
        Canny(binary_img,canny_img,50,150,5);
        Canny(hls_binary_img, hls_canny_img,150,300,5);
        
        if(first)
        {
            Left_ROI(canny_img,left_img);
            
        }
        else if(notfound)
        {
            Left_ROI2(canny_img,left_img);
            
        }
        else
        {
            Left_ROI(canny_img,left_img,left_start,left_end);
        }
        //Left_ROI(canny_img,left_img);
        
        if(rfirst)
        {
            Right_ROI(hls_canny_img,right_img);
        }
        else if(rnotfount)
        {
            Right_ROI2(hls_canny_img,right_img);
        }
        
        // else if()
        // {
        //
        // }
        
        // left_img = canny_img(rect1;
        // right_img = canny_img(rect2);
        
        
        vector<Vec2f> linesLeft;
        vector<Vec2f> linesRight; // will hold the results of the detection
        
        HoughLines(left_img, linesLeft, 1, CV_PI/180, 60,0, 0,0,CV_PI/2); // runs the actual detection
        HoughLines(right_img,linesRight,1, CV_PI/180, 60,0,0,0,CV_PI/2);    // img, line, rho, theta, threshold, srn, stn, min_theta, max_theta
        //sort(linesLeft.begin(),linesLeft.end());
        //linesLeft.erase(unique(linesLeft.begin(),linesLeft.end()),linesLeft.end());
        
        int countRight = linesRight.size();
        int countLeft = linesLeft.size();
        
        //      line(dst, pt3, pt4, Scalar(255,0,0), 4, CV_AA);
        
        
        int left_top_x = 0;
        int left_top_y = 0;
        int left_bot_x = 0;
        int left_bot_y = 0;
        int right_top_x = 0;
        int right_top_y = 0;
        int right_bot_x = 0;
        int right_bot_y = 0;
        for( size_t i = 0; i < linesRight.size(); i++ )
        {
            float rho = linesRight[i][0], theta = linesRight[i][1];
            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;
            pt3.x = cvRound(x0 + 1000*(-b));
            pt3.y = cvRound(y0 + 1000*(a));
            pt4.x = cvRound(x0 - 1000*(-b));
            pt4.y = cvRound(y0 - 1000*(a));
            
            right_top_x += pt3.x;
            right_top_y += pt3.y;
            right_bot_x += pt4.x;
            right_bot_y += pt4.y;
            
            //line(dst, pt3, pt4, Scalar(0,0,255), 2, CV_AA);
        }
        
        for( size_t i = 0; i < linesLeft.size(); i++ )
        {
            float rho = linesLeft[i][0], theta = linesLeft[i][1];
            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;
            pt3.x = cvRound(x0 + 1000*(-b));
            pt3.y = cvRound(y0 + 1000*(a));
            pt4.x = cvRound(x0 - 1000*(-b));
            pt4.y = cvRound(y0 - 1000*(a));
            
            
            left_top_x += pt3.x;
            left_top_y += pt3.y;
            left_bot_x += pt4.x;
            left_bot_y += pt4.y;
            //line(dst, pt3, pt4, Scalar(255,0,0), 2, CV_AA);
        }
        left_top_x /= countLeft;
        
        left_top_y /=countLeft;
        left_bot_x /=countLeft;
        left_bot_y /= countLeft;
        
        right_top_x /= countRight;
        right_top_y /=countRight;
        right_bot_x /=countRight;
        right_bot_y /= countRight;
        
        line(dst, Point(right_top_x, right_top_y), Point(right_bot_x, right_bot_y), Scalar(255,0,0), 2, LINE_AA);
        line(dst, Point(left_top_x, left_top_y), Point(left_bot_x, left_bot_y), Scalar(0,0,255), 2, LINE_AA);
        
        // y변화량 / x 변화량
        float dx = float(left_bot_x - left_top_x);
        float dy = float(left_bot_y - left_top_y);
        dy *= -1;
        if(dx != 0) {
            cout << "기울기 " << dy / dx << endl;

        }
        else {
            cout << "기울기 1 " << endl;
        }
        
        cout << (atan(1/(dy/dx))*180/M_PI)*10 << endl;
        
        imshow("src", src);
        imshow("left",left_img);
        imshow("DST",dst);
        imshow("Bi2",binary_img);
        imshow("right",right_img);
        //imshow("result", result);
        //imshow("Left", left_img);
        quit = waitKey(33);
        
        if(quit == 27)
        {
            waitKey(100000);
        }
        
    }
    
}
