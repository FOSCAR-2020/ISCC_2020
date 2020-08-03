#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <iostream>

using namespace cv;
using namespace std;

const Vec3b HSV_RED_LOWER = Vec3b(0, 100, 100);
const Vec3b HSV_RED_UPPER = Vec3b(10, 255, 255);
const Vec3b HSV_RED_LOWER1 = Vec3b(160, 100, 100);
const Vec3b HSV_RED_UPPER1 = Vec3b(190, 255, 255);

const Vec3b HSV_GREEN_LOWER = Vec3b(40, 200, 30);
const Vec3b HSV_GREEN_UPPER = Vec3b(120, 255, 255);

const Vec3b HSV_YELLOW_LOWER = Vec3b(10, 70, 130);
const Vec3b HSV_YELLOW_UPPER = Vec3b(50, 255, 255);


const Vec3b HSV_BLACK_LOWER = Vec3b(0, 0, 0);
const Vec3b HSV_BLACK_UPPER = Vec3b(180, 255, 50);

const int MAX_SIZE = 230;
const int MIN_SIZE = 50;
const int MAX_HEIGHT = 50;
const int MIN_HEIGHT = 20;

bool use_roi = false;

//Contour 영역 내에 텍스트 쓰기
//https://github.com/bsdnoobz/opencv-code/blob/master/shape-detect.cpp
void setLabel(Mat& image, string str, vector<Point> contour)
{
    int fontface = FONT_HERSHEY_SIMPLEX;
    double scale = 0.5;
    int thickness = 1;
    int baseline = 0;

    Size text = getTextSize(str, fontface, scale, thickness, &baseline);
    Rect r = boundingRect(contour);

    Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
    rectangle(image, pt + Point(0, baseline), pt + Point(text.width, -text.height), CV_RGB(200, 200, 200), FILLED);
    putText(image, str, pt, fontface, scale, CV_RGB(0, 0, 0), thickness, 8);
}

struct TrafficLight {
    int left;
    int top;
    int width;
    int height;
    bool red_on = false;
    bool yellow_on = false;
    bool left_on = false;
    bool green_on = false;
    int state = 0;
    TrafficLight() {

    }
    TrafficLight(int l, int t, int w, int h, int s) : left(l), top(t), width(w), height(h), state(s) {
        if(s == 0)
            red_on = true;
        if(s == 1)
            yellow_on = true;
        if(s == 2)
            left_on = true;
        if(s == 3)
            green_on = true;
    }
};

int main(int argc, char** argv) {
    VideoCapture cap("test.mov");
    if (!cap.isOpened()) return -1;

    while(1) {
        Mat img;
        cap >> img;

        vector<TrafficLight> v;

        int rows = img.rows;
        int cols = img.cols;

        Rect rect(0, 0, cols, rows/2);

        Mat org_img = img(rect);

        Mat light_off;
        org_img.copyTo(light_off);

        Mat binary_red, binary_yellow, binary_green;

        Mat hsv_img;
        cvtColor(org_img, hsv_img, COLOR_BGR2HSV);

        imshow("hsv", hsv_img);
        // RED Detect
        Mat binaryImg1;
        Mat binaryImg2;
        inRange(hsv_img, HSV_RED_LOWER, HSV_RED_UPPER, binaryImg1);
        inRange(hsv_img, HSV_RED_LOWER1, HSV_RED_UPPER1, binaryImg2);

        binary_red = binaryImg1 | binaryImg2;

//        dilate(binary_red, binary_red, Mat());

        imshow("red", binary_red);

        Mat red_img_labels, red_stats, red_centroids;
        int red_num;

        red_num = connectedComponentsWithStats(binary_red, red_img_labels, red_stats, red_centroids, 8, CV_32S);
        for(int i = 1; i < red_num; i++) {
            int area = red_stats.at<int>(i, CC_STAT_AREA);
            int left = red_stats.at<int>(i, CC_STAT_LEFT);
            int top = red_stats.at<int>(i, CC_STAT_TOP);
            int width = red_stats.at<int>(i, CC_STAT_WIDTH);
            int height = red_stats.at<int>(i, CC_STAT_HEIGHT);
            if((float)width / (float)height > 1.5)
                continue;
            if(width < 20)
                continue;
            width += 30;
            left -= 10;

            Rect rc(left,top,width,height);
            rectangle(light_off, rc, Scalar(0, 0, 0), FILLED);
        }


        // YELLOW Detect
        inRange(hsv_img, HSV_YELLOW_LOWER, HSV_YELLOW_UPPER, binary_yellow);

//        dilate(binary_yellow, binary_yellow, Mat());

        Mat yel_img_labels, yel_stats, yel_centroids;
        int yel_num;

        yel_num = connectedComponentsWithStats(binary_yellow, yel_img_labels, yel_stats, yel_centroids, 8, CV_32S);

        for(int i = 1; i < yel_num; i++) {
            int area = yel_stats.at<int>(i, CC_STAT_AREA);
            int left = yel_stats.at<int>(i, CC_STAT_LEFT);
            int top = yel_stats.at<int>(i, CC_STAT_TOP);
            int width = yel_stats.at<int>(i, CC_STAT_WIDTH);
            int height = yel_stats.at<int>(i, CC_STAT_HEIGHT);
            if((float)width / (float)height > 1.5)
                continue;
            if(width < 20)
                continue;
            width += 30;
            left -= 15;

            Rect rc(left,top,width,height);
            rectangle(light_off, rc, Scalar(0, 0, 0), FILLED);
        }

        imshow("yellow", binary_yellow);

        // GREEN Detect
        inRange(hsv_img, HSV_GREEN_LOWER, HSV_GREEN_UPPER, binary_green);

        dilate(binary_green, binary_green, Mat());

        Mat green_img_labels, green_stats, green_centroids;
        int green_num;

        green_num = connectedComponentsWithStats(binary_green, green_img_labels, green_stats, green_centroids, 8, CV_32S);
        for(int i = 1; i < green_num; i++) {
            int area = green_stats.at<int>(i, CC_STAT_AREA);
            int left = green_stats.at<int>(i, CC_STAT_LEFT);
            int top = green_stats.at<int>(i, CC_STAT_TOP);
            int width = green_stats.at<int>(i, CC_STAT_WIDTH);
            int height = green_stats.at<int>(i, CC_STAT_HEIGHT);
            if((float)width / (float)height > 1.5)
                continue;
            if(width < 20)
                continue;
            left -= 20;
            width += 20;

            Rect rc(left,top,width,height);
            rectangle(light_off, rc, Scalar(0, 0, 0), FILLED);
        }


        imshow("green", binary_green);

        imshow("light_off", light_off);


        Mat binary_light_off;
        cvtColor(light_off, light_off, COLOR_BGR2GRAY);
        threshold(light_off, binary_light_off, 100, 255, THRESH_BINARY_INV);

        imshow("binary_light_off", binary_light_off);

        // RED Detection
        for(int j = 1; j < red_num; j++) {
            int area = red_stats.at<int>(j, CC_STAT_AREA);
            int left = red_stats.at<int>(j, CC_STAT_LEFT);
            int top = red_stats.at<int>(j, CC_STAT_TOP);
            int width = red_stats.at<int>(j, CC_STAT_WIDTH);
            int height = red_stats.at<int>(j, CC_STAT_HEIGHT);
            for(int start = left+width; start < left + width * 6; start ++) {
                if(start == binary_light_off.cols - 1)
                    break;
                if(binary_light_off.at<uchar>(top+height/3, start) == 0) {
//                    width = start - left;
                    break;
                }
                width ++;
            }

            if(use_roi) {
                if(width > MAX_SIZE || width < MIN_SIZE || height > MAX_HEIGHT || height < MIN_HEIGHT)
                    continue;
            }

            v.push_back(TrafficLight(left, top, width, height, 0));
        }

        // YELLOW Detection
        for(int j = 1; j < yel_num; j++) {
            int area = yel_stats.at<int>(j, CC_STAT_AREA);
            int left = yel_stats.at<int>(j, CC_STAT_LEFT);
            int top = yel_stats.at<int>(j, CC_STAT_TOP);
            int width = yel_stats.at<int>(j, CC_STAT_WIDTH);
            int height = yel_stats.at<int>(j, CC_STAT_HEIGHT);

            for(int start = left+width; start < left + width * 4; start ++) {
                if(start == binary_light_off.cols - 1)
                    break;
                if(binary_light_off.at<uchar>(top+height/2, start) == 0) {
                    break;
                }
                width ++;
            }

            for(int start = left; start > left - width * 2; start--) {
                if(start == 0)
                    break;
                if(binary_light_off.at<uchar>(top+height/3, start) == 0) {
                    width += left - start;
                    left = start;
                    break;
                }
            }

            if(use_roi) {
                if(width > MAX_SIZE || width < MIN_SIZE || height > MAX_HEIGHT || height < MIN_HEIGHT)
                    continue;
            }

            bool familiar = false;

            for(int i = 0; i < v.size(); i++) {
                TrafficLight tl = v[i];
                if(abs(tl.left - left) < 30 && abs(tl.top - top) < 30) {
                    tl.yellow_on = true;
                    familiar = true;
                }

            }
            if(!familiar) {
                v.push_back(TrafficLight(left, top, width, height, 1));
            }
        }

        // GREEN Detection
        for(int j = 1; j < green_num; j++) {
            int area = green_stats.at<int>(j, CC_STAT_AREA);
            int left = green_stats.at<int>(j, CC_STAT_LEFT);
            int top = green_stats.at<int>(j, CC_STAT_TOP);
            int width = green_stats.at<int>(j, CC_STAT_WIDTH);
            int height = green_stats.at<int>(j, CC_STAT_HEIGHT);

            int org_width = width;

            for(int start = left; start > left - width * 5; start --) {
                if(start == 0)
                    break;
                if(binary_light_off.at<uchar>(top+height/3, start) == 0) {
                    width += left - start;
                    left = start;
                    break;
                }
            }

            int delta = 0;
            for(int start = left+width; start < left + width * 4; start ++) {
                if(start == binary_light_off.cols - 1)
                    break;
                if(binary_light_off.at<uchar>(top+height/2, start) == 0) {
                    break;
                }
                width ++;
                delta++;
            }

            if(use_roi) {
                if(width > MAX_SIZE || width < MIN_SIZE || height > MAX_HEIGHT || height < MIN_HEIGHT)
                    continue;
            }

            bool familiar = false;

            for(int i = 0; i < v.size(); i++) {
                TrafficLight tl = v[i];
                if(abs(tl.left - left) < 30 && abs(tl.top - top) < 30) {
                    if(delta > org_width/2) {
                        v[i].left_on = true;
                    }
                    else
                        v[i].green_on = true;
                    familiar = true;
                }

            }
            if(!familiar) {
                // 좌회전 불
                if(delta > org_width/3)
                    v.push_back(TrafficLight(left, top, width, height, 2));
                else
                    v.push_back(TrafficLight(left, top, width, height, 3));
            }
        }

        for(int i = 0; i < v.size(); i++) {
            rectangle(org_img, Point(v[i].left, v[i].top), Point(v[i].left+v[i].width, v[i].top+v[i].height), Scalar(0,255,0), 3);
            cout << v[i].red_on << " " << v[i].yellow_on << " " << v[i].left_on << " " << v[i].green_on << endl;
        }

        imshow("org_img", org_img);
        waitKey(0);
        if (waitKey(5) >= 0)
            break;


    }
}
