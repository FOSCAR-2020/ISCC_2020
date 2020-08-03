/* 2019-08-28 금요일
 * 20153155 김다훈
 * 20153183 박호준
 * mode_controller.cpp 예선
 */

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <nav_msgs/Odometry.h>
#include "race/mode.h"



struct Point {
    float x;
    float y;
    Point() {x = 0; y = 0;}
    Point(float _x, float _y) : x(_x), y(_y) {}
};


std_msgs::Int8 pstatus;
std_msgs::Int8 pmode;
std_msgs::Int8 pspd_limit;
Point current_position;
int mission_num; // 예선전 미션 숫자 목록 저장 변수


// 미션목록
enum MISSION{

    BASE,                   //0. 일반주행
    TURN_LEFT,              //1. 교차로 좌회전
    STATIC_OBSTACLE,        //2. 정적 장애물(드럼통)
    TURN_RIGHT,             //3. 교차로 우회전
    GO_STRAIGHT_TRAFFIC,    //4. 교차로 직진(신호등)
    TURN_LEFT_TRAFFIC,      //5. 교차로 좌회전(신호등)
    DYNAMIC_OBSTACLE,       //6. 돌발 장애물/일시정지
    TURN_LEFT_TRAFFIC_2,    //7. 교차로 좌회전(신호등)
    GO_STRAIGHT_TRAFFIC_2,  //8. 교차로 직진(신호등)
    GO_STRAIGHT,            //9. 교차로 직진
    PARKING,                //10 .주차

};


// 표지판 신호
enum TRAFFIC_SIGN{

    LEFT_SIGN, // 좌회전 표지판
    RIGHT_SIGN, //우회전 표지판
    STATIC_OBSTACLE_SIGN, // 공사중(정적 장애물)표지판
    DYNAMIC_OBSTACLE_SIGN, // 자전거(동적 장애물)표지판
    FOUR_DISTANCE_SIGN, // 4거리 표지판
    KIDS_ZONE_SIGN, //어린이 보호구역 표지판
    PARKING_SIGN, // 주차 표지판

};

int left_sign_cnt = 0; // 좌회전 표지판 카운트가 1,2일땐 비신호, 3일땐 신호
int go_straight_sign_cnt = 0; // 직진 표지판 카운트가 1,2 일땐 신호, 3일땐 비신호

void traffic_sign_callback(std_msgs::UInt8 msg) { // 표지판 메세지 콜백 함수

    std_msgs::UInt8 ts_flag;
   
    ts_flag.data = 0;
    ts_flag.data = msg.data;

    
    if(ts_flag.data == LEFT_SIGN && left_sign_cnt == 1){                       
        mission_num = TURN_LEFT; // 첫번째 좌회전 표지판에선 비신호 좌회전
        left_sign_cnt++;
   }
    else if(ts_flag.data == LEFT_SIGN && left_sign_cnt == 2){          
        mission_num = TURN_LEFT_TRAFFIC; // 두번째 좌회전 표지판에선 신호 좌회전
   }
    else if(ts_flag.data == LEFT_SIGN && left_sign_cnt == 3){          
        mission_num = TURN_LEFT_TRAFFIC_2; // 세번째 좌회전 표지판에선 신호 좌회전
   }
    else if(ts_flag.data == RIGHT_SIGN){ // 우회전 표지판
        mission_num = TURN_RIGHT;
    }
    else if(ts_flag.data == STATIC_OBSTACLE_SIGN){ // 공사중(정적장애물)표지판
        mission_num = STATIC_OBSTACLE;
    }
    else if(ts_flag.data == DYNAMIC_OBSTACLE_SIGN){ // 자전거(동적장애물)표지판
        mission_num = DYNAMIC_OBSTACLE;
    }
    else if(ts_flag.data == KIDS_ZONE_SIGN){ //어린이 보호구역 표지판
        pspd_limit.data = 30; // 제한속도 30
    }
    else if(ts_flag.data == FOUR_DISTANCE_SIGN && left_sign_cnt != 3){   
        //사거리 표지판, 신호 직진       
        mission_num = GO_STRAIGHT_TRAFFIC;
   }
    else if(ts_flag.data == FOUR_DISTANCE_SIGN && left_sign_cnt == 3){          
        //사거리 표지판, 비신호 직진
        mission_num = GO_STRAIGHT;
   }
        else if(ts_flag.data == PARKING_SIGN){ //주차 표지판         
        mission_num = PARKING;
   }
}

/* 청신호 000X
 * 좌회전 00X0
 * 황신호 0X00
 * 적신호 X000
 */
void traffic_light_callback(std_msgs::UInt8 msg) { // 신호등 메세지 콜백 함수
    std_msgs::UInt8 tl_flag;
    tl_flag.data = 0b00001111;
    tl_flag.data = tl_flag.data & msg.data;

    if(tl_flag.data == 1) { //청신호 0001
        
        pstatus.data = GO_STRAIGHT_TRAFFIC;

    }


    else if(tl_flag.data == 10 && left_sign_cnt == 2){ //적, 좌회전 1010

        pstatus.data = 1;
        mission_num = TURN_LEFT_TRAFFIC;

    }

    else if(tl_flag.data == 3 && left_sign_cnt == 3) { //청, 좌회전 0011
        
        pstatus.data = 1;
        mission_num = TURN_LEFT_TRAFFIC_2;

    }

    // 황,적신호 1100
    // 적신호    1000
    // 황신호    0100
    else {
        pstatus.data = 0;
    }
}

void odom_callback(nav_msgs::Odometry msg) { // Odom 메세지 콜백 함수
    // Odom에서 진행중인 미션의 완료를 판단
/* 미션 목록
 * 0. 일반주행
 * 1. 교차로 좌회전
 * 2. 정적 장애물(드럼통)
 * 3. 교차로 우회전
 * 4. 교차로 직진(신호등)
 * 5. 교차로 좌회전(신호등))
 * 6. 돌발 장애물/일시정지
 * 7. 교차로 좌회전(신호등)
 * 8. 교차로 직진(신호등)
 * 9. 교차로 직진
 * 10.주차
 */
    

    current_position.x = msg.pose.pose.position.x;
    current_position.y = msg.pose.pose.position.y;


/* 미션 */


}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mode_controller_node");

    pstatus.data = 1;
    pmode.data = 0;
    mission_num = BASE;

    ros::NodeHandle nh;
    race::mode m;

    ros::Subscriber traffic_sign_sub = nh.subscribe("traffic_sign", 1, traffic_sign_callback);
    ros::Subscriber traffic_light_sub = nh.subscribe("traffic_light", 1, traffic_light_callback);
    ros::Subscriber odom_sub = nh.subscribe("odom", 1, odom_callback);

    ros::Publisher mode_pub = nh.advertise<race::mode>("mode", 1);


    // TODO 표지판, 신호등, 차선 정보 가공
    if(pstatus.data) {
        if(mission_num == BASE|| mission_num == GO_STRAIGHT_TRAFFIC || mission_num == GO_STRAIGHT) {
            m.mode = 0; // 일반주행
        } 
        else if(mission_num == TURN_LEFT || mission_num == TURN_LEFT_TRAFFIC || mission_num == TURN_LEFT_TRAFFIC_2) {
            m.mode = 1; // 좌회전
        }
        else if(mission_num == TURN_RIGHT) {
            m.mode = 2; // 우회전
        }
        else if(mission_num == DYNAMIC_OBSTACLE) {
            m.mode = 3; // 동적장애물
        }
        else if(mission_num == STATIC_OBSTACLE) {
            m.mode = 4; // 정적 장애물
        }
        else if(mission_num == PARKING) {
            m.mode = 6; // 주차
        }

    }

    // mode 발행
    m.status = pstatus.data;
    m.spd_limit = pspd_limit.data;
    mode_pub.publish(m);

    ros::spin();
    return 0;
}

/*
mode msg
  status
0 : 정지
1 : 진행 
   mode
0 : 기본 주행
1 : 좌회전
2 : 우회전
3 : 동적 장애물
4 : 정적 장애물
5 : 차선변경
6 : 주차
*/

