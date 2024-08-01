// gimbal_control.cpp
#include "gimbal_control.h"
#include <Servo.h>
#include "Motor-Control.h"

extern Servo myservo;

// 滴灌命令
extern uint16_t water_status;

//抓取命令
extern uint16_t crawl_status;

//坐标指令标志
extern uint16_t Normal_Crawl[2];
extern uint16_t Defridence[2];

int x_last = 2000;                          //x轴基于上一次需要移动的位置
int y_last = 2000;                          //y轴基于上一次需要移动的位置
int drip_timg = 500;                        //滴灌时间
int nomal_status = 0;                       //常态滴灌状态
int different_status = 0;                   //异层滴灌状态
uint16_t hand_on = 50;                     //机械手开
uint16_t hand_off = 0;                     //机械手关
int crawl_First_hight[2] = {2, 4};      //第一层机械手抓取和运动的安全高度
int crawl_Second_hight[2] = {8, 10};      //第二层机械手抓取和运动的安全高度
uint8_t First_flag = 0;
uint8_t Second_flag = 0;

int Middle_aisle[4] = {0, 14500 };                                      //仅记住中间点的X,Y坐标，Z的坐标需要根据到时候在那一层来取反。
int First_normal[7] = {0, 0, 1600, 1600, 4800, 4800};             //第一层常态化区域的其中两盆需要抓取的坐标。x,y.x,y,z为常规高度
int Second_normal[7] = {1600, 1600, 4800, 4800};                  //第二层常态化区域的其中两盆需要抓取的坐标。x,y.x,y,z
int First_different[5] = {1600, 6*1600, 4800, 6*4800, 3200};            //第一层异层区域的其中两盆需要抓取的坐标。x,y.x,y,z
int Second_different[5] = {1600, 6*1600, 4800, 6*4800, 6400};           //第二层异层区域的其中两盆需要抓取的坐标。x,y.x,y,z
int first_crawl_flag[4] = {1, 1, 0, 0};                                 //第一层抓取标志,前两位为常态化区域的抓取标志，后两位为异层区域的抓取标志
int Second_crawl_flag[4] = {1, 1, 0, 0};                                //第二层抓取标志，前两位为常态化区域的抓取标志，后两位为异层区域的抓取标志
//箱子状态,FF 抓取状态 滴灌状态 FE
uint8_t Motor_Status[4] = {0};


void crawl_drip_control()
{
    if(!water_status && crawl_status)
    {
        Motor_Status[1] = 0x01;
        Serial1.write(Motor_Status, 4);
        stepperZ.runToNewPosition(crawl_First_hight[1]);          //z轴上升到安全运动高度

            if(Normal_Crawl[0])
            {
                stepperX.runToNewPosition(First_normal[0]);        //x轴运动到对应位置
                stepperY.runToNewPosition(First_normal[1]);        //y轴运动到对应位置
                myservo.write(hand_on);                            //机械手开
                stepperZ.runToNewPosition(crawl_First_hight[0]);   //z轴下降到安全抓取高度
                myservo.write(hand_off);                           //机械手关
                stepperZ.runToNewPosition(crawl_First_hight[1]);   //z轴上升到安全运动高度

                stepperY.runToNewPosition(First_different[1]);     //镜像关系，x轴不动，y轴移动即可
                stepperZ.moveTo(crawl_First_hight[0]);             //z轴下降到安全抓取高度
                myservo.write(hand_on);                            //机械手开
                stepperZ.moveTo(crawl_First_hight[1]);             //z轴上升到安全运动高度
                myservo.write(hand_off);                           //机械手关

                first_crawl_flag[2] = 0x01;                        //常态化第一花盆已经抓取到差异化第一位置
                first_crawl_flag[0] = 0x00;
            }
            else if(Normal_Crawl[1])
            {
                stepperY.runToNewPosition(First_different[3]);  //Y轴去到第二盆常态化花盆
                stepperX.runToNewPosition(First_different[2]);  //X轴去到第二盆常态化花盆
                myservo.write(hand_on);                         //机械手开
                stepperZ.moveTo(crawl_First_hight[0]);          //z轴下降到安全抓取高度
                myservo.write(hand_off);                        //机械手关
                stepperZ.moveTo(crawl_First_hight[1]);          //z轴上升到安全运动高度
                stepperY.runToNewPosition(First_different[4]);  //Y轴去到第二盆异层花盆
                stepperX.runToNewPosition(First_different[3]);  //X轴去到第二盆异层花盆
                stepperZ.runToNewPosition(crawl_First_hight[0]);//z轴下降到安全抓取高度
                myservo.write(hand_on);
                stepperZ.runToNewPosition(crawl_First_hight[1]);//z轴上升到安全运动高度
                myservo.write(hand_off);                        //机械手关

                first_crawl_flag[3] = 0x01;                     //常态化第二花盆已经抓取到差异化第二位置
            }
            else if(Defridence[0])
            {
                //进入二层
                stepperX.runToNewPosition(Middle_aisle[0]);     //x轴运动到对应位置
                stepperY.runToNewPosition(Middle_aisle[1]);     //y轴移动到中间过道
                stepperZ.runToNewPosition(crawl_Second_hight[0]);//进入二层，z轴上升到二层安全高度抓取
                //二层抓取
                stepperX.runToNewPosition(Second_crawl_flag[0]);  //x轴运动到对应位置
                stepperY.runToNewPosition(Second_crawl_flag[1]);  //y轴运动到对应位置
                myservo.write(hand_on);                         //机械手开
                stepperZ.runToNewPosition(crawl_Second_hight[0]);//z轴下降到安全抓取
                myservo.write(hand_off);                         //机械手关
                stepperZ.runToNewPosition(crawl_Second_hight[1]);//z轴上升到安全运动位置
                stepperX.runToNewPosition(Second_crawl_flag[0]);    //x轴运动到对应位置
                stepperY.runToNewPosition(Second_crawl_flag[1]);    //y轴运动到对应位置
                myservo.write(hand_on);                             //机械手开
                stepperZ.runToNewPosition(crawl_Second_hight[0]);   //z轴下降到安全抓取
                myservo.write(hand_off);                            //机械手关
                stepperZ.runToNewPosition(crawl_Second_hight[1]);   //z轴上升到安全运动位置
                stepperX.runToNewPosition(Second_different[0]);     //x轴运动到二层差异区对应位置
                stepperY.runToNewPosition(Second_different[1]);     //y轴运动到二层差异区对应位置
                stepperZ.runToNewPosition(crawl_Second_hight[0]);   //z轴下降到安全抓取
                myservo.write(hand_on);                             //机械手开
                stepperZ.runToNewPosition(crawl_Second_hight[1]);   //z轴上升到安全运动位置
                myservo.write(hand_off);                            //机械手关

                Second_crawl_flag[2] = 0x01;                        //常态化二层第一个花盆已经抓到差异化第一位置
            }
            /*++++++++++++++++开始抓取二层常态化第二盆+++++++++++++++++++*/
            else if(Defridence[1])
            {
                stepperX.runToNewPosition(Second_normal[3]);     //x轴运动到对应位置
                stepperY.runToNewPosition(Second_normal[4]);     //y轴运动到对应位置 
                myservo.write(hand_on);                          //机械手开
                stepperZ.runToNewPosition(crawl_Second_hight[0]);//z轴下降到安全抓取位置
                myservo.write(hand_off);
                stepperZ.runToNewPosition(crawl_Second_hight[1]);//z轴上升到安全运动位置
                stepperX.runToNewPosition(Second_normal[3]);
                stepperY.runToNewPosition(Second_normal[4]);
                stepperZ.runToNewPosition(crawl_Second_hight[0]);//z轴下降到安全抓取位置
                myservo.write(hand_on);
                stepperZ.runToNewPosition(crawl_Second_hight[1]);//z轴上升到安全运动位置
                myservo.write(hand_off);

                

                 Second_crawl_flag[3] = 0x01;                    //二层第一盆已经抓取到差异化第二位置

               

            }
               /*++++++++++++++++回中间过道位置+++++++++++++++++++++*/
                stepperX.runToNewPosition(Middle_aisle[0]);     //x轴运动到对应位置
                stepperY.runToNewPosition(Middle_aisle[1]);     //y轴移动到中间过道
                stepperZ.runToNewPosition(crawl_First_hight[0]);//进入一层，z轴下降到一层安全高度
                /*++++++++++++++++回到初始位置+++++++++++++++++++*/
                stepperX.runToNewPosition(0); 
                stepperY.runToNewPosition(0);
                 

                /*上位机交互，抓取完成*/
                Motor_Status[1] = 0x00;
                Serial1.write(Motor_Status,4);
    }

/**************************************************************************************************** */
/***************************+++++++++++滴灌++++++++++++++++****************************************** */
/*************************************************************************************************** */
    if(water_status && !crawl_status)
    {
        Motor_Status[2] = 0x01;
        Serial1.write(Motor_Status, 4);
        stepperZ.runToNewPosition(crawl_First_hight[1]);                //z轴上升到安全运动高度

        /******************常态化第一层滴灌************************* */
        for(int i = 0; i < 3; i++)                                      //遍历y轴，每次走固定坐标
        {
            int lastY = 0;
            stepperY.runToNewPosition(First_normal[2] + lastY);          //基于上一盆坐标y轴移动到下一盆y值
            lastY += y_last;                                               //预设值，调整更改，y值之间的距离

            nomal_status ++;            
            for(int j = 0;j < 4;j++)                                    //遍历x轴，每次走固定坐标
            {
                int lastX = 0;
                stepperX.runToNewPosition(First_normal[0] + lastX );    //基于上一盆坐标x轴移动到下一盆x值
                if(first_crawl_flag[0] == 0x00 || first_crawl_flag[1] == 0x00)                         //如果常态化第一盆被抓取
                {
                    lastX += x_last;                                      //预设值，调整更改，x值之间的距离
                }
                else
                {
                    Motor_Water(1);
                    delay(drip_timg);                                         //滴灌时间
                    Motor_Water(0);
                    lastX += x_last;
                }
                 if(nomal_status == 4)                                       //判断常态化滴灌是否完成，数值可能是3！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
                {
                    break;                                                   //跳出上层循环
                }
            }

           
        }
            /*************常态化滴灌完成，进行差异化滴灌**************** */
         if(nomal_status == 4)                                       //判断常态化滴灌是否完成
            {
                nomal_status = 0;
                for(int j = 0; j < 2; j++)
                {
                    stepperY.runToNewPosition(First_different[j]);
                    for(int i = 0; i <= 1; i++)                                       //常态化滴灌状态位重置
                    {  
                        stepperX.runToNewPosition(First_different[i]);    //基于上一盆坐标x轴移动到下一盆x值
                        if(first_crawl_flag[2] == 0x01 || first_crawl_flag[3] == 0x01)
                        {
                            Motor_Water(1);
                            delay(drip_timg);                                         //滴灌时间
                            Motor_Water(0);
                        }
                       if((j == 0 && i == 1) || (j == 1 && i == 0))
                       {
                             Motor_Water(1);
                            delay(drip_timg);                                         //滴灌时间
                            Motor_Water(0);
                       }
                    }
                }
               
            }
            /*++++++++++++++++回中间过道位置+++++++++++++++++++++*/
            stepperX.runToNewPosition(Middle_aisle[0]);
            stepperY.runToNewPosition(Middle_aisle[1]);
            stepperZ.runToNewPosition(crawl_Second_hight[1]);               //进入二层

           /******************常态化第二层滴灌************************* */
        for(int i = 0; i < 3; i++)                                      //遍历y轴，每次走固定坐标
        {
            int lastY = 0;
            stepperY.runToNewPosition(Second_normal[2] + lastY);          //基于上一盆坐标y轴移动到下一盆y值
            lastY += y_last;                                               //预设值，调整更改，y值之间的距离

            different_status ++;            
           
            
            for(int j = 0;j < 4;j++)                                    //遍历x轴，每次走固定坐标
            {
                int lastX = 0;
                stepperX.runToNewPosition(Second_normal[0] + lastX );    //基于上一盆坐标x轴移动到下一盆x值
                if(Second_crawl_flag[0] == 0x00 || Second_crawl_flag[1] == 0x00) //如果常态化第一盆被抓取
                {
                    lastX += x_last;                                      //预设值，调整更改，x值之间的距离
                }
                else
                {
                    Motor_Water(1);
                    delay(drip_timg);                                         //滴灌时间
                    Motor_Water(0);
                    lastX += x_last;
                }
                 if(different_status == 4)                                       //判断常态化滴灌是否完成，数值可能是3！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
                {
                    break;                                                   //跳出上层循环
                }
            }

           
        }
            /*************常态化滴灌完成，进行差异化滴灌**************** */
         if(different_status == 4)                                       //判断常态化滴灌是否完成
            {
                nomal_status = 0;
                for(int j = 0; j < 2; j++)
                {
                    stepperY.runToNewPosition(Second_different[j]);
                    for(int i = 0; i <= 1; i++)                                       //常态化滴灌状态位重置
                    {  
                        stepperX.runToNewPosition(Second_different[i]);    //基于上一盆坐标x轴移动到下一盆x值
                        if(first_crawl_flag[2] == 0x01 || first_crawl_flag[3] == 0x01)
                        {
                            Motor_Water(1);
                            delay(drip_timg);                                         //滴灌时间
                            Motor_Water(0);
                        }
                       if((j == 0 && i == 1) || (j == 1 && i == 0))
                       {
                             Motor_Water(1);
                            delay(drip_timg);                                         //滴灌时间
                            Motor_Water(0);
                       }
                    }
                }
               
            } 
    }
}