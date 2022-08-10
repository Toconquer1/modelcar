#pragma once
/**
 * @file ring_recognition.cpp
 * @author zhht
 * @brief 环岛
 * @version 0.0未成功版本
 * @date 2022-08-10 23：36
 *
 * @copyright Copyright (c) 2022
 * @note  环岛识别，进入补线，出去补线，没有解决出环岛再次识别环岛的问题
 *
 */

#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
#include "track_recognition.cpp"

using namespace cv;
using namespace std;

class RingRecognition
{
public:
    enum RingStep
    {
        RingNoneStep = 0,
        RingEntering,
        RingEnterFinish,
        RingExiting
    };
    enum RingPosition
    {
        RingNonePosition=0,
        Left,
        Right
    };
    RingStep nowStep=RingStep::RingNoneStep;
    RingPosition LorRFlag=RingPosition::RingNonePosition;

    bool ringRecognition(TrackRecognition &track)
    {

        //检测是否是环岛，直接在这里检测，不在trackRecognition中进行特殊处理
        //std判断是左边还是右边
        if(nowStep==RingStep::RingNoneStep)
        {
            //if (track.pointsEdgeLeft.size() < 20 || track.pointsEdgeRight.size() < 20)return false;
            
            
            
            bool isRing1 = false;
            bool leftloss=false,rightloss=false;
            int cntleft=0,cntright=0;//丢线计数器
            int edgenum = 10;
            int i = min(track.pointsEdgeLeft.size() - 1, track.pointsEdgeRight.size() - 1);
            for(;i>=0;--i){
                if(track.pointsEdgeLeft[i].y<= edgenum)cntleft++;//左边丢线计数器
                if(track.pointsEdgeRight[i].y>=COLSIMAGE- edgenum)cntright++;//右边丢线计数器
                if(cntleft>50){
                    if(cntright<10){
                        LorRFlag=RingPosition::Left;
                        isRing1=true;
                    }
                    else isRing1=false;
                    break;
                }
                if(cntright>50){
                    if(cntleft<10){
                        LorRFlag=RingPosition::Right;
                        isRing1=true;
                    }
                    else isRing1=false;
                    break;
                }
            }
            if(i==0)return false;
            if (isRing1) {
                nowStep = RingStep::RingEntering;
                return true;
            }
            else {
                return false;
            }




        }
        else if(nowStep==RingStep::RingEntering){
            //补线
            if(LorRFlag==RingPosition::Right){
                int xl=0,yl=0,xr=0,yr=0;
                int indexl=0,indexr=0;
                for(int i=0;i<=track.pointsEdgeLeft.size()-1&&i<=track.pointsEdgeRight.size()-1;++i){
                    if(abs(track.pointsEdgeLeft[i].x-ROWSIMAGE/2)<5){
                        xl=track.pointsEdgeLeft[i].x;
                        yl=track.pointsEdgeLeft[i].y;
                        indexl=i;
                    }
                    if(track.pointsEdgeRight[i].y>=COLSIMAGE-2){
                        xr=track.pointsEdgeRight[i].x;
                        yr=track.pointsEdgeRight[i].y;
                        indexr=i;
                    }
                }
                int temp=track.pointsEdgeLeft[indexr].y+track.pointsEdgeRight[indexr].y;
                track.pointsEdgeLeft[indexr].y=temp/2>COLSIMAGE/2?COLSIMAGE/2:temp/2;//补线终点

                //开始补左线
                float k = (float)(track.pointsEdgeLeft[indexr].y - track.pointsEdgeLeft[indexl].y) / (float)(track.pointsEdgeLeft[indexr].x - track.pointsEdgeLeft[indexl].x);
                float b = track.pointsEdgeLeft[indexl].y - k * track.pointsEdgeLeft[indexl].x;
                for(int i=indexl;i<=indexr;++i){
                    track.pointsEdgeLeft[i].y=(int)(k * track.pointsEdgeLeft[i].x + b);
                }
            }
            //如果找不到分叉点
            if(track.spurroad.size()<1){
                nowStep=RingStep::RingEnterFinish;
            }
            return true;
        }
        else if(nowStep==RingStep::RingEnterFinish){
            //什么都不干正常转弯

            //如果圆环在右，左测有突变
            //如果圆环在左，右测有突变

            if(track.stdevLeft>10&&track.stdevRight>10)//脱离弯道
                nowStep=RingStep::RingExiting;
            return true;
        }
        else if(nowStep=RingStep::RingExiting){
            //应该补线，但是暂时什么都不做
            //环岛在右，找到左边突变点
            if(LorRFlag==RingPosition::Right){
                //找到左边缘的突变点
                int rowBreakLeftDown=searchBreakLeftDown(track.pointsEdgeLeft);
                if(rowBreakLeftDown>=track.pointsEdgeLeft.size()-3){
                    nowStep=RingStep::RingNoneStep;
                    LorRFlag=RingPosition::RingNonePosition;
                    return false;
                }
                int rowRepair=track.pointsEdgeLeft.size()-1;
                for(int i=track.pointsEdgeLeft.size()-1;i>=0;--i){
                    if(track.pointsEdgeLeft[i].y<COLSIMAGE/2){
                        rowRepair=i;
                        break;
                    }
                }
                track.pointsEdgeLeft[rowRepair].y=COLSIMAGE/2;
                //开始补线
                float k = (float)(track.pointsEdgeLeft[rowRepair].y - track.pointsEdgeLeft[rowBreakLeftDown].y) / (float)(track.pointsEdgeLeft[rowRepair].x - track.pointsEdgeLeft[rowBreakLeftDown].x);
                float b = track.pointsEdgeLeft[rowBreakLeftDown].y - k * track.pointsEdgeLeft[rowBreakLeftDown].x;
                for(int i=rowBreakLeftDown;i<=rowRepair;++i){
                    track.pointsEdgeLeft[i].y=(int)(k * track.pointsEdgeLeft[i].x + b);
                }
            }



            
            return true;
           
        }
        else return false;

    }


    /**
     * @brief 绘制十字道路识别结果
     *
     * @param Image 需要叠加显示的图像/RGB
     */
     
    void drawImage(TrackRecognition track, Mat &Image)
    {
        //绘制边缘点
        for (int i = 0; i < track.pointsEdgeLeft.size(); i++)
        {
            circle(Image, Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x), 2,
                   Scalar(0, 255, 0), -1); //绿色点
        }
        for (int i = 0; i < track.pointsEdgeRight.size(); i++)
        {
            circle(Image, Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 2,
                   Scalar(0, 255, 255), -1); //黄色点
        }

        //绘制岔路点
        for (int i = 0; i < track.spurroad.size(); i++)
        {
            circle(Image, Point(track.spurroad[i].y, track.spurroad[i].x), 6,
                   Scalar(0, 0, 255), -1); //红色点
        }
        if(LorRFlag==RingPosition::Right)
        putText(Image, "RingRight", Point(COLSIMAGE / 2 - 5, 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 1, CV_AA); //显示赛道识别类型
        else if(LorRFlag==RingPosition::Left)
        putText(Image, "RingLeft", Point(COLSIMAGE / 2 - 5, 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 1, CV_AA); //显示赛道识别类型)
    }
    
private:
   
    /**
     * @brief 搜索十字赛道突变行（左下）
     *
     * @param pointsEdgeLeft
     * @return uint16_t
     */
    uint16_t searchBreakLeftDown(vector<POINT> pointsEdgeLeft)
    {
        uint16_t rowBreakLeftDown = 0;
        uint16_t counter = 0;

        for (int i = 0; i < pointsEdgeLeft.size() - 10; i++) //寻找左边跳变点
        {
            if (pointsEdgeLeft[i].y > pointsEdgeLeft[rowBreakLeftDown].y)
            {
                rowBreakLeftDown = i;
                counter = 0;
            }
            else if (pointsEdgeLeft[i].y < pointsEdgeLeft[rowBreakLeftDown].y) //突变点计数
            {
                counter++;
                if (counter > 5)
                    return rowBreakLeftDown;
            }
        }

        return rowBreakLeftDown;
    }
};
