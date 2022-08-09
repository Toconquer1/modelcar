#pragma once
/**
 * @file cross_recognition.cpp
 * @author Leo (liaotengjun@hotmail.com)
 * @brief 十字道路识别与图像处理
 * @version 0.1
 * @date 2022-03-14
 *
 * @copyright Copyright (c) 2022
 *
 * @note 十字道路处理步骤：
 *                      [01] 入十字类型识别：track_recognition.cpp
 *                      [02] 补线起止点搜索
 *                      [03] 边缘重计算
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
            if (track.pointsEdgeLeft.size() < 20 || track.pointsEdgeRight.size() < 20)return false;
            
            
            int i=min(track.pointsEdgeLeft.size()-1,track.pointsEdgeRight.size()-1);
            bool isRing1 = false;
            bool leftloss=false,rightloss=false;
            int cntleft=0,cntright=0;//丢线计数器
            int edgenum = 5;
            for(;i>=0;--i){
                if(track.pointsEdgeLeft[i].y<= edgenum)cntleft++;//左边丢线计数器
                if(track.pointsEdgeRight[i].y>=COLSIMAGE- edgenum)cntright++;//右边丢线计数器
                if(cntleft>100){
                    if(cntright<10){
                        LorRFlag=RingPosition::Left;
                        isRing1=true;
                    }
                    else isRing1=false;
                    break;
                }
                if(cntright>100){
                    if(cntleft<10){
                        LorRFlag=RingPosition::Right;
                        isRing1=true;
                    }
                    else isRing1=false;
                    break;
                }
            }
            if (isRing1) {
                nowStep = RingStep::RingEntering;
                return true;
            }
            else {
                return false;
            }






            /*
            //如果是环岛，低视角
            //判断逻辑为某一侧先丢线，某一侧后丢线
            bool isRing1=false,isRing2=false;
            int i=0;
            int cntleft=0,cntright=0;
            int edgenum = 5;
            for(i=0;i<=track.pointsEdgeLeft.size()-1&&i<=track.pointsEdgeRight.size()-1;++i){
                if(track.pointsEdgeLeft[i].y<= edgenum &&track.pointsEdgeRight[i].y>=COLSIMAGE- edgenum)continue;//跳过一开始的两侧都丢线
            }
            for(;i<=track.pointsEdgeLeft.size()-1&&i<=track.pointsEdgeRight.size()-1;++i){
                if(track.pointsEdgeLeft[i].y>= edgenum)cntleft++;//左边有线计数器
                if(track.pointsEdgeRight[i].y<=COLSIMAGE- edgenum)cntright++;
                i
            }
            if(!isRing1)return false;

            cntleft=0,cntright=0;//清空计数器
            if(LorRFlag==RingPosition::Right){
                for(;i<=track.pointsEdgeLeft.size()-1&&i<=track.pointsEdgeRight.size()-1;++i){
                    if(cntleft<10){
                        if(track.pointsEdgeLeft[i].y>=2)cntleft++;
                        else cntleft=0;
                    }
                    else break;
                }
                cntleft=0,cntright=0;
                for(;i<=track.pointsEdgeLeft.size()-1&&i<=track.pointsEdgeRight.size()-1;++i){
                    if(track.pointsEdgeLeft[i].y>=2)cntleft++;//左边有线计数器
                    if(track.pointsEdgeRight[i].y<=COLSIMAGE-2)cntright++;
                    if(cntleft>40){
                        if(cntright<15){
                            isRing2=true;
                        }
                        break;
                    }
                }

                if(!isRing2){
                    LorRFlag=RingPosition::RingNonePosition;
                    return false;
                }

                //找分叉点
                //左边的分叉点定为row中间的点,右边的点是最后一次丢线的点
                int xl=0,yl=0,xr=0,yr=0;
                int indexl=0,indexr=0;
                for(i=0;i<=track.pointsEdgeLeft.size()-1&&i<=track.pointsEdgeRight.size()-1;++i){
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
                track.pointsEdgeLeft[indexr].y=temp/2>COLSIMAGE/2?COLSIMAGE/2:temp/2;

                //开始补左线
                float k = (float)(track.pointsEdgeLeft[indexr].y - track.pointsEdgeLeft[indexl].y) / (float)(track.pointsEdgeLeft[indexr].x - track.pointsEdgeLeft[indexl].x);
                float b = track.pointsEdgeLeft[indexl].y - k * track.pointsEdgeLeft[indexl].x;
                for(i=indexl;i<=indexr;++i){
                    track.pointsEdgeLeft[i].y=(int)(k * track.pointsEdgeLeft[i].x + b);
                }
            }

            
        
            if(isRing1&&isRing2){
                nowStep=RingStep::RingEntering;
                return true;
            }
            else{
                return false;
            }
           */

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
                track.pointsEdgeLeft[indexr].y=temp/2>COLSIMAGE/2?COLSIMAGE/2:temp/2;

                //开始补左线
                float k = (float)(track.pointsEdgeLeft[indexr].y - track.pointsEdgeLeft[indexl].y) / (float)(track.pointsEdgeLeft[indexr].x - track.pointsEdgeLeft[indexl].x);
                float b = track.pointsEdgeLeft[indexl].y - k * track.pointsEdgeLeft[indexl].x;
                for(int i=indexl;i<=indexr;++i){
                    track.pointsEdgeLeft[i].y=(int)(k * track.pointsEdgeLeft[i].x + b);
                }
            }
            //如果找不到分叉点
            if(track.spurroad.size()<2){
                nowStep=RingStep::RingEnterFinish;
            }
            return true;
        }
        else if(nowStep==RingStep::RingEnterFinish){
            //什么都不干正常转弯

            //如果圆环在右，左测有突变
            //如果圆环在左，右测有突变
            nowStep=RingStep::RingExiting;
            return true;
        }
        else if(nowStep=RingStep::RingExiting){
            //应该补线，但是暂时什么都不做

            //如果圆环在右，左测没有突变
            //如果圆环在左，右测没有突变
            if(track.stdevLeft<60&&track.stdevRight<60){
                 nowStep=RingStep::RingNoneStep;
                LorRFlag=RingPosition::RingNonePosition;
                return false;
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
   
    
    /*
    bool repairLeftEnable = false;  //左边缘补线使能标志
    bool repairRightEnable = false; //右边缘补线使能标志

    uint16_t _index = 0;
    POINT _pointLU;
    POINT _pointLD;
    POINT _pointRU;
    POINT _pointRD;
    string _text;
    */
    /**
     * @brief 十字道路类型
     *
     */
     /*
    enum CrossroadType
    {
        None = 0,
        CrossroadLeft,     //左斜入十字
        CrossroadRight,    //右斜入十字
        CrossroadStraight, //直入十字
    };

    CrossroadType crossroadType = CrossroadType::None; //十字道路类型
    */
};