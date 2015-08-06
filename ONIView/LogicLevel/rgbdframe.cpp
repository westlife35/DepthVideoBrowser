#include "LogicLevel/rgbdframe.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include<istream>
using namespace std;

using namespace cv;


RGBDFrame::RGBDFrame()
{
  depthData_=0;
  rgbData_=0;
  bHas_depth_=false;
  bHas_rgb_=false;
  nWidth_=0;
  nHeight_=0;
  nFrameIndex_=-1;
  bIsInitMem=false;
}

void RGBDFrame::setIndexAndSize(int nFrameIndex,int nWidth,int nHeight)
{
  nFrameIndex_=nFrameIndex;
  nWidth_=nWidth;
  nHeight_=nHeight;
}

void RGBDFrame::initMem()
{
  if( nWidth_>0 && nHeight_>0 && nFrameIndex_>=0 )
  {
     bIsInitMem=true;
     depthData_=new unsigned short[nWidth_*nHeight_];
     rgbData_=new unsigned char[nWidth_*nHeight_*3];
     rgbFromDepthData_=new unsigned char[nWidth_*nHeight_*3];
  }
}

void RGBDFrame::setData(unsigned short* depthData,unsigned char* rgbData,unsigned char* rgbFromDepthData)
{
  if(nWidth_==0 || nHeight_==0)
  {
    //cout<<"the error: nWidth_==0 || nHeight_==0 "<<endl;
    return;
  }

  unsigned short* tempDepthData_=depthData_;
  unsigned char* tempRgbData_=rgbData_;
  unsigned char* tempRgbFromDepthData_=rgbFromDepthData_;

  //debug
  //cout<<&tempRgbData_<<" sdsdsds  "<<&rgbData_<<endl;

//  *depthData_=*depthData;
//  *rgbData_=*rgbData;
//  *rgbFromDepthData_=*rgbFromDepthData;

  for(int i=0;i<nWidth_*nHeight_;i++)
  {
    *tempDepthData_ = (*depthData);
    tempDepthData_++;
    depthData++;

    *tempRgbData_ = *rgbData;
    *(tempRgbData_+1) = *(rgbData+1);
    *(tempRgbData_+2) = *(rgbData+2);
    tempRgbData_+=3;
    rgbData+=3;

    *tempRgbFromDepthData_ = *rgbFromDepthData;
    *(tempRgbFromDepthData_+1) = *(rgbFromDepthData+1);
    *(tempRgbFromDepthData_+2) = *(rgbFromDepthData+2);
    tempRgbFromDepthData_+=3;
    rgbFromDepthData+=3;
  }

//  //test
//  Mat mTempColorForPaintID=Mat(480,640,CV_8UC3);
//  mTempColorForPaintID.data=rgbFromDepthData_;
//  imshow("x",mTempColorForPaintID);
//  waitKey(0);
}

RGBDFrame::~RGBDFrame()
{
  if(bIsInitMem)
  {
      delete depthData_;
      delete rgbData_;
      delete rgbFromDepthData_;
  }

//  if(depthData_!=0)
//  {
//    delete depthData_;
//    depthData_=0;
//  }
//  if(rgbData_!=0)
//  {
//    delete rgbData_;
//    rgbData_=0;
//  }
//  if(rgbFromDepthData_!=0)
//  {
//    delete rgbFromDepthData_;
//    rgbFromDepthData_=0;
//  }
}
