#ifndef RGBDFRAME_H
#define RGBDFRAME_H

class RGBDFrame {
public:
 RGBDFrame();
 ~RGBDFrame();
 void setIndexAndSize(int nFrameIndex,int nWidth,int nHeight);
 void initMem();
 void setData(unsigned short* depthData,unsigned char* rgbData,unsigned char* rgbFromDepthData_);
public:
  unsigned short* depthData_;
  unsigned char* rgbData_;
  unsigned char* rgbFromDepthData_;
  bool bHas_depth_;
  bool bHas_rgb_;
  int nWidth_;
  int nHeight_;
  int nFrameIndex_;
  bool bIsInitMem;
//  unsigned long long time_stamp_;
//  unsigned long long time_;
//  unsigned long long unique_ID;
};

#endif // RGBDFRAME_H
