/*************************************************************************
	> File Name: prime_sensor_frame_get.h
	> Author: 
	> Mail: 
	> Created Time: Mon 01 Jun 2015 11:32:39 AM CST
 ************************************************************************/

#ifndef _PRIME_SENSOR_FRAME_GET_H
#define _PRIME_SENSOR_FRAME_GET_H

//#include <fcntl.h>
//#include <iostream>
//#include "glog/logging.h"
#include "CamLevel/sensor.h"
#include "CamLevel/depth_sensor.h"
#include "CamLevel/primesense_sensor.h"
#include <cstring>
#include "color_space.h"
//#include "TrackingLevel/util/color_space.h"

//include to make the Point3D and Point2Df can be used
//#include "TrackingLevel/util/common_things.h"

//RGBDFrame header file
#include "LogicLevel/rgbdframe.h"

#include <vector>

//add tracker
//#include "TrackingLevel/Tracker.h"


namespace dg_controls{
  using namespace std;
  using namespace dg_devices;

//  struct FrameStorage{
//    unsigned char* colorFromDepthdata;
//    //unsigned char* colorData;
//    void* colorData;
//  };

  //make the file depth_MoG_background.cpp can use the sensor_ to get the depthstream
  extern DepthSensor *sensor_;
  extern vector<RGBDFrame> rgbdFrame;
  extern float g_pointCam3D[640*480*3];

  struct FrameNoAndall_people
  {
    int nFrameNo;
    //vector<objectLocation3D> currentAll_people;
    int newID;

  };

  class ONICapture
  {

public:
  ONICapture();
  ~ONICapture();
  bool Open(std::string name, std::string path);
  bool Open();
  void Update();
  int GetDepthHeight();
  int GetDepthWidth();
  int GetColorHeight();
  int GetColorWidth();
  const char* GetSensorName();
  const char* GetSensorURI();
  bool GetSensorProperties(map<string, float> &prop);
  bool StartRecording();
  bool StopRecording();
  bool ConvertDepthToWorld(float depthX, float depthY, float depthZ,
			float* pWorldX, float* pWorldY, float* pWorldZ);
  void ** GetData();
  bool Close();
  void SetMode(int nIndex);
  void GetColorFromDepth(unsigned char* rgb);
  void* GetColorData();
  void* GetColorFromDepthData();
  int GetFrameIndex();

public:
  //DepthSensor *sensor_;
  //FrameStorage frameStorage_;
  bool bIsOpened;
  bool isFileOrCam;
  unsigned char *colorFromDepthdata;
  //PlaybackControl *playbackControl_;
  string sensorName_;
  //RGBDFrame
  //RGBDFrame rgbdFrame;

  //fix the bug of the number of tracked ID will bigger than before the binary is auto replay
  int nLastFrameNo;
  bool bIsHaveUpdateOnce;
  bool bIsBackPlayOrForwardPlay;

  //20150604 add
public:
  void GetPlaybackControl();
  int GetNumberOfDepthFrames ();
  int GetNumberOfColorFrames();
  bool GetRepeatEnabled();
  float GetSpeed();
  bool IsValid();
  Status SeekColorStream (int frameIndex);
  Status SeekDepthStream (int frameIndex);
  Status SetRepeatEnabled (bool repeat);
  Status SetSpeed (float speed);

//  void DrawObjectTracker();
//  static void ConvertDepth2Cam3D();//declare the function in static to make can be used from ::
//  void Get2Dfrom3D(Point3D p3d, Point2Di& p2d, unsigned short& depth_value);
//  static void Get3Dfrom2D(int x, int y, unsigned short depth_value, Point3D& output);
//  void DrawTracker(int x, int y, int id);
  //void DrawMessage(void* font, int x, int y, const char* message, float fRed, float fGreen, float fBlue);

  //fix the bug of error happens when choose a new oni after have played a oni
  void SetNewOniFlag();
//  void AddTrackPosByHand(int x,int y);
//  void SetTrackIDByHand(int nID,int nIndex);
  //int FindNearstTrackGoal(framepositions originPositions,int x,int y);
//  int IsFindNearstTrackGoal(int x,int y);
//  int FindNearstTrackGoal(vector<objectLocation3D> all_people,int x,int y);
//  int ComputeEuclidean(Point2Di pt2d_img,int x,int y);
//  void SetTrackingStateToCurrentFrame();
//  bool GetTrackingState(int nCurrentFrameNo,vector<objectLocation3D> &all_peopleIn);
//  void DeletePeopleByIndex(int nIndex);

  vector<FrameNoAndall_people>  vecFrameNoAndall_people;

  };

}

#endif
