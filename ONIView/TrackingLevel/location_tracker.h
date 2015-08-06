#pragma once
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "util/tools.h"
#include "util/common_things.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define INTER_OBJECT_DISTANCE_MAX                   1000000.0f
#define OBJECT_CONTINUITY_DISPLACEMENT_MAX          1000.0f
#define OBJECT_TRACKING_EXPIRE_TIMER_MIN            5
#define OBJECT_TRACKING_EXPIRE_TIMER_MAX            30
#define OBJECT_TRACKING_SMOOTHING_FILTERSIZE        10
#define OBJECT_TRACKING_SMOOTHING_GAUSSIAN_SIGMA    3.0f
#define TRACKING_MATRURITY_TH                       10
#define OBJECT_POSITION_BUFFER_SIZE                 500
#define VELOCITY_TIME_DELTA                         3   // the time duration on which the velocity is computed.
#define VALID_TRACKING_TRAJECTORY_MIN       10

using namespace std;
using namespace cv;

typedef struct
{
  int ID;
  vector<Eigen::Vector3f> trajectory;
} objectTrajectory;

typedef struct
{
  int id;
  Eigen::Vector3f p3d;
} objectPosition;

typedef struct
{
  int frame_id;
  int object_num;
  vector<objectPosition> positions;
} framepositions;

//-------------------------------------------------------
class objectLocation3D {
public:
  objectLocation3D();
  objectLocation3D(int newID, Eigen::Vector3f position);
  objectLocation3D(Eigen::Vector3f position);
  ~objectLocation3D();

  Eigen::Vector3f getPosition();
  void updateTracking(Eigen::Vector3f newPosition);
  bool updateLostTracking();
  //add for change tracking position by hand
  void resetOneFrameTracking(Eigen::Vector3f newPosition);

  float distance(objectLocation3D other);
  float expected_distance(objectLocation3D other);
  int getID();
  void setID(int nNewID);
  void info();
  int getAge();
  bool isCurrentlyDetected();
  bool isMature();
  void getTrajectoryPoints(vector<Eigen::Vector3f>& output);

private:
  Eigen::Vector3f velocity;
  Eigen::Vector3f tracked_position;
  Eigen::Vector3f expected_position;
  vector<Eigen::Vector3f> detection_position_buffer;
  vector<Eigen::Vector3f> tracking_position_buffer;
  float smoothing_filter_parameters[OBJECT_TRACKING_SMOOTHING_FILTERSIZE];

  int ID;
  int expirationTimeOut;  // the count-down timeout of expiration, when this equals to zeros, this object will be expired and removed.
  int lifeTime; // the overall age of this people
  bool currentlyDetectionState;

  void compute_Gaussian_parameters();
  void apply_Gaussian_filter();
  void update_velocity();
};


//-------------------------------------------------------
class Location3DTracker {
public:
  Location3DTracker();
  //added by myself to solve the problem of leaved info of last *.oni when changed to a new *.oni
  void reInit(int nNumOfTotalFrame);
  ~Location3DTracker();
  int getPeopleNum();

  bool getPositions(int frameId, framepositions& positions);

  //set the current tracking state
  void setTrackingState(vector<objectLocation3D> all_peopleIn);

  //add for adjust the tracking by hand
  //bool setPosition(int frameId, framepositions& positions);
  void setPosition(int i,int x,int y);
  void addPosition(int x,int y);
  void setID(int nID,int nIndex);
  void deletePeopleByIndex(int nIndex);

  void restoreAllPeople(framepositions& positions);
  bool getPeople(int i, objectLocation3D& output);
  //add for adjust track pos by hand
  void getAllPeople(vector<objectLocation3D> &all_people);
  bool getPeopleWithID(int id, objectLocation3D& output);
  bool deletePeopleWithID(int id);
  bool newPeopleWithID(int id);
  void update(vector<objectLocation3D> detected_people, unsigned long long time_stamp, int fid);
  void startRecording(char* per_frame_filename, char* per_people_filename);
  void stopRecording(void);

  void saveLabeling(char *save_file);
  void updateLabeling(int fid);

  int getnewID();
  void setnewID(int newIDtoSet);

  vector<objectTrajectory> curCompletedTrackingResults; // this vector stores the tracking results completed at current frame;

  bool no_update;
  pair<int, int> p2d;
  Eigen::Vector3f p3d;

private:
  vector<objectLocation3D> all_people;
  int newID;

  // assets for tracking result recorder
  bool isRecorderOn;
  FILE* recorderFilePerPeople;
  FILE* recorderFilePerFrame;

  FileStorage fs;
  vector<framepositions> all_trajectory;

  unsigned long long cur_time;

  void record_tracking_result_per_people(int id);
  void record_tracking_result_per_frame();
};
