#pragma once

#include "depth_MoG_background.h"
#include "floor_calib.h"
#include "location_tracker.h"
#include "back_projection.h"

namespace PROCESSOR {

  extern depth_MoG_background* bgmodel;

  extern floor_calib floorCalibration;

  extern Location3DTracker head_tracker;

  // // 3D point clouds
  // extern Eigen::MatrixXf FGPointsInCam;
  // extern Eigen::MatrixXf FGPointsInWorld;
  // extern int validFGPointsNumber; 

  // depth based connected component labeling (CCL)
  // extern Contour_normalize *Contour_normal;
  void init(void);
  void update(unsigned long long time_stamp, int fid);
  void release(void);
}