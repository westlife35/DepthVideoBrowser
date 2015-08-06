#pragma once

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;
using namespace std;

namespace PLANVIEW
{
  struct pixel_loc_value
  {
    int row;
    int col;
    float value;
  };

  struct detected_object
  {
    Eigen::Vector3f centroidInWorld;
    Eigen::Vector3f headTopInWorld;
  };

  extern vector<struct detected_object> detectedObjects;

  extern void init(void);
  extern void update(int pointNumber, const Eigen::MatrixXf& pointsInCam, const Eigen::MatrixXf& pointsInWorld);
  extern void release(void);
  extern void switchDebugVisualization(void);
  extern void visualizationUpdate(void);


}