#pragma once

//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/highgui/highgui.hpp"
#include "opencv/cv.h"
#include "opencv/highgui.h"

using namespace cv;
using namespace std;

//all include files in cpp are put here
#include <stdio.h>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>
//I comment one row below
//#include "util/configfile.h"
#include "Tracker.h"
#include "planview.h"
#include "colormap.hpp"
//I comment one row below
//#include "Device.h"

//I add one include file
#include "LogicLevel/prime_sensor_frame_get.h"

//add for the type DepthSensor
#include "CamLevel/depth_sensor.h"
using namespace dg_devices;

namespace BACK_PROJECTION {

extern Size backprojection_resolution;
extern Mat RGB_Vis, Depth_Vis;

extern void init(void);
extern void update(void);
extern void release(void);

void Get2Dfrom3D(DepthSensor *sensor_,Point3D p3d, Point2Di& p2d, unsigned short& depth_value);

}
