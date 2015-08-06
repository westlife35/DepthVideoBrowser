#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "depth_MoG_background.h"
// #include "alg/depth_background.h"
#include <math.h>
//#include "util/configfile.h"
#include "util/color_space.h"
//#include "Device.h"

#include "openni/include/openni/OpenNI.h"
//I comment the util/configfile.h and Device.h , add the openni's include file

#include "LogicLevel/prime_sensor_frame_get.h"

using namespace std;


//-----------------------------------------------------------------------------
// Mixture of Gaussian algorithm
// Improved by this paper: An Improved Adaptive Background Mixture Model for Real-time Tracking with Shadow Detection
//-----------------------------------------------------------------------------


//------------------------------------------------------------
depth_MoG_background::depth_MoG_background(int rows, int cols) :
    is_empty(true) {
  resize(rows, cols);
  is_empty = true;
  learning_frame_cnt = -1;
  fg_detection_th = 30.0f;

  ///////////// ransac //////////////
  mapVec = new int[640 * 480];
  ransacDataBuf = new Point3D[640 * 480];
  rgbBuf = new unsigned char[640 * 480 * 3];
  //////////////////////
}

depth_MoG_background::~depth_MoG_background() {
  is_empty = true;
  clear();
}

//------------------------------------------------------------
void depth_MoG_background::clear() {
  is_empty = true;
  if (idx.N <= 0)
    return;

  MoG_buf.clear();
  delete[] fgmask;
  delete[] avg_rgb;
  delete[] dist_mean;
  delete[] dist_sigma;


  ///////////// ransac //////////////
  delete []mapVec;
  delete []ransacDataBuf;
  delete []rgbBuf;
  ///////////////////////////////////
}

//------------------------------------------------------------
void depth_MoG_background::resize(int rows, int cols) {
  if (idx.cols() != cols || idx.rows() != rows) {
    printf("depth_MoG_background::resize() to %dx%d\n", cols, rows);

    clear();
    idx.setSize(rows, cols);

    MoG_buf.resize(idx.N);
    fgmask = new bool[idx.N];
    avg_rgb = new unsigned char[3 * idx.N];
    dist_mean = new float[idx.N];
    dist_sigma = new float[idx.N];

    // reset the values
    memset(avg_rgb, 0, sizeof(unsigned char) * idx.N * 3);
    for(int i=0; i<idx.N; i++)
    {
      MoG_buf[i].reset();
    }
  }
}

inline float clamp(float value, float min, float max) {
  if (value < min)
    return min;

  if (value > max)
    return max;

  return value;
}

void depth_MoG_background::start_learning() {
  // reset the values
  memset(avg_rgb, 0, sizeof(unsigned char) * idx.N * 3);
  for(int i=0; i<idx.N; i++)
  {
    MoG_buf[i].reset();
  }

  is_empty = true;
  learning_frame_cnt = 1;
  printf(
      "Start learning MoG background modelling for %d frames. Please clear the scene ... \n",
      backgroundModelLearningFrameNumber);
}

void depth_MoG_background::finish_learning() {
  learning_frame_cnt = -1;
}

//-------------------------------------------------------------
// this is the method called at every frame.
// when a background model is available, compute foreground using it
// otherwise, learn a background model
void depth_MoG_background::update(const unsigned short* depthImg, int depth_w,
    int depth_h, const unsigned char * rgbImg, int rgb_w, int rgb_h) {

  static int learning_frame_interval_cnt = 0;

  if (is_empty) {
    learning_frame_interval_cnt++;
    if(learning_frame_interval_cnt%5 == 0)
      learn(depthImg, depth_w, depth_h, rgbImg, rgb_w, rgb_h);
  } else {
    computeForeground(depthImg, depth_w, depth_h);
  }
}

void depth_MoG_background::learn(const unsigned short* depthImg, int depth_w,
    int depth_h, const unsigned char * rgbImg, int rgb_w, int rgb_h) {
  if (learning_frame_cnt < 0)
    return;

  // print the learning progress
  printf(".");
  if (learning_frame_cnt % 30 == 0 && learning_frame_cnt > 1) {
    printf("%3.3f %% \n",
        100 * (float) learning_frame_cnt
            / (float) backgroundModelLearningFrameNumber);
    setvbuf(stdout, 0, _IONBF, 0);
  }

  // make sure the size of input depth image matches
  if (depth_w != idx.cols() || depth_h != idx.rows()) {
    printf(
        "Error: depth_MoG_background::update, input depth image wrong size!\n");
    exit(-1);
  }

  const unsigned short *_dist = depthImg;
  const unsigned char *_color = rgbImg;

  for (int i = 0; i < idx.N; i++) {
    // ignore the invalid depth values;
    if (_dist[i] == 65535 || _dist[i] == 0)
      continue;

    MoG_buf[i].update(_dist[i], _color[3 * i + 0], _color[3 * i + 1],
        _color[3 * i + 2]);
  }

/*
  /////////////////////////////////////////////////////////////////////////////
  // for debug
  int pixel_id = idx.rc2i(300, 300);
  // ignore the invalid depth values;
  if (_dist[pixel_id] != 65535 && _dist[pixel_id] != 0)
    MoG_buf[pixel_id].update(_dist[pixel_id], _color[3 * pixel_id + 0], _color[3 * pixel_id + 1], _color[3 * pixel_id + 2]);

  printf("Input value: %d\n", _dist[pixel_id]);
  MoG_buf[pixel_id].print();
  /////////////////////////////////////////////////////////////////////////////
*/

  // conclude the final background;
  conclude();

  learning_frame_cnt++;
  learning_frame_cnt =
      (learning_frame_cnt > backgroundModelLearningFrameNumber) ?
          backgroundModelLearningFrameNumber : learning_frame_cnt;

  if (learning_frame_cnt >= backgroundModelLearningFrameNumber) {
    is_empty = false;
    learning_frame_cnt = -1;

    // conclude the final background;
    conclude();

    printf(
        "Learning is finished. A background modelling is now available\n");
  }
}

void depth_MoG_background::conclude(void)
{
  // conclude the background model
  for(int i=0; i<idx.N; i++)
  {
    float mean, sigma;
    unsigned char r, g, b;
    MoG_buf[i].conclude(&mean, &sigma, &r, &g, &b);

    dist_mean[i] = mean;
    dist_sigma[i] = sigma;
    avg_rgb[3*i+0] = r;
    avg_rgb[3*i+1] = g;
    avg_rgb[3*i+2] = b;
  }

  return;
}

//------------------------------------------------------------
void depth_MoG_background::construct() {
}

bool depth_MoG_background::save(char* filename) const {

  FILE* bg_file = fopen(filename, "wb");

  if (bg_file != NULL && is_empty == false) {
    // first write the size of background model
    unsigned int sizeinfo[2];
    sizeinfo[0] = idx.rows();
    sizeinfo[1] = idx.cols();
    fwrite(sizeinfo, sizeof(unsigned int), 2, bg_file);

    // write depth background model data
    fwrite(dist_mean, sizeof(float), idx.N, bg_file);
    fwrite(dist_sigma, sizeof(float), idx.N, bg_file);

    // write the RGB data
    fwrite(avg_rgb, sizeof(unsigned char), idx.N * 3, bg_file);

    fflush(bg_file);

    fclose(bg_file);
    printf("Depth-based background model is saved in %s.\n", filename);
    return true;
  } else {
    printf("Saving background file %s failed. \n", filename);
    return false;
  }

  return true;
}

bool depth_MoG_background::load(char* filename) {

  FILE* bg_file = fopen(filename, "rb");

  if (bg_file != NULL) {
    // first load the size of background model
    unsigned int sizeinfo[2];
    fread(sizeinfo, sizeof(unsigned int), 2, bg_file);
    if (idx.rows() != sizeinfo[0] || idx.cols() != sizeinfo[1]) {
      printf("Load background model failed. Size doesn't match.\n");
      return false;
    }

    // load other data
    fread(dist_mean, sizeof(float), idx.N, bg_file);
    fread(dist_sigma, sizeof(float), idx.N, bg_file);
    fread(avg_rgb, sizeof(unsigned char), idx.N * 3, bg_file);

    //////////////////////////ransac////////////////////////////
    //update data for ransac alg
    for(int i = 0; i < idx.N; ++i)
    {
      if(dist_mean[i] != 0 && dist_mean[i] != 65535)
      {
        Point3D p;
        int x = i % 640;
        int y = i / 640;
      }
    }

    memcpy(rgbBuf, avg_rgb, 640*480*3);
    //////////////////////////////////////////////////////

    fflush(bg_file);

    fclose(bg_file);

    is_empty = false;
    return true;
  } else {
    printf("Loading background file %s failed. \n", filename);
    return false;
  }

  return true;
}

////////////////////////////////////////////////////////////////////
// this method allows synthesizing a backgroud model from imported depth map (of the synthesized background)
////////////////////////////////////////////////////////////////////
bool depth_MoG_background::importFromDepthMap(unsigned short* depth, int w, int h)
{
  if (idx.rows() != h || idx.cols() != w) {
    printf("Background model importing failed. Size doesn't match.\n");
    return false;
  }

  // copy depth buffer to mean buffer and automatically set sigma
  int buffer_size = w*h;
  int validPointNum = 0;
  for(int i=0; i<buffer_size; i++)
  {
    dist_mean[i] = (float)depth[i];
    dist_sigma[i] = 30; // TODO: set sigma based on depth value
    UInt16RGB2Jet(depth[i]%20000, 0, 20000, avg_rgb+3*i);

    if(dist_mean[i] > 0)
      validPointNum++;
  }

  is_empty = false;

  return true;
}


void depth_MoG_background::computeForeground(const unsigned short* depthImg,
    int depth_w, int depth_h) {

  // make sure a background model is already available
  if (is_empty) {
    printf(
        "Warning: depth_MoG_background::computeForeground, a background model is not available!\n");
    return;
  }

  // make sure the size of input depth image matches
  if (depth_w != idx.cols() || depth_h != idx.rows()) {
    printf(
        "Error: depth_MoG_background::getForeground, input depth image wrong size!\n");
    exit(-1);
  }

  // compute foreground mask
  const unsigned short* _depth = depthImg;
  int validFGNum = 0;
  for (int i = 0; i < idx.N; i++) {
    if (_depth[i] == 0 || _depth[i] == 65535) {
      fgmask[i] = false;
      continue;
    }

    if (dist_mean[i] - _depth[i] < fg_detection_th * dist_sigma[i])
      fgmask[i] = false;
    else
      fgmask[i] = true;

  }
}

bool* depth_MoG_background::getForeground() {
  return fgmask;
}

bool depth_MoG_background::modelAvailable(void) {
  return (!is_empty);
}

unsigned char* depth_MoG_background::getRGBData() {
  return avg_rgb;
}

void depth_MoG_background::increaseSensitivity(void) {
  fg_detection_th *= 0.95f;
  printf("Foreground detection threshold: %3.3f.\n", fg_detection_th);
}

void depth_MoG_background::decreaseSensitivity(void) {
  fg_detection_th *= 1.05f;
  printf("Foreground detection threshold: %3.3f.\n", fg_detection_th);
}

float* depth_MoG_background::getDepthMean(void) {
  return dist_mean;
}

bool depth_MoG_background::get3DPointFromPatch(int x, int y, int patchSize, Point3D& output) {
  unsigned int depthValue;
  Point3D projectivePoint;
  Point3D realWorldPoint;
  unsigned long allDepthValue = 0;
  unsigned int validPointNum = 0;
  int sizeSquare = patchSize * patchSize;
  for (int xx = x - patchSize; xx <= x + patchSize; xx++) {
    if (xx < 0 || xx > idx.width())
      continue;

    for (int yy = y - patchSize; yy <= y + patchSize; yy++) {
      if (yy < 0 || yy > 480)
        continue;

      int distanceSquare = (xx - x) * (xx - x) + (yy - y) * (yy - y);
      if (distanceSquare <= sizeSquare) {
        depthValue = dist_mean[yy * idx.width() + xx];
        if (depthValue > 400) {
          allDepthValue += depthValue;
          validPointNum++;
        }
      }
    }
  }

  if (validPointNum > 0) {
    projectivePoint.x = (float) x;
    projectivePoint.y = (float) y;
    projectivePoint.z = (float) allDepthValue / validPointNum;

    float x, y, z;
    //openni::CoordinateConverter::convertDepthToWorld(getDepthStream(), projectivePoint.x, projectivePoint.y, projectivePoint.z,&x, &y, &z);
    openni::CoordinateConverter::convertDepthToWorld(dg_controls::sensor_->GetDepthStream(), projectivePoint.x, projectivePoint.y, projectivePoint.z,&x, &y, &z);
    realWorldPoint.x = x;
    realWorldPoint.y = y;
    realWorldPoint.z = z;

    output = realWorldPoint;
  } else {
    output.x = 0.0f;
    output.y = 0.0f;
    output.z = 0.0f;

    return false;
  }

  return true;
}
