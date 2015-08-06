#pragma once

#include <stdio.h>
#include <algorithm>
#include <vector>
#include "util/tools.h"
#include "util/common_things.h"

#define MOG_COMPONENT_NUM_MAX 5
#define MOG_WEIGHT_INIT     0.05f
#define SIGMA_INIT        500.0f
#define MATCHING_SIGMA_FACTOR 10.0f

#define MOG_LEARNING_INITIAL_LEARNING_FRAME_NUMBER  300
#define backgroundModelLearningFrameNumber  30

using namespace std;

//-----------------------------------------------------------------------------
// Mixture of Gaussian algorithm
// Improved by this paper: An Improved Adaptive Background Mixture Model for Real-time Tracking with Shadow Detection
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// depth_Gaussian_Kernel: A component of depth_MoG
//-----------------------------------------------------------------------------
class depth_Gaussian_Kernel {

public:
  depth_Gaussian_Kernel(float mu_, unsigned char r, unsigned char g,
      unsigned char b) {
    weight = MOG_WEIGHT_INIT;
    mu = mu_;
    covar = SIGMA_INIT;
    fitness = weight / sqrt(covar);
    cur_N = 1.0f;
    matching_times = 1.0f;

    rgb[0] = r;
    rgb[1] = g;
    rgb[2] = b;
  }
  ;

  ~depth_Gaussian_Kernel() {
    return;
  }
  ;

  bool isMatch(float value) {
    float distance_sq = (mu - value) * (mu - value);

    if (distance_sq < (MATCHING_SIGMA_FACTOR * covar))
      return true;
    else
      return false;
  }
  ;

  void positive_update(float value, unsigned char r, unsigned char g,
      unsigned char b) {
    // update learning time and matched frequency
    cur_N += 1.0f;
    matching_times += 1.0f;

    // update kernel
    weight += (1.0f - weight) / (cur_N + 1.0f);
    mu += (value - mu) / matching_times;
    covar += ((value - mu) * (value - mu) - covar) / matching_times;
    fitness = weight / sqrt(covar);

    if (cur_N == 1.0f) {
      rgb[0] = r;
      rgb[1] = g;
      rgb[2] = b;
    } else {
      float new_color_weight = 1.0f/matching_times;
      // running average
      rgb[0] = (unsigned char)((1.0f-new_color_weight)*rgb[0]  + new_color_weight*r);
      rgb[1] = (unsigned char)((1.0f-new_color_weight)*rgb[1]  + new_color_weight*g);
      rgb[2] = (unsigned char)((1.0f-new_color_weight)*rgb[2]  + new_color_weight*b);
    }

  }
  ;

  void negtive_update(float value) {
    // update learning time and matched frequency
    cur_N += 1.0f;

    // update kernel
    weight += (0.0f - weight) / (cur_N + 1.0f);
  }
  ;

  void exchange(depth_Gaussian_Kernel& other) {
    float temp;

    temp = weight; weight = other.weight; other.weight = temp;
    temp = mu; mu = other.mu; other.mu = temp;
    temp = covar; covar = other.covar; other.covar = temp;
    temp = fitness; fitness = other.fitness; other.fitness = temp;
    temp = cur_N; cur_N = other.cur_N; other.cur_N = temp;
    temp = matching_times; matching_times = other.matching_times; other.matching_times = temp;

    unsigned char tempc;
    for (int i = 0; i < 3; i++) {
      tempc = rgb[i];
      rgb[i] = other.rgb[i];
      other.rgb[i] = tempc;
    }
  }
  ;

  void print(void)
  {
    printf("\tweight: %3.5f, mu: %3.5f, covar: %3.5f, fitness: %3.5f, cur_N: %d, matching_times: %d.\n",
        weight, mu, covar, fitness, (int)cur_N, (int)matching_times);
    return;
  }

  float weight;
  float mu; // the mean value
  float covar; // the covariance
  float fitness; // weight/sqrt(covariance)
  float cur_N;
  float matching_times;

  unsigned char rgb[3];

private:

};

//-----------------------------------------------------------------------------
// depth_Gaussian_Kernel: A component of depth_MoG
//-----------------------------------------------------------------------------
class depth_MoG {

public:
  depth_MoG() {
    kernels.clear();
  }
  ;

  ~depth_MoG() {
    kernels.clear();
  }
  ;

  // resort all kernerls by their fitness
  void resort_by_fitness() {
    int n = kernels.size();
    if (n < 2)
      return;

    if (n == 2) {
      if (kernels[0].fitness < kernels[1].fitness)
        kernels[0].exchange(kernels[1]);
      return;
    }

    for (int i = 0; i < n - 1; i++)
      for (int j = 0; j < n - i - 1; j++) {
        if (kernels[j].fitness < kernels[j + 1].fitness)
          kernels[j].exchange(kernels[j + 1]);
      }

    return;
  }
  ;

  // resort all kernerls by their weight
  void resort_by_weight() {
    int n = kernels.size();
    if (n < 2)
      return;

    if (n == 2) {
      if (kernels[0].weight < kernels[1].weight)
        kernels[0].exchange(kernels[1]);
      return;
    }

    for (int i = 0; i < n - 1; i++)
      for (int j = 0; j < n - i - 1; j++) {
        if (kernels[j].weight < kernels[j + 1].weight)
          kernels[j].exchange(kernels[j + 1]);
      }

    return;
  }
  ;


  void update(float value, unsigned char r, unsigned char g,
      unsigned char b) {

    int component_num = kernels.size();
    bool found = false;

    // find the first match and update all kernels
    for (int i = 0; i < component_num; i++) {

      if (!found) {
        if (kernels[i].isMatch(value)) {
          kernels[i].positive_update(value, r, g, b);
          found = true;
        } else
          kernels[i].negtive_update(value);
      } else
        kernels[i].negtive_update(value);
    }

    // re-sort all the Gaussian kernel component
    resort_by_fitness();

    // if no match is found,
    if (!found) {
      // create a new one
      depth_Gaussian_Kernel new_kernel(value, r, g, b);

      // if there are too many components, remove the one with the least fitness
      if(kernels.size() >= MOG_COMPONENT_NUM_MAX)
        kernels.pop_back();

      // add the new one
      kernels.push_back(new_kernel);
    }

    // normalize all the weight so that the sum of weight is 1.0
    component_num = kernels.size();
    float weight_sum = 0.0f;
    for (int i = 0; i < component_num; i++)
      weight_sum += kernels[i].weight;

    if (weight_sum > 0) {
      for (int i = 0; i < component_num; i++)
        kernels[i].weight /= weight_sum;
    }

  }
  ;

  void reset(void) {
    kernels.clear();
  }
  ;

  void print(void)
  {
    for(int i=0; i<kernels.size(); i++)
    {
      kernels[i].print();
    }
  };

  void conclude(float* mean, float* sigma, unsigned char* r, unsigned char* g, unsigned char* b)
  {
    int n = kernels.size();

    if(n == 0)
    {
      *mean = 0;
      *sigma = SIGMA_INIT;
      *r = 0;
      *g = 0;
      *b = 0;
      return;
    }

    // search in the top three (in terms of weight) components that have the largest mu
    resort_by_weight();

    float cur_mu_max = 0;
    int cur_mu_idx = 0;
    n = (n>3)?3:n;
    for(int i=0; i<n; i++)
    {
      if(kernels[i].mu > cur_mu_max)
      {
        cur_mu_max = kernels[i].mu;
        cur_mu_idx = i;
      }
    }

    *mean = kernels[cur_mu_idx].mu;
    *sigma = sqrt(kernels[cur_mu_idx].covar);
    *r = kernels[cur_mu_idx].rgb[0];
    *g = kernels[cur_mu_idx].rgb[1];
    *b = kernels[cur_mu_idx].rgb[2];

    return;
  };

private:

  std::vector<depth_Gaussian_Kernel> kernels;

};

class depth_MoG_background {

public:
  depth_MoG_background(int rows, int cols);
  ~depth_MoG_background();
  void update(const unsigned short* depthImg, int depth_w, int depth_h,
      const unsigned char * rgbImg, int rgb_w, int rgb_h);
  void learn(const unsigned short* depthImg, int depth_w, int depth_h,
      const unsigned char * rgbImg, int rgb_w, int rgb_h);
  void construct();
  void clear();
  bool save(char* filename) const;
  bool load(char* filename);
  void resize(int rows, int cols);
  bool importFromDepthMap(unsigned short* depth, int w, int h);
  void start_learning();
  void finish_learning();
  float* getDepthMean();
  float* getDepthSigma();
  unsigned char* getRGBData();
  void computeForeground(const unsigned short* depthImg, int depth_w,
      int depth_h);
  bool* getForeground(void);
  bool modelAvailable(void);
  void increaseSensitivity(void);
  void decreaseSensitivity(void);
  bool get3DPointFromPatch(int x, int y, int patchSize, Point3D& output);

  ImageIndexer idx;

  //////// ransac asset///////////
  int ransac_point_num;
  int *mapVec;
  Point3D *ransacDataBuf;
  unsigned char *rgbBuf;
  ///////////////////////////////

private:

  void conclude(void);

  vector<depth_MoG> MoG_buf;
  float* dist_mean;
  float* dist_sigma;
  unsigned char* avg_rgb;

  bool* fgmask;

  int learning_frame_cnt;
  bool is_empty;

  float fg_detection_th;
};
