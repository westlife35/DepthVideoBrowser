#pragma once

// all common things (but not easy to be catogorized) can be defined here.

#define PAI 3.14159265358f

#define PRIMESENSE_DEPTH_VALUE_MAX 10000

/** Holds the value of a single 3d point */
typedef struct {
  float x;
  float y;
  float z;
} Point3D;

typedef struct {
  float x;
  float y;
} Point2Df;

typedef struct {
  int x;
  int y;
} Point2Di;

typedef struct {
  int x;
  int y;
} Point2D;

//typedef struct {
//  int x0;
//  int y0;
//  int x1;
//  int y1;
//} Rect;

//#define min(a,b) ((a)<(b)?(a):(b))
//#define max(a,b) ((a)>(b)?(a):(b))
/*
#define abs(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x)*(x))
*/
