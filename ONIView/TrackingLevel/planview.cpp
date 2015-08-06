#include <stdio.h>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>
//I comment the sectence below only one row
//#include "util/configfile.h"
// #include "alg/processor.h"
#include "planview.h"
#include "colormap.hpp"

using namespace cv;

namespace PLANVIEW
{
  // global variables of PLANVIEW
  // transformation from XY plane of floorCoords to floor_map
  float scaleFloorCoords2FloorMap;
  float txFloorCoords2FloorMap, tyFloorCoords2FloorMap;

  // algorithm parameters
  float floorMapPhysicalSize;
  float unit_per_meter;
  float floor_map_front;
  float meter_per_unit;
  float planview_average_object_size;
  int box_filter_size;

  // floor maps
  Size floor_map_resolution;
  Mat heightMap, heightMapVis;
  Mat occupancyMap, occupancyMapVis;
  Mat heightWeightedOccupancyMap, heightWeightedOccupancyMapVis;
  Mat smoothedHeightWeightedOccupancyMap, smoothedHeightWeightedOccupancyMapVis;

  // visualization windows
  char* window_name_height_map = "Height Map";
  char* window_name_occupancy_map = "Occupancy Map";
  char* window_name_height_weighted_occupancy_map = "Height Weighted Occupancy Map";
  char* window_name_smoothed_height_weighted_occupancy_map = "Smoothed Height Weighted Occupancy Map";
  bool debugVisualizationOn = false;


  // object detection
  vector<struct pixel_loc_value> localMax;
  vector<struct pixel_loc_value> dominantLocalMax;
  vector<struct detected_object> detectedObjects;

  // ----------------------------------------------------
  // reduce the set of points to local dominant ones, based on distance and value
  // input: a set of 2D points and their associated values
  // output: dominant points
  // dist_th: a threshold of distance in which only one dominant point can exist.
  void donimantPointReduction2D(const vector<struct pixel_loc_value>& input, vector<struct pixel_loc_value>& output, float dist_th)
  {
    int input_point_number = input.size();
    output.clear();

    if(input_point_number == 0)
      return;

    vector<struct pixel_loc_value> pointBuf = input;
    
    // reduce the non-dominant points
    for(int i=0; i<input_point_number; i++)
    {
      if(pointBuf[i].value < 0)
        continue;
      for(int j=i+1; j<input_point_number; j++)
      {
        if(pointBuf[j].value < 0)
          continue;

        float dist = sqrtf((float)((pointBuf[i].col-pointBuf[j].col)*(pointBuf[i].col-pointBuf[j].col) 
                    + (pointBuf[i].row-pointBuf[j].row)*(pointBuf[i].row-pointBuf[j].row)));

        if(dist < dist_th)
        {
          if(pointBuf[i].value > pointBuf[j].value)
            pointBuf[j].value = -1.0f;
          else if(pointBuf[i].value == pointBuf[j].value)
          {
            // if two point have equal value, kill the second and move the first to the middle of two points
            pointBuf[i].row = (pointBuf[i].row + pointBuf[j].row)/2;
            pointBuf[i].col = (pointBuf[i].col + pointBuf[j].col)/2;
            pointBuf[j].value = -1.0f;
          }
          else
            pointBuf[i].value = -1.0f;
        }
      }
    }

    // collect the remaining points from the above reduction
    for(int i=0; i<input_point_number; i++)
    {
      if(pointBuf[i].value > 0)
        output.push_back(pointBuf[i]);
    }

  }

  // ----------------------------------------------------
  // 2D Non Maximum Surpression
  // search NonMaximum points from a sparse grid and then refine the location recursively
  // input: 2D input image of type 32FC1
  // low_threshold: the minimum threshold of qualified Non-Maximum point
  // output: a list of output Non-Maximum points
  void NMS2D_sparse(const Mat& input, float low_threshold, int searching_grid_size, vector<struct pixel_loc_value>& output)
  {
    if(input.type() != CV_32FC1)
    {
      printf("Error in NMS2D_sparse: data type of input array must be CV_32FC1.\n");
      return;
    }

    int width = input.size().width;
    int height = input.size().height;
    int left = searching_grid_size;
    int right = width - searching_grid_size;
    int top = searching_grid_size;
    int bottom = height - searching_grid_size;

    output.clear();

    for(int row=top; row<bottom; row+=searching_grid_size)
    for(int col=left; col<right; col+=searching_grid_size)
    {
      // compare with 8 sparse neighbors
      float value = input.at<float>(row, col);
      if(value >= low_threshold)
      {
        bool isMax = true;
        for(int i=-searching_grid_size; i<=searching_grid_size; i+=searching_grid_size)
        {
          for(int j=-searching_grid_size; j<=searching_grid_size; j+=searching_grid_size)
          {
            if(value < input.at<float>(row+i, col+j))
            {
              isMax = false;
              break;
            }
          }
          if(isMax == false)
            break;
        }

        if(isMax)
        {
          // recursively refine the position;
          int refined_row = row;
          int refined_col = col;
          float refined_value = value;
          float refine_grid = 0.5f*(float)searching_grid_size;
          while(refine_grid > 1.0f)
          {
            int gs = (int)refine_grid;
            int candidate_row = refined_row;
            int candidate_col = refined_col;
            for(int i=-gs; i<=gs; i+=gs)
            for(int j=-gs; j<=gs; j+=gs)
            {
              if(i==0 && j==0)
                continue;

              if(input.at<float>(refined_row+i, refined_col+j) > refined_value)
              {
                refined_value = input.at<float>(candidate_row, candidate_col);
                candidate_row = refined_row+i;
                candidate_col = refined_col+j;
              }
            }
            refined_row= candidate_row;
            refined_col = candidate_col;

            refine_grid *= 0.5f;
          }

          // add refined value to output
          pixel_loc_value point;
          point.row = refined_row;
          point.col = refined_col;
          point.value = refined_value;
          output.push_back(point);
        }
      }
    }

  }

  void generateVisualization(const Mat& input, Mat& output, float maxValue = -1.0f)
  {
    //make sure that input and output buffer match on size
    if(input.size != output.size)
    {
      printf("Error in PLANVIEW::generateVisualization: input buffers do not match on size.\n");
      return;
    }

    float max;

    // get the max value in data
    float data_max = -1.0f;
    for(int i = 0; i < input.rows; i++)
    {
        const float* Mi = input.ptr<float>(i);
        for(int j = 0; j < input.cols; j++)
          data_max = (Mi[j]>data_max)?Mi[j]:data_max;
    }


    // if no valid maxValue is provided, find the maxValue in the input image
    if(maxValue == -1.0f)
      max = data_max;
    else
      max = (maxValue>data_max)?maxValue:data_max;

    //printf("max value %3.3f.\n", max);

    Mat normalizedImg;
    input.convertTo(normalizedImg, CV_8UC1, (255.0f/max));
    applyColorMap(normalizedImg, output, COLORMAP_WINTER);
  }

  void drawCirclesOnImage(Mat& image, const vector<struct pixel_loc_value>& points, int radius, Scalar color, int thickness=1)
  {

    for(int i=0; i<points.size(); i++)
    {
      Point p(points[i].col, points[i].row);
      circle(image, p, radius, color, thickness);
    }

  }

  void drawCrossMarksOnImage(Mat& image, const vector<struct pixel_loc_value>& points, int radius, Scalar color, int thickness=1)
  {
    int w = image.size().width;
    int h = image.size().height;
    int cross_size = (radius/2)+1;
    for(int i=0; i<points.size(); i++)
    {
      int top = points[i].row - cross_size;
      top = (top<0)?0:top;
      int bottom = points[i].row + cross_size;
      bottom = (bottom>=h)?h-1:bottom;
      int left = points[i].col - cross_size;
      left = (left<0)?0:left;
      int right = points[i].col + cross_size;
      right = (right>w)?w-1:right;

      Point p1(points[i].col, top);
      Point p2(points[i].col, bottom);
      Point p3(left, points[i].row);
      Point p4(right, points[i].row);
      line(image, p1, p2, color, thickness);
      line(image, p3, p4, color, thickness);
    }
  }

  void floorMapGeneration(int pointNum, const Eigen::MatrixXf& pointsInCamCoords, const Eigen::MatrixXf& pointsInFloorCoords, 
                          Mat& height, Mat& occupany, Mat& heightWeightedOccupancy)
  {
    // first make sure all input buffers have the same size;
    if(  pointsInCamCoords.rows() !=  pointsInFloorCoords.rows() 
      || pointsInCamCoords.cols() !=  pointsInFloorCoords.cols() )
    {
      printf("Error in PLANVIEW::floorMapGeneration: input buffers do not match on size.\n");
      return;
    }
    // first make sure all output floormaps has the same size;
    if(  height.size() != occupany.size()
      || height.size() != heightWeightedOccupancy.size())
    {
      printf("Error in PLANVIEW::floorMapGeneration: output buffers do not match on size.\n");
      return;
    }

    float s, tx, ty;
    s = scaleFloorCoords2FloorMap;
    tx = txFloorCoords2FloorMap;
    ty = tyFloorCoords2FloorMap;

    int img_w,img_h;
    img_w = height.size().width;
    img_h = height.size().height;

    float* heightMapBuf = (float*)(height.data);
    float* occupanyMapBuf = (float*)(occupany.data);
    float* heightWeightedOccupancyMapBuf = (float*)(heightWeightedOccupancy.data);
    for(int i=0; i<pointNum; i++)
    {
      float x = s*pointsInFloorCoords(i,0) + tx;
      float y = s*pointsInFloorCoords(i,1) + ty;
      float top = pointsInFloorCoords(i,2)*meter_per_unit;

      if(top > 2.0f)
        continue;

      int x_rnd = (int)x;
      int y_rnd = (int)y;

      if(x_rnd>=0 && x_rnd<img_w && y_rnd>=0 && y_rnd<img_h)
      {
        // update height map
        float old_top = height.at<float>(y_rnd, x_rnd);
        if(top>old_top)
          height.at<float>(y_rnd, x_rnd) = top;

        // update occupancy map
        float x_cam = pointsInCamCoords(i,0)*meter_per_unit;
        float y_cam = pointsInCamCoords(i,1)*meter_per_unit;
        float z_cam = pointsInCamCoords(i,2)*meter_per_unit;
        float occupancy = x_cam*x_cam + y_cam*y_cam + z_cam*z_cam;
        occupany.at<float>(y_rnd, x_rnd) += occupancy;

        // update height weighted occupancy map
        heightWeightedOccupancy.at<float>(y_rnd, x_rnd) += (occupancy*top);
      }
    }

  }

  // convert dominantLocalMax to detected objects
  void findObjectsFromLocaDominantPoints(void)
  {
    int img_w = heightMap.size().width;
    int img_h = heightMap.size().height;

    detectedObjects.clear();

    // get the transformation from floor_map to 3D world coords;
    float s = 1.0f/scaleFloorCoords2FloorMap;
    float tx = -s*txFloorCoords2FloorMap;
    float ty = -s*tyFloorCoords2FloorMap;

    for(int i=0; i<dominantLocalMax.size(); i++)
    {
      int center_row = dominantLocalMax[i].row;
      int center_col = dominantLocalMax[i].col;

      // find the highest point in the local neighborhood
      int neighboorhood_halfsize = box_filter_size/2;
      int l,r,t,b;
      l = center_col-neighboorhood_halfsize; l = (l<0)?0:l;
      r = center_col+neighboorhood_halfsize; r = (r>img_w)?img_w-1:r;
      t = center_row-neighboorhood_halfsize; t = (t<0)?0:t;
      b = center_row+neighboorhood_halfsize; b = (b>img_h)?img_h-1:b;

      float summit = 0.0f;
      int summit_x=0;
      int summit_y=0;
      for(int row=t; row<b; row++)
      {
        const float* row_buf = heightMap.ptr<float>(row);
        for(int col=l; col<r; col++)
        {
          float cur_height = row_buf[col];
          if(cur_height > summit)
          {
            summit = cur_height;
            summit_x = col;
            summit_y = row;
          }
        }
      }

      if(summit > 0) // find a head
      {
        detected_object object;
        object.headTopInWorld(0) = s*summit_x + tx;       // X
        object.headTopInWorld(1) = s*summit_y + ty;       // Y
        object.headTopInWorld(2) = summit*unit_per_meter; // Z

        object.centroidInWorld(0) = s*center_col + tx;    // X
        object.centroidInWorld(1) = s*center_row + ty;    // Y
        object.centroidInWorld(2) = 0;                    // Z (not meaningful here)

        detectedObjects.push_back(object);
      }
      
    }

  }

  void init(void)
  {
    floorMapPhysicalSize = 10.0;
    unit_per_meter = 917;
    floor_map_front = 1.0;
    meter_per_unit = 1.0f/unit_per_meter;

    floor_map_resolution.width = 512;
    floor_map_resolution.height = floor_map_resolution.width;

    planview_average_object_size = 0.25;
    box_filter_size = (int)(planview_average_object_size*floor_map_resolution.width/floorMapPhysicalSize);
    box_filter_size = box_filter_size + 1 - (box_filter_size%2); // make sure this is an odd number

    heightMap.create(floor_map_resolution, CV_32FC1);
    occupancyMap.create(floor_map_resolution, CV_32FC1);
    heightWeightedOccupancyMap.create(floor_map_resolution, CV_32FC1);

    heightMapVis.create(floor_map_resolution, CV_8UC3);
    occupancyMapVis.create(floor_map_resolution, CV_8UC3);
    heightWeightedOccupancyMapVis.create(floor_map_resolution, CV_8UC3);

    smoothedHeightWeightedOccupancyMap.create(floor_map_resolution, CV_32FC1);
    smoothedHeightWeightedOccupancyMapVis.create(floor_map_resolution, CV_8UC3);

    // figure out the 2D transformation from real-world coordinates to floor map
    scaleFloorCoords2FloorMap = (float)floor_map_resolution.width/(floorMapPhysicalSize*unit_per_meter);
    txFloorCoords2FloorMap = 0.5f*floor_map_resolution.width;
    tyFloorCoords2FloorMap = -floor_map_front * (float)floor_map_resolution.width / floorMapPhysicalSize;

    //printf("Scale:%3.3f, tx:%3.3f, ty:%3.3f.\n", scaleFloorCoords2FloorMap, txFloorCoords2FloorMap, tyFloorCoords2FloorMap);
  }

  void switchDebugVisualization(void)
  {
    debugVisualizationOn = !debugVisualizationOn;
    if(debugVisualizationOn)
    {
      namedWindow(window_name_height_map, 1); //CV_WINDOW_NORMAL);
      namedWindow(window_name_occupancy_map, 1); //CV_WINDOW_NORMAL);
      namedWindow(window_name_height_weighted_occupancy_map, 1); //CV_WINDOW_NORMAL);
    }
    else
    {
      destroyWindow(window_name_height_map);
      destroyWindow(window_name_occupancy_map);
      destroyWindow(window_name_height_weighted_occupancy_map);
    }

    // somehow this delay is necessary in Linux. Otherwise these CV windows can not be shown.
    cvWaitKey(10);

  }


  void update(int pointNumber, const Eigen::MatrixXf& pointsInCam, const Eigen::MatrixXf& pointsInWorld)
  {
    // ---------------------------------
    // clear buffers
    heightMap = Scalar::all(0);
    occupancyMap = Scalar::all(0);
    heightWeightedOccupancyMap = Scalar::all(0);

    // ---------------------------------
    // generat height map, occupancy map and heightWeightedOccupancyMap
    floorMapGeneration(pointNumber, pointsInCam, pointsInWorld,
                      heightMap, occupancyMap, heightWeightedOccupancyMap);

    // ---------------------------------
    // smooth heightWeightedOccupancyMap using boxFilter
    Size boxFilterSize(25,25);
    boxFilter(heightWeightedOccupancyMap, smoothedHeightWeightedOccupancyMap, -1, boxFilterSize);

    // ---------------------------------
    // apply Non Maximum Surpression on smoothedHeightWeightedOccupancyMap
    NMS2D_sparse(smoothedHeightWeightedOccupancyMap, 40.f, box_filter_size/2, localMax);
    donimantPointReduction2D(localMax, dominantLocalMax, 0.5*(float)(box_filter_size));

    //----------------------------------
    // convert dominantLocalMax to detected objects
    findObjectsFromLocaDominantPoints();
  }

  void visualizationUpdate(void)
  {
    if(!debugVisualizationOn)
      return;

    // -----------------------------------
    // visualizations
    generateVisualization(heightMap, heightMapVis, 2.0f);
    generateVisualization(occupancyMap, occupancyMapVis, 500.0f);
    generateVisualization(heightWeightedOccupancyMap, heightWeightedOccupancyMapVis, 500.0f);
    generateVisualization(smoothedHeightWeightedOccupancyMap, smoothedHeightWeightedOccupancyMapVis, 100.0f);
    drawCirclesOnImage(smoothedHeightWeightedOccupancyMapVis, localMax, 10, Scalar(1.0, 1.0, 1.0), 2);
    drawCrossMarksOnImage(smoothedHeightWeightedOccupancyMapVis, dominantLocalMax, 10, Scalar(0.0, 0.0, 0.0), 2);

    // debug visulaization
    imshow(window_name_height_map, heightMapVis);
    imshow(window_name_occupancy_map, occupancyMapVis);
    imshow(window_name_height_weighted_occupancy_map, heightWeightedOccupancyMapVis);
    imshow(window_name_smoothed_height_weighted_occupancy_map, smoothedHeightWeightedOccupancyMapVis);

    // somehow this delay is necessary in Linux. Otherwise these CV windows can not be shown.
    cvWaitKey(3);

  }

  void release(void)
  {
    heightMap.release();
    heightMapVis.release();
    occupancyMap.release();
    occupancyMapVis.release();
    heightWeightedOccupancyMap.release();
    heightWeightedOccupancyMapVis.release();
    smoothedHeightWeightedOccupancyMap.release();
    smoothedHeightWeightedOccupancyMapVis.release();

    // destory the debug window
    destroyWindow(window_name_height_map);
    destroyWindow(window_name_occupancy_map);
    destroyWindow(window_name_height_weighted_occupancy_map);
    destroyWindow(window_name_smoothed_height_weighted_occupancy_map);
  }

}
