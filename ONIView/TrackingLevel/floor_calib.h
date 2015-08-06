#pragma once
#include <vector>
//#include <Eigen/Core>
#include "Eigen/Core"
#include <Eigen/Dense>
#include "basic_geometry.h"
#include "util/tools.h"
#include "util/common_things.h"
#include "opencv2/core/core.hpp"
#include "opencv2/legacy/legacy.hpp"

#define SMOOTHED_HEIGHT_WEIGHTED_OCCUPANCY_THRESHOLD      20.f   // this should be adjustable in the future.
#define MAX_HEIGHT                        2.0f

using namespace std;

//-----------------------------------------------------------------------------
class floor_calib {

  public:
    floor_calib();
    ~floor_calib();
    
    // calibrate floor coordinate system from at least 3 points on a same plane which is parrallel to floor plane
    void calib(vector<float> &planeParameters, float height);
    void calib(const vector<Point3D>& points, float height);
    void cam2plane(const vector<Point3D>& inputPoints, vector<Point3D>& outputPoints);
    void plane2cam(const vector<Point3D>& inputPoints, vector<Point3D>& outputPoints);
    void cam2floor(const vector<Point3D>& inputPoints, vector<Point3D>& outputPoints);
    void cam2floor(Point3D& inputPoint, Point3D& outputPoint);
    void cam2floor(const Eigen::MatrixXf & inputPoints,  Eigen::MatrixXf & outputPoints, int pointNum);
    void cam2floor(const float* inputPoints,  float* outputPoints, int pointNum);
    void floor2cam(const Point3D& inputPoint, Point3D& outputPoint);
    void floor2cam(const vector<Point3D>& inputPoints, vector<Point3D>& outputPoints);
    void floor2cam(const Eigen::Vector3f& inputPoint, Eigen::Vector3f& outputPoint);
    void floor2cam(const vector<Eigen::Vector3f>& inputPoints, vector<Eigen::Vector3f>& outputPoints);
    bool isAvailable(void) {return available;}
    void clear(void);
    bool save(char* filename) const;
    bool load(char* filename);

    vector<Point3D> inputPoints;
    vector<float> planeParamVectorPoint;
    vector<float> planeParamABCD;

private:
    bool available;
    float inputPointsHeightFromFloor;
    cv::Mat R_cam2plane, T_cam2plane;
    cv::Mat R_plane2cam, T_plane2cam;
    cv::Mat R_cam2floor, T_cam2floor;
    cv::Mat R_floor2cam, T_floor2cam;

};
