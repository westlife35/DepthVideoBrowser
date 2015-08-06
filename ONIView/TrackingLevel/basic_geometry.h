#pragma once
#include <vector>
//#include <Eigen/Core>
//#include <Eigen/Dense>
#include "util/common_things.h"

using namespace std;

//-----------------------------------------------------------------------------
class plane3D {
public:
  plane3D(){};
  ~plane3D(){};
  plane3D(const vector<Point3D>& points);
  plane3D(Point3D p1, Point3D p2, Point3D p3);

  void getProjection(const Point3D& point, Point3D& projection);
  float distance(const Point3D& point);

  Point3D calPlaneLineIntersectPoint(Point3D pointA,Point3D pointB);
  void calPlaneLineIntersectPoint(vector<float> &planeParameters, vector<float> &lineParameters, Point3D &intersactPnt);
  // reference: http://www.9math.com/book/projection-point-plane
  // reference: http://mathworld.wolfram.com/Point-PlaneDistance.html
  // one way of represent a 3D plane is A*x + B*y + C*z + D = 0;
  // or A*x + B*y + C*z + 1 = 0, if you want to remove one parameter
  // if |(A, B, C)| = 1, then D is the distance between origin to the plane
  // in this class we will keep the D, but make the |(A, B, C)| = 1, and (A, B, C) is the normal of the plance
  float A, B, C, D;
};

class polygon2D {
public:
	polygon2D();
	~polygon2D();
	polygon2D(const vector<Point2Df>& points);

	bool isInsidePoint(const Point2Df point);
	void addCorner(const Point2Df point);
	int getNumberOfCorners();

	vector<Point2Df> corners;
};
