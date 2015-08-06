#include "floor_calib.h"
#include "basic_geometry.h"
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
//#include <Eigen/Core>
//#include <Eigen/Dense>
//#include "util/configfile.h"
#include "opencv2/core/core.hpp"
#include "opencv2/legacy/legacy.hpp"

using namespace std;
using namespace cv;
//------------------------------------------------------------

plane3D::plane3D(const vector<Point3D>& points)
{
  // assume D = 1, vecotr b = [-1, -1, ... -1];
  // vector ABC = [A B C]'
  // and matrix xyz = [x0, y0, z0; x1, y1, z1; ...; xn, yn, zn; ]
  // we have xyz * abc = [-1, -1, ..., -1];
  int pointNum = points.size();
  Mat_<float> xyz(pointNum, 3);
  Mat_<float> b(pointNum, 1);
  Mat_<float> ABC(3, 1);
  for(int i=0; i<pointNum; i++)
  {
    xyz(i, 0) = points[i].x;
    xyz(i, 1) = points[i].y;
    xyz(i, 2) = points[i].z;
    b(i, 0) = -1.0f;
  }

  Mat_<float> invXYZ;
  invert(xyz, invXYZ, DECOMP_SVD); // only method DECOMP_SVD can handle pseudo-inverse

  ABC = invXYZ*b;

  // normalize ABC
  float mag = sqrt(ABC(0,0)*ABC(0,0) + ABC(1,0)*ABC(1,0) + ABC(2,0)*ABC(2,0));
  A = ABC(0,0)/mag;
  B = ABC(1,0)/mag;
  C = ABC(2,0)/mag;
  D = 1.0f/mag;

  if(B<0)
  {
    A = -A;
    B = -B;
    C = -C;
    D = -D;
  }

  //printf("A:%3.3f, B:%3.3f, C:%3.3f, D:%3.3f.\n", A, B, C, D);
}

plane3D::plane3D(Point3D p1, Point3D p2, Point3D p3) {

	// normal = cross(p1-p2, p1-p3);
	Mat_<float> a(3, 1);
	a(0,0) = p1.x - p2.x;
	a(1,0) = p1.y - p2.y;
	a(2,0) = p1.z - p2.z;
	Mat_<float> b(3, 1);
	b(0,0) = p1.x - p3.x;
	b(1,0) = p1.y - p3.y;
	b(2,0) = p1.z - p3.z;
	Mat_<float> normal(3, 1);
	normal = a.cross(b);

	// given any point P on the plane, then dot(normal, P-p1) = 0.
	// nx*(x - p1.x) + ny*(y - p1.y) + nz*(z - p1.z) = 0
	// therefore:
	// A = nx;
	// B = ny;
	// C = nz;
	// D = -(nx*p1.x + ny*p1.y + nz*p1.z);
	float nx = normal(0,0);
	float ny = normal(1,0);
	float nz = normal(2,0);
	A = nx;
	B = ny;
	C = nz;
	D = -(nx*p1.x + ny*p1.y + nz*p1.z);

	float avg_value = (fabs(A) + fabs(B) + fabs(C) + fabs(D))/4.0f;
	float refactor = 10.0f/avg_value;

	A *= refactor;
	B *= refactor;
	C *= refactor;
	D *= refactor;

}

void plane3D::getProjection(const Point3D& point, Point3D& projection) {
	float t0 = -(A * point.x + B * point.y + C * point.z + D)
			/ (A * A + B * B + C * C);
	projection.x = point.x + A * t0;
	projection.y = point.y + B * t0;
	projection.z = point.z + C * t0;
}

float plane3D::distance(const Point3D& point) {
	float l = A * point.x + B * point.y + C * point.z + D;
	return l * l / ((A * A + B * B + C * C));
}

Point3D plane3D::calPlaneLineIntersectPoint(Point3D pointA, Point3D pointB) {
	Point3D result;
	vector<float> planeParameters(6);
	vector<float> lineParameters(6);

	planeParameters[0] = A;
	planeParameters[1] = B;
	planeParameters[2] = C;

	planeParameters[3] = 1;
	planeParameters[4] = 1;
	planeParameters[5] = (0 - D - A - B) / C;

	lineParameters[3] = pointA.x;
	lineParameters[4] = pointA.y;
	lineParameters[5] = pointA.z;

	lineParameters[0] = pointA.x - pointB.x;
	lineParameters[1] = pointA.y - pointB.y;
	lineParameters[2] = pointA.z - pointB.z;
	//printf( "--calPlaneLineIntersectPoint %f %f %f \n",result.x,result.y,result.z);
	calPlaneLineIntersectPoint(planeParameters, lineParameters, result);
	//printf( "--calPlaneLineIntersectPoint %f %f %f \n",result.x,result.y,result.z);
	return result;
}

void plane3D::calPlaneLineIntersectPoint(vector<float> &planeParameters,
		vector<float> &lineParameters, Point3D &intersactPnt) {
	float vp1, vp2, vp3, v1, v2, v3;
	float n1, n2, n3, m1, m2, m3;
	float t, vpt;
	float x, y, z;

	vp1 = planeParameters[0];
	vp2 = planeParameters[1];
	vp3 = planeParameters[2];
	n1 = planeParameters[3];
	n2 = planeParameters[4];
	n3 = planeParameters[5];

	v1 = lineParameters[0];
	v2 = lineParameters[1];
	v3 = lineParameters[2];
	m1 = lineParameters[3];
	m2 = lineParameters[4];
	m3 = lineParameters[5];

	//printf("----%f %f %f %f %f %f \n",v1,v2,v3,vp1,vp2,vp3);
	//printf("----%f %f %f %f %f %f \n",m1,m2,m3,n1 ,n2,n3);
	vpt = v1 * vp1 + v2 * vp2 + v3 * vp3;

	//parallel
	if (vpt == 0)
		return;  // TODO  : return point

	t = ((n1 - m1) * vp1 + (n2 - m2) * vp2 + (n3 - m3) * vp3) / vpt;

	intersactPnt.x = m1 + v1 * t;
	intersactPnt.y = m2 + v2 * t;
	intersactPnt.z = m3 + v3 * t;
	//printf("----%f %f %f %f  \n", intersactPnt.x, intersactPnt.y, intersactPnt.z,t);
}


polygon2D::polygon2D() {
	corners.clear();
}

polygon2D::~polygon2D() {
	corners.clear();
}

polygon2D::polygon2D(const vector<Point2Df>& points) {

	int pointNum = points.size();
	for(int i=0; i<pointNum; i++)
		corners.push_back(points[i]);
}

void polygon2D::addCorner(const Point2Df point) {
	corners.push_back(point);
}

int polygon2D::getNumberOfCorners() {
	return corners.size();
}

// this algorithm comes from http://alienryderflex.com/polygon/
bool polygon2D::isInsidePoint(const Point2Df point) {
	float x = point.x;
	float y = point.y;
	int i, j = corners.size() - 1;
	bool oddNodes = false;

	for (i = 0; i < corners.size(); i++) {
		if (corners[i].y < y && corners[j].y >= y || corners[j].y < y && corners[i].y >= y) {
			if (corners[i].x + (y - corners[i].y) / (corners[j].y - corners[i].y)
							* (corners[j].x - corners[i].x) < x) {
				oddNodes = !oddNodes;
			}
		}
		j = i;
	}
	return oddNodes;
}
