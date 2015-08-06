#include "floor_calib.h"
#include <iostream>
#include <fstream>
#include <math.h>
// #include <Eigen/Core>
// #include <Eigen/Dense>
//#include "util/configfile.h"

using namespace std;
using namespace cv;

//------------------------------------------------------------
floor_calib::floor_calib() {
	available = false;
}

floor_calib::~floor_calib() {

}

void computeRigidBodyTransformation(const vector<Point3D> pointSetSrc,
		const vector<Point3D> pointSetDes, Mat& R, Mat& T) {
	// fit rigid body transformation from two sets of points
	// reference: http://www.kwon3d.com/theory/jkinem/rotmat.html

	int pointNum = pointSetSrc.size();
	if (pointNum != pointSetDes.size()) {
		printf(
				"Error in computeRigidBodyTransformation: pointSetSrc and pointSetDes must be the same size\n");
		return;
	}

	// compute centroid the input points
	Point3D centroidSrc, centroidDes;
	centroidSrc.x = 0.0f;
	centroidSrc.y = 0.0f;
	centroidSrc.z = 0.0f;
	centroidDes.x = 0.0f;
	centroidDes.y = 0.0f;
	centroidDes.z = 0.0f;
	for (int i = 0; i < pointNum; i++) {
		centroidSrc.x += pointSetSrc[i].x;
		centroidSrc.y += pointSetSrc[i].y;
		centroidSrc.z += pointSetSrc[i].z;
		centroidDes.x += pointSetDes[i].x;
		centroidDes.y += pointSetDes[i].y;
		centroidDes.z += pointSetDes[i].z;
	}
	centroidSrc.x /= (float) pointNum;
	centroidSrc.y /= (float) pointNum;
	centroidSrc.z /= (float) pointNum;
	centroidDes.x /= (float) pointNum;
	centroidDes.y /= (float) pointNum;
	centroidDes.z /= (float) pointNum;

	// points clouds with centroid at origin
	Mat_<float> src(3, pointNum);
	Mat_<float> des(3, pointNum);
	for (int i = 0; i < pointNum; i++) {
		src(0, i) = pointSetSrc[i].x - centroidSrc.x;
		src(1, i) = pointSetSrc[i].y - centroidSrc.y;
		src(2, i) = pointSetSrc[i].z - centroidSrc.z;
		des(0, i) = pointSetDes[i].x - centroidDes.x;
		des(1, i) = pointSetDes[i].y - centroidDes.y;
		des(2, i) = pointSetDes[i].z - centroidDes.z;
	}

	Mat_<float> H = des * src.t();
	Mat A, w, u, vt;
	SVD::compute(H, w, u, vt);

	float tmp = determinant(u * vt);
	// if determinant(u*vt) == -1 means we find a reflection
	Mat_<float> C(3, 3);
	C(0, 0) = 1.0f;
	C(0, 1) = 0.0f;
	C(0, 2) = 0.0f;
	C(1, 0) = 0.0f;
	C(1, 1) = 1.0f;
	C(1, 2) = 0.0f;
	C(2, 0) = 0.0f;
	C(2, 1) = 0.0f;
	C(2, 2) = tmp;

	R = vt.t() * C * u.t();
	Mat_<float> ahat(3, 1);
	Mat_<float> bhat(3, 1);
	ahat(0, 0) = centroidSrc.x;
	ahat(1, 0) = centroidSrc.y;
	ahat(2, 0) = centroidSrc.z;
	bhat(0, 0) = centroidDes.x;
	bhat(1, 0) = centroidDes.y;
	bhat(2, 0) = centroidDes.z;
	T = ahat - R * bhat;

	return;
}

bool floor_calib::save(char* filename) const {
	//////////////////////////////////////////////////////
	// open a file for write
	ofstream calib_file;
	calib_file.open(filename, ios::out | ios::binary);
	if (calib_file.fail()) {
		printf("Failed to open calibration file %s.\n", filename);
		return false;
	}

	///////////////////////////////////////////////////////
	// save the 3D points which are used for calibrations.
//  unsigned int pointNum = inputPoints.size();
//  printf("inputPoints size = %d \n", pointNum);
//  calib_file.write((const char*)(&pointNum), sizeof(unsigned int));
//  for(int i=0; i<inputPoints.size(); i++)
//  {
//    float xyz[3];
//    xyz[0] = inputPoints[i].x;
//    xyz[1] = inputPoints[i].y;
//    xyz[2] = inputPoints[i].z;
//    calib_file.write((const char*)xyz, 3*sizeof(float));
//  }

	///////////////////////////////////////////////////////
	// save the ransac plane parameters.
	//calib_file.write((const char*)&planeParamVectorPoint, 6*sizeof(float));
	for (int i = 0; i < 6; ++i) {
		float param = planeParamVectorPoint[i];
		calib_file.write((const char*) &param, sizeof(float));
		//printf(" planeParamVectorPoint[%d] : %f \n", i, param);
	}

	for (int i = 0; i < 4; ++i) {
		float param = planeParamABCD[i];
		calib_file.write((const char*) &param, sizeof(float));
		//printf(" planeParamABCD[%d] : %f \n", i, param);
	}

	///////////////////////////////////////////////////////
	// save R_cam2plane, T_cam2plane
	calib_file.write((const char*) R_cam2plane.data, 9 * sizeof(float));
	calib_file.write((const char*) T_cam2plane.data, 3 * sizeof(float));
	///////////////////////////////////////////////////////
	// save R_plane2cam, T_plane2cam
	calib_file.write((const char*) R_plane2cam.data, 9 * sizeof(float));
	calib_file.write((const char*) T_plane2cam.data, 3 * sizeof(float));
	///////////////////////////////////////////////////////
	// save R_cam2floor, T_cam2floor
	calib_file.write((const char*) R_cam2floor.data, 9 * sizeof(float));
	calib_file.write((const char*) T_cam2floor.data, 3 * sizeof(float));
	///////////////////////////////////////////////////////
	// save R_floor2cam, T_floor2cam
	calib_file.write((const char*) R_floor2cam.data, 9 * sizeof(float));
	calib_file.write((const char*) T_floor2cam.data, 3 * sizeof(float));

	if (calib_file.fail()) {
		printf("Failed to save calibration data to %s.\n", filename);
		return false;
	}

	//////////////////////////////////////////////////////
	// close file
	calib_file.close();

	printf("Save calibration data to %s.\n", filename);
	return true;
}

bool floor_calib::load(char* filename) {
	//////////////////////////////////////////////////////
	// open a file for read
	ifstream calib_file;
	calib_file.open(filename, ios::in | ios::binary);
	if (calib_file.fail()) {
		printf("Failed to open calibration file %s.\n", filename);
		return false;
	}

//  ///////////////////////////////////////////////////////
//  // load the ransac plane parameters.
//  unsigned int pointNum;
//  calib_file.read((char*)(&pointNum), sizeof(unsigned int));
//  inputPoints.clear();
//  for(int i=0; i<pointNum; i++)
//  {
//    float xyz[3];
//    Point3D point;
//    calib_file.read((char*)xyz, 3*sizeof(float));
//    point.x = xyz[0];
//    point.y = xyz[1];
//    point.z = xyz[2];
//    inputPoints.push_back(point);
//  }

	///////////////////////////////////////////////////////
	// load the 3D points which are used for calibrations.
	int i;
	planeParamVectorPoint.clear();
	for (i = 0; i < 6; ++i) {
		float param;
		calib_file.read((char *) &param, sizeof(float));
		planeParamVectorPoint.push_back(param);
		//printf(" planeParamVectorPoint[%d] : %f \n", i, param);
	}

	planeParamABCD.clear();
	for (i = 0; i < 4; ++i) {
		float param;
		calib_file.read((char *) &param, sizeof(float));
		planeParamABCD.push_back(param);
		//printf(" planeParamABCD[%d] : %f \n", i, param);
	}

	///////////////////////////////////////////////////////
	// load R_cam2plane, T_cam2plane
	R_cam2plane = Mat_<float>(3, 3);
	T_cam2plane = Mat_<float>(3, 1);
	calib_file.read((char*) R_cam2plane.data, 9 * sizeof(float));
	calib_file.read((char*) T_cam2plane.data, 3 * sizeof(float));
	///////////////////////////////////////////////////////
	// load R_plane2cam, T_plane2cam
	R_plane2cam = Mat_<float>(3, 3);
	T_plane2cam = Mat_<float>(3, 1);
	calib_file.read((char*) R_plane2cam.data, 9 * sizeof(float));
	calib_file.read((char*) T_plane2cam.data, 3 * sizeof(float));
	///////////////////////////////////////////////////////
	// load R_cam2floor, T_cam2floor
	R_cam2floor = Mat_<float>(3, 3);
	T_cam2floor = Mat_<float>(3, 1);
	calib_file.read((char*) R_cam2floor.data, 9 * sizeof(float));
	calib_file.read((char*) T_cam2floor.data, 3 * sizeof(float));
	///////////////////////////////////////////////////////
	// load R_floor2cam, T_floor2cam
	R_floor2cam = Mat_<float>(3, 3);
	T_floor2cam = Mat_<float>(3, 1);
	calib_file.read((char*) R_floor2cam.data, 9 * sizeof(float));
	calib_file.read((char*) T_floor2cam.data, 3 * sizeof(float));

	if (calib_file.fail()) {
		printf("Failed to load calibration data from %s.\n", filename);
		return false;
	}

	//////////////////////////////////////////////////////
	// close file
	calib_file.close();

	available = true;

	printf("Load calibration data from %s.\n", filename);
	return true;
}

// calib the floor using a set of 3D points which sit on a plane parallel to floor,
// height: the height of the plane above the floor, in native sensor unit (instead of meter)
void floor_calib::calib(const vector<Point3D>& points, float height) {
	if (points.size() < 3) {
		printf("At least three points are needed for floor calibration!\n");
		return;
	}

	inputPoints = points;
	// compute the calibration plane: Ax + By +Cz + D = 0;
	plane3D calibPlane(inputPoints);

	// compute the projection of camera center onto the calib plane
	Point3D camCenter;
	camCenter.x = 0.0f;
	camCenter.y = 0.0f;
	camCenter.z = 0.0f;

	Point3D camCenterProjection;
	calibPlane.getProjection(camCenter, camCenterProjection);

	// compute the normal of X,Y and Z axels of floor coords
	// floor X axis is the projection of camera X axis onto the plane
	// floor Z axis is the plane normal;
	// floor Y axis is decided by floor X and floor Y;
	Point3D planeXprojection, temp;
	temp.x = -1.0f;
	temp.y = 0.0f;
	temp.z = 0.0f;
	calibPlane.getProjection(temp, planeXprojection);
	Point3D planeXNorm, planeYNorm, planeZNorm;
	planeXNorm.x = planeXprojection.x - camCenterProjection.x;
	planeXNorm.y = planeXprojection.y - camCenterProjection.y;
	planeXNorm.z = planeXprojection.z - camCenterProjection.z;
	float norm = sqrt(
			planeXNorm.x * planeXNorm.x + planeXNorm.y * planeXNorm.y
					+ planeXNorm.z * planeXNorm.z);
	planeXNorm.x /= norm;
	planeXNorm.y /= norm;
	planeXNorm.z /= norm;
	planeZNorm.x = calibPlane.A;
	planeZNorm.y = calibPlane.B;
	planeZNorm.z = calibPlane.C;
	// comopute planeYNorm = planeZNorm cross-product planeXNorm
	planeYNorm.x = planeZNorm.y * planeXNorm.z - planeZNorm.z * planeXNorm.y;
	planeYNorm.y = planeZNorm.z * planeXNorm.x - planeZNorm.x * planeXNorm.z;
	planeYNorm.z = planeZNorm.x * planeXNorm.y - planeZNorm.y * planeXNorm.x;

	// compute the rotation and translation between camera coords and floor plane coords. X_floor = R*X_camera + T;
	float axis_size = 100.0f; // pick some value which is not too big, nor too small for numerical precision
	Point3D camXAxis, camYAxis, camZAxis;
	camXAxis.x = 1.0f * axis_size;
	camXAxis.y = 0.0f;
	camXAxis.z = 0.0f;
	camYAxis.x = 0.0f;
	camYAxis.y = 1.0f * axis_size;
	camYAxis.z = 0.0f;
	camZAxis.x = 0.0f;
	camZAxis.y = 0.0f;
	camZAxis.z = 1.0f * axis_size;

	Point3D planeXAxis, planeYAxis, planeZAxis;
	planeXAxis.x = camCenterProjection.x + axis_size * planeXNorm.x;
	planeXAxis.y = camCenterProjection.y + axis_size * planeXNorm.y;
	planeXAxis.z = camCenterProjection.z + axis_size * planeXNorm.z;

	planeYAxis.x = camCenterProjection.x + axis_size * planeYNorm.x;
	planeYAxis.y = camCenterProjection.y + axis_size * planeYNorm.y;
	planeYAxis.z = camCenterProjection.z + axis_size * planeYNorm.z;

	planeZAxis.x = camCenterProjection.x + axis_size * planeZNorm.x;
	planeZAxis.y = camCenterProjection.y + axis_size * planeZNorm.y;
	planeZAxis.z = camCenterProjection.z + axis_size * planeZNorm.z;

	vector<Point3D> camCoordsPoints;
	vector<Point3D> planeCoordsPoints;

	camCoordsPoints.push_back(camXAxis);
	camCoordsPoints.push_back(camYAxis);
	camCoordsPoints.push_back(camZAxis);

	planeCoordsPoints.push_back(planeXAxis);
	planeCoordsPoints.push_back(planeYAxis);
	planeCoordsPoints.push_back(planeZAxis);

	computeRigidBodyTransformation(camCoordsPoints, planeCoordsPoints,
			R_cam2plane, T_cam2plane);

	R_plane2cam = R_cam2plane.inv();
	T_plane2cam = -R_plane2cam * T_cam2plane;

	// compute R_cam2floor and T_cam2floor
	R_cam2floor = R_cam2plane;
	T_cam2floor = T_cam2plane;
	float* z = (float*) T_cam2floor.ptr(2, 0);
	(*z) += height;   // compensate the tripod height

	R_floor2cam = R_cam2floor.inv();
	//T_floor2cam = - R_cam2floor*T_cam2floor;
	T_floor2cam = -R_floor2cam * T_cam2floor;

	available = true;
}

//Overload calib
void floor_calib::calib(vector<float> &planeParameters, float height) {
	// compute the calibration plane: Ax + By +Cz + D = 0 & n dot p
	plane3D calibPlane;
	float resultA, resultB, resultC, resultD, resultX, resultY, resultZ;

	resultA = planeParameters[0];
	resultB = planeParameters[1];
	resultC = planeParameters[2];
	resultX = planeParameters[3];
	resultY = planeParameters[4];
	resultZ = planeParameters[5];
	resultD = -(resultA * resultX + resultB * resultY + resultC * resultZ);

	if (resultB < 0) {
		resultA = -resultA;
		resultB = -resultB;
		resultC = -resultC;
		resultD = -resultD;
	}
//	printf("RANSAC plane parameters [A, B, C, D][X0, Y0, Z0] : [%f, %f, %f, %f], [%f, %f, %f,]\n",
//				resultA, resultB, resultC, resultD, resultX, resultY, resultZ);

	planeParamVectorPoint.clear();
	planeParamVectorPoint.push_back(resultA);
	planeParamVectorPoint.push_back(resultB);
	planeParamVectorPoint.push_back(resultC);
	planeParamVectorPoint.push_back(resultX);
	planeParamVectorPoint.push_back(resultY);
	planeParamVectorPoint.push_back(resultZ);

	planeParamABCD.clear();
	planeParamABCD.push_back(resultA);
	planeParamABCD.push_back(resultB);
	planeParamABCD.push_back(resultC);
	planeParamABCD.push_back(resultD);

	calibPlane.A = resultA;
	calibPlane.B = resultB;
	calibPlane.C = resultC;
	calibPlane.D = resultD;

	// compute the projection of camera center onto the calib plane
	Point3D camCenter;
	camCenter.x = 0.0f;
	camCenter.y = 0.0f;
	camCenter.z = 0.0f;

	Point3D camCenterProjection;
	calibPlane.getProjection(camCenter, camCenterProjection);

	// compute the normal of X,Y and Z axels of floor coords
	// floor X axis is the projection of camera X axis onto the plane
	// floor Z axis is the plane normal;
	// floor Y axis is decided by floor X and floor Y;
	Point3D planeXprojection, temp;
	temp.x = -1.0f;
	temp.y = 0.0f;
	temp.z = 0.0f;
	calibPlane.getProjection(temp, planeXprojection);
	Point3D planeXNorm, planeYNorm, planeZNorm;
	planeXNorm.x = planeXprojection.x - camCenterProjection.x;
	planeXNorm.y = planeXprojection.y - camCenterProjection.y;
	planeXNorm.z = planeXprojection.z - camCenterProjection.z;
	float norm = sqrt(
			planeXNorm.x * planeXNorm.x + planeXNorm.y * planeXNorm.y
					+ planeXNorm.z * planeXNorm.z);
	planeXNorm.x /= norm;
	planeXNorm.y /= norm;
	planeXNorm.z /= norm;
	planeZNorm.x = calibPlane.A;
	planeZNorm.y = calibPlane.B;
	planeZNorm.z = calibPlane.C;
	// comopute planeYNorm = planeZNorm cross-product planeXNorm
	planeYNorm.x = planeZNorm.y * planeXNorm.z - planeZNorm.z * planeXNorm.y;
	planeYNorm.y = planeZNorm.z * planeXNorm.x - planeZNorm.x * planeXNorm.z;
	planeYNorm.z = planeZNorm.x * planeXNorm.y - planeZNorm.y * planeXNorm.x;

	// compute the rotation and translation between camera coords and floor plane coords. X_floor = R*X_camera + T;
	float axis_size = 100.0f; // pick some value which is not too big, nor too small for numerical precision
	Point3D camXAxis, camYAxis, camZAxis;
	camXAxis.x = 1.0f * axis_size;
	camXAxis.y = 0.0f;
	camXAxis.z = 0.0f;
	camYAxis.x = 0.0f;
	camYAxis.y = 1.0f * axis_size;
	camYAxis.z = 0.0f;
	camZAxis.x = 0.0f;
	camZAxis.y = 0.0f;
	camZAxis.z = 1.0f * axis_size;

	Point3D planeXAxis, planeYAxis, planeZAxis;
	planeXAxis.x = camCenterProjection.x + axis_size * planeXNorm.x;
	planeXAxis.y = camCenterProjection.y + axis_size * planeXNorm.y;
	planeXAxis.z = camCenterProjection.z + axis_size * planeXNorm.z;

	planeYAxis.x = camCenterProjection.x + axis_size * planeYNorm.x;
	planeYAxis.y = camCenterProjection.y + axis_size * planeYNorm.y;
	planeYAxis.z = camCenterProjection.z + axis_size * planeYNorm.z;

	planeZAxis.x = camCenterProjection.x + axis_size * planeZNorm.x;
	planeZAxis.y = camCenterProjection.y + axis_size * planeZNorm.y;
	planeZAxis.z = camCenterProjection.z + axis_size * planeZNorm.z;

	vector<Point3D> camCoordsPoints;
	vector<Point3D> planeCoordsPoints;

	camCoordsPoints.push_back(camXAxis);
	camCoordsPoints.push_back(camYAxis);
	camCoordsPoints.push_back(camZAxis);

	planeCoordsPoints.push_back(planeXAxis);
	planeCoordsPoints.push_back(planeYAxis);
	planeCoordsPoints.push_back(planeZAxis);

	computeRigidBodyTransformation(camCoordsPoints, planeCoordsPoints,
			R_cam2plane, T_cam2plane);

	R_plane2cam = R_cam2plane.inv();
	T_plane2cam = -R_plane2cam * T_cam2plane;

	// compute R_cam2floor and T_cam2floor
	R_cam2floor = R_cam2plane;
	T_cam2floor = T_cam2plane;
	float* z = (float*) T_cam2floor.ptr(2, 0);
	(*z) += height;   // compensate the tripod height

	R_floor2cam = R_cam2floor.inv();
	//T_floor2cam = - R_cam2floor*T_cam2floor;
	T_floor2cam = -R_floor2cam * T_cam2floor;

	available = true;
}

void floor_calib::cam2plane(const vector<Point3D>& inputPoints,
		vector<Point3D>& outputPoints) {
	Mat_<float> R(R_cam2plane);
	Mat_<float> T(T_cam2plane);

	int pointNum = inputPoints.size();
	if (pointNum != outputPoints.size()) {
		printf(
				"Error in floor_calib::cam2plane: inputPoints and outputPoints must be the same size!\n");
		return;
	}
	for (int i = 0; i < pointNum; i++) {
		Point3D input = inputPoints[i];
		outputPoints[i].x = R(0, 0) * input.x + R(0, 1) * input.y
				+ R(0, 2) * input.z + T(0, 0);
		outputPoints[i].y = R(1, 0) * input.x + R(1, 1) * input.y
				+ R(1, 2) * input.z + T(1, 0);
		outputPoints[i].z = R(2, 0) * input.x + R(2, 1) * input.y
				+ R(2, 2) * input.z + T(2, 0);
	}
}

void floor_calib::plane2cam(const vector<Point3D>& inputPoints,
		vector<Point3D>& outputPoints) {
	Mat_<float> R(R_plane2cam);
	Mat_<float> T(T_plane2cam);

	int pointNum = inputPoints.size();
	if (pointNum != outputPoints.size()) {
		printf(
				"Error in floor_calib::cam2plane: inputPoints and outputPoints must be the same size!\n");
		return;
	}
	for (int i = 0; i < pointNum; i++) {
		Point3D input = inputPoints[i];
		outputPoints[i].x = R(0, 0) * input.x + R(0, 1) * input.y
				+ R(0, 2) * input.z + T(0, 0);
		outputPoints[i].y = R(1, 0) * input.x + R(1, 1) * input.y
				+ R(1, 2) * input.z + T(1, 0);
		outputPoints[i].z = R(2, 0) * input.x + R(2, 1) * input.y
				+ R(2, 2) * input.z + T(2, 0);
	}
}

void floor_calib::cam2floor(const vector<Point3D>& inputPoints,
		vector<Point3D>& outputPoints) {
	Mat_<float> R(R_cam2floor);
	Mat_<float> T(T_cam2floor);

	int pointNum = inputPoints.size();
	if (pointNum != outputPoints.size()) {
		printf(
				"Error in floor_calib::cam2plane: inputPoints and outputPoints must be the same size!\n");
		return;
	}
	for (int i = 0; i < pointNum; i++) {
		Point3D input = inputPoints[i];
		outputPoints[i].x = R(0, 0) * input.x + R(0, 1) * input.y
				+ R(0, 2) * input.z + T(0, 0);
		outputPoints[i].y = R(1, 0) * input.x + R(1, 1) * input.y
				+ R(1, 2) * input.z + T(1, 0);
		outputPoints[i].z = R(2, 0) * input.x + R(2, 1) * input.y
				+ R(2, 2) * input.z + T(2, 0);
	}
}

void floor_calib::cam2floor(Point3D& inputPoint, Point3D& outputPoint) {
	Mat_<float> R(R_cam2floor);
	Mat_<float> T(T_cam2floor);

	outputPoint.x = R(0, 0) * inputPoint.x + R(0, 1) * inputPoint.y
			+ R(0, 2) * inputPoint.z + T(0, 0);
	outputPoint.y = R(1, 0) * inputPoint.x + R(1, 1) * inputPoint.y
			+ R(1, 2) * inputPoint.z + T(1, 0);
	outputPoint.z = R(2, 0) * inputPoint.x + R(2, 1) * inputPoint.y
			+ R(2, 2) * inputPoint.z + T(2, 0);

}

void floor_calib::cam2floor(const Eigen::MatrixXf & inputPoints,
		Eigen::MatrixXf & outputPoints, int pointNum) {
	// make sure both matrix is large enought
	if (inputPoints.rows() < pointNum || outputPoints.rows() < pointNum) {
		printf(
				"Error in floor_calib::cam2floor, the data buffer is not enought.\n");
		return;
	}

	Mat_<float> R(R_cam2floor);
	Mat_<float> T(T_cam2floor);

	float R00 = R(0, 0);
	float R01 = R(0, 1);
	float R02 = R(0, 2);
	float R10 = R(1, 0);
	float R11 = R(1, 1);
	float R12 = R(1, 2);
	float R20 = R(2, 0);
	float R21 = R(2, 1);
	float R22 = R(2, 2);

	float T0 = T(0, 0);
	float T1 = T(1, 0);
	float T2 = T(2, 0);

	for (int i = 0; i < pointNum; i++) {
		outputPoints(i, 0) = R00 * inputPoints(i, 0) + R01 * inputPoints(i, 1)
				+ R02 * inputPoints(i, 2) + T0;
		outputPoints(i, 1) = R10 * inputPoints(i, 0) + R11 * inputPoints(i, 1)
				+ R12 * inputPoints(i, 2) + T1;
		outputPoints(i, 2) = R20 * inputPoints(i, 0) + R21 * inputPoints(i, 1)
				+ R22 * inputPoints(i, 2) + T2;
	}

	/*
	 // At this moment, Eigen is much slower than above regular computation.
	 Eigen::Matrix3f R_eigen;
	 R_eigen(0,0) = R(0,0); R_eigen(0,1) = R(0,1); R_eigen(0,2) = R(0,2);
	 R_eigen(1,0) = R(1,0); R_eigen(1,1) = R(1,1); R_eigen(1,2) = R(1,2);
	 R_eigen(2,0) = R(2,0); R_eigen(2,1) = R(2,1); R_eigen(2,2) = R(2,2);

	 outputPoints = inputPoints * R_eigen.transpose();
	 outputPoints.col(0) += T(0,0)*Eigen::MatrixXf::Ones(outputPoints.rows(), 1);
	 outputPoints.col(1) += T(1,0)*Eigen::MatrixXf::Ones(outputPoints.rows(), 1);
	 outputPoints.col(2) += T(2,0)*Eigen::MatrixXf::Ones(outputPoints.rows(), 1);
	 */
}

void floor_calib::cam2floor(const float* inputPoints, float* outputPoints,
		int pointNum) {
	Mat_<float> R(R_cam2floor);
	Mat_<float> T(T_cam2floor);

	float R00 = R(0, 0);
	float R01 = R(0, 1);
	float R02 = R(0, 2);
	float R10 = R(1, 0);
	float R11 = R(1, 1);
	float R12 = R(1, 2);
	float R20 = R(2, 0);
	float R21 = R(2, 1);
	float R22 = R(2, 2);

	float T0 = T(0, 0);
	float T1 = T(1, 0);
	float T2 = T(2, 0);

	float* op = outputPoints;
	float* ip = (float*) inputPoints;
	for (int i = 0; i < pointNum; i++) {
		op[0] = R00 * ip[0] + R01 * ip[1] + R02 * ip[2] + T0;
		op[1] = R10 * ip[0] + R11 * ip[1] + R12 * ip[2] + T1;
		op[2] = R20 * ip[0] + R21 * ip[1] + R22 * ip[2] + T2;

		op += 3;
		ip += 3;
	}
}

void floor_calib::floor2cam(const Point3D& inputPoint, Point3D& outputPoint) {
	Mat_<float> R(R_floor2cam);
	Mat_<float> T(T_floor2cam);

	outputPoint.x = R(0, 0) * inputPoint.x + R(0, 1) * inputPoint.y
			+ R(0, 2) * inputPoint.z + T(0, 0);
	outputPoint.y = R(1, 0) * inputPoint.x + R(1, 1) * inputPoint.y
			+ R(1, 2) * inputPoint.z + T(1, 0);
	outputPoint.z = R(2, 0) * inputPoint.x + R(2, 1) * inputPoint.y
			+ R(2, 2) * inputPoint.z + T(2, 0);
}

void floor_calib::floor2cam(const vector<Point3D>& inputPoints,
		vector<Point3D>& outputPoints) {
	Mat_<float> R(R_floor2cam);
	Mat_<float> T(T_floor2cam);

	int pointNum = inputPoints.size();
	if (pointNum != outputPoints.size()) {
		printf(
				"Error in floor_calib::cam2plane: inputPoints and outputPoints must be the same size!\n");
		return;
	}
	for (int i = 0; i < pointNum; i++) {
		Point3D input = inputPoints[i];
		outputPoints[i].x = R(0, 0) * input.x + R(0, 1) * input.y
				+ R(0, 2) * input.z + T(0, 0);
		outputPoints[i].y = R(1, 0) * input.x + R(1, 1) * input.y
				+ R(1, 2) * input.z + T(1, 0);
		outputPoints[i].z = R(2, 0) * input.x + R(2, 1) * input.y
				+ R(2, 2) * input.z + T(2, 0);
	}
}

void floor_calib::floor2cam(const Eigen::Vector3f& inputPoint,
		Eigen::Vector3f& outputPoint) {
	Mat_<float> R(R_floor2cam);
	Mat_<float> T(T_floor2cam);

	outputPoint(0) = R(0, 0) * inputPoint(0) + R(0, 1) * inputPoint(1)
			+ R(0, 2) * inputPoint(2) + T(0, 0);
	outputPoint(1) = R(1, 0) * inputPoint(0) + R(1, 1) * inputPoint(1)
			+ R(1, 2) * inputPoint(2) + T(1, 0);
	outputPoint(2) = R(2, 0) * inputPoint(0) + R(2, 1) * inputPoint(1)
			+ R(2, 2) * inputPoint(2) + T(2, 0);
}

void floor_calib::floor2cam(const vector<Eigen::Vector3f>& inputPoints,
		vector<Eigen::Vector3f>& outputPoints) {
	outputPoints.resize(inputPoints.size());
	Mat_<float> R(R_floor2cam);
	Mat_<float> T(T_floor2cam);

	int pointNum = inputPoints.size();
	if (pointNum != outputPoints.size()) {
		printf(
				"Error in floor_calib::cam2plane: inputPoints and outputPoints must be the same size!\n");
		return;
	}
	for (int i = 0; i < pointNum; i++) {
		Eigen::Vector3f input = inputPoints[i];
		outputPoints[i][0] = R(0, 0) * input[0] + R(0, 1) * input[1]
				+ R(0, 2) * input[2] + T(0, 0);
		outputPoints[i][1] = R(1, 0) * input[0] + R(1, 1) * input[1]
				+ R(1, 2) * input[2] + T(1, 0);
		outputPoints[i][2] = R(2, 0) * input[0] + R(2, 1) * input[1]
				+ R(2, 2) * input[2] + T(2, 0);
	}

}

void floor_calib::clear(void) {
	inputPoints.clear();
	available = false;
}