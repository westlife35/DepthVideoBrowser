//all the include files are put in include file

//#include <stdio.h>
//#include <math.h>
//#include <Eigen/Core>
//#include <Eigen/Dense>
////I comment one row below
////#include "util/configfile.h"
//#include "Tracker.h"
//#include "planview.h"
//#include "colormap.hpp"
////I comment one row below
////#include "Device.h"

////I add one include file
//#include "LogicLevel/prime_sensor_frame_get.h"

//using namespace cv;

#include "back_projection.h"

//////////////////////////////////////////////////////////////
// this namespace contains functions to back-project 3D detection results
// to 2D image plane for visualization propuse.
//////////////////////////////////////////////////////////////

namespace BACK_PROJECTION {

float LabelColors[][3] = { { 0, 1, 1 }, { 0, 0, 1 }, { 0, 1, 0 }, { 1, 1, 0 }, {
		1, 0, 0 }, { 1, .5, 0 }, { .5, 1, 0 }, { 0, .5, 1 }, { .5, 0, 1 }, { 1,
		1, .5 }, { 1, 1, 1 } };
unsigned int nLabelColors = 10;

// visualization maps
Size backprojection_resolution;
Mat RGB_Vis, Depth_Vis;

// object detection
vector<Point3f> head_pos_in_cam;
vector<Point2f> head_pos_in_img;

void init(void) {
	backprojection_resolution.width = 640;
	backprojection_resolution.height = 480;
	RGB_Vis.create(backprojection_resolution, CV_8UC3);
	Depth_Vis.create(backprojection_resolution, CV_8UC3);
}

void update(void) {
	//openni::VideoFrameRef& pDepth = getDepthFrame();
	//openni::VideoFrameRef& pColor = getColorFrame();
	openni::VideoFrameRef& pDepth = dg_controls::sensor_->GetDepthFrame();
	openni::VideoFrameRef& pColor = dg_controls::sensor_->GetColorFrame();

	// copy the RGB and Depth data
	memcpy(RGB_Vis.data, pColor.getData(), 3 * 640 * 480);

	// convert the head position from world to camera coords
	if (!PROCESSOR::floorCalibration.isAvailable())
		return;
	

	int objectNumber = PROCESSOR::head_tracker.getPeopleNum();
	for (int i = 0; i < objectNumber; i++) {
		//------------------------------------------------------
		// get a valid tracked head
		objectLocation3D curHead;
		Eigen::Vector3f posInWorld;
		Eigen::Vector3f posInCam;

		PROCESSOR::head_tracker.getPeople(i, curHead);

		if (!curHead.isCurrentlyDetected() || !curHead.isMature())
			continue;

		int id = curHead.getID();
		posInWorld = curHead.getPosition();
		PROCESSOR::floorCalibration.floor2cam(posInWorld, posInCam);

		// convert from 3D to image plane
		Point3D pt3d_cam;
		Point2Di pt2d_img;
		unsigned short depth_value;
		pt3d_cam.x = posInCam(0);
		pt3d_cam.y = posInCam(1);
		pt3d_cam.z = posInCam(2);
		//get2Dfrom3D(pt3d_cam, pt2d_img, depth_value);
		//dg_controls::ONICapture::Get2Dfrom3D(pt3d_cam, pt2d_img, depth_value);
		Get2Dfrom3D(dg_controls::sensor_,pt3d_cam, pt2d_img, depth_value);

		// draw a circle around head;
		RotatedRect headBox;
		headBox.center.x = pt2d_img.x;
		headBox.center.y = pt2d_img.y;
		headBox.size.width = 100000.0f/depth_value;
		headBox.size.height = 50000.0f/depth_value;
		Scalar color(LabelColors[id % nLabelColors][0] * 255,
				LabelColors[id % nLabelColors][1] * 255,
				LabelColors[id % nLabelColors][2] * 255);
		ellipse(RGB_Vis, headBox, color, 3, CV_AA);

		//------------------------------------------------------
		// display trajectory of each person's head
		vector<Eigen::Vector3f> trajectory_points_in_world;
		vector<Eigen::Vector3f> trajectory_points_in_cam;
		curHead.getTrajectoryPoints(trajectory_points_in_world);
		PROCESSOR::floorCalibration.floor2cam(trajectory_points_in_world,
				trajectory_points_in_cam);

		for (int pointCnt = 0; pointCnt < trajectory_points_in_cam.size()-1;
				pointCnt++) {
			Point3D p3d_1, p3d_2;
			Point2Di p2d_1, p2d_2;

			// convert each trajectory point to image plane
			p3d_1.x = trajectory_points_in_cam[pointCnt][0];
			p3d_1.y = trajectory_points_in_cam[pointCnt][1];
			p3d_1.z = trajectory_points_in_cam[pointCnt][2];
			//get2Dfrom3D(p3d_1, p2d_1, depth_value);
			//dg_controls::ONICapture::Get2Dfrom3D(p3d_1, p2d_1, depth_value);
			Get2Dfrom3D(dg_controls::sensor_,p3d_1, p2d_1, depth_value);
			p3d_2.x = trajectory_points_in_cam[pointCnt+1][0];
			p3d_2.y = trajectory_points_in_cam[pointCnt+1][1];
			p3d_2.z = trajectory_points_in_cam[pointCnt+1][2];
			//get2Dfrom3D(p3d_2, p2d_2, depth_value);
			//dg_controls::ONICapture::Get2Dfrom3D(p3d_2, p2d_2, depth_value);
			Get2Dfrom3D(dg_controls::sensor_,p3d_2, p2d_2, depth_value);


			Point p1, p2;
			p1.x = p2d_1.x; p1.y = p2d_1.y;
			p2.x = p2d_2.x; p2.y = p2d_2.y;
			line(RGB_Vis, p1, p2, color, 1);
		}

	}
}

void release(void) {
	RGB_Vis.release();
	Depth_Vis.release();
}

//this function used in the file
void Get2Dfrom3D(DepthSensor *sensor_,Point3D p3d, Point2Di& p2d, unsigned short& depth_value)
{
	int x, y;
	unsigned short d;
	//openni::CoordinateConverter::convertWorldToDepth(g_depthStream, p3d.x, p3d.y, p3d.z, &x, &y,&d);
	openni::CoordinateConverter::convertWorldToDepth(sensor_->GetDepthStream(), p3d.x, p3d.y, p3d.z, &x, &y,&d);

	depth_value = d;
	p2d.x = x;
	p2d.y = y;

}

}
