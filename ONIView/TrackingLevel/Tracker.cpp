#include <Eigen/Core>
#include <Eigen/Dense>
#include "Tracker.h"
#include "planview.h"
// #include "depth_MoG_background.h"
// #include "floor_calib.h"
// #include "location_tracker.h"
// #include "back_projection.h"
//#include "Device.h"

//add by myself
#include "LogicLevel/prime_sensor_frame_get.h"

namespace PROCESSOR {

/////////////////////////////////////////////////
// processing blocks
depth_MoG_background* bgmodel;

// variables for floor calibration
floor_calib floorCalibration;

// Foreground points in world
Eigen::MatrixXf FGPointsInCam;
Eigen::MatrixXf FGPointsInWorld;
int validFGPointsNumber;

// location based object tracker;
Location3DTracker head_tracker;

 // transform frontground scene points to floor coords
void computeForgroundPointsInWorld(void) {
	bool* fg_mask = bgmodel->getForeground();
	// if a background model is not available, treat all points as foreground

	//float *pPointCam3D = getPointCam3D();g_pointCam3D
	float *pPointCam3D = dg_controls::g_pointCam3D;

	if (!bgmodel->modelAvailable()) {
		validFGPointsNumber = bgmodel->idx.N;
		for (int i = 0; i < bgmodel->idx.N; i++) {
			FGPointsInCam(i, 0) = pPointCam3D[i*3 + 0];
			FGPointsInCam(i, 1) = pPointCam3D[i*3 + 1];
			FGPointsInCam(i, 2) = pPointCam3D[i*3 + 2];
		}
	} else {
		validFGPointsNumber = 0;
		for (int i = 0; i < bgmodel->idx.N; i++) {
			if (fg_mask[i]) {
				FGPointsInCam(validFGPointsNumber, 0) =
						pPointCam3D[i*3 + 0];
				FGPointsInCam(validFGPointsNumber, 1) =
						pPointCam3D[i*3 + 1];
				FGPointsInCam(validFGPointsNumber, 2) =
						pPointCam3D[i*3 + 2];

				validFGPointsNumber++;
			}
		}
	}

	//printf("validFGPointsNumber is %d.\n", validFGPointsNumber);

	floorCalibration.cam2floor(FGPointsInCam, FGPointsInWorld,
			validFGPointsNumber);

}


//////////////////////////////////////////////////
// initialization
void init(void) {
	printf("process, init...\n");

	//const string preload_path = "/home/qc/Downloads/Tools/demoroom";
	const string preload_path = "./Data";

	//---------------------------------------
	// background model initialization
	bgmodel = new depth_MoG_background(480, 640);

	// load a background model from file
	const string bgmodel_filename = "MoG_bgmodel.dat";
	char bgmodel_fullpath[1024];
	sprintf(bgmodel_fullpath, "%s\/%s", preload_path.c_str(),
			bgmodel_filename.c_str());
	printf("background model path: %s.\n", bgmodel_fullpath);
	if (bgmodel_filename.length() > 1) {
		bgmodel->load(bgmodel_fullpath);
	}

	//----------------------------------------
	// load floor calibration
	char floor_calib_path[1024];
	sprintf(floor_calib_path, "%s\/floor_calib.dat", preload_path.c_str());
	floorCalibration.load(floor_calib_path);

	// init Foreground point buffer
	FGPointsInCam = Eigen::MatrixXf::Zero(640*480, 3);
	FGPointsInWorld = Eigen::MatrixXf::Zero(640*480, 3);
	printf("FGPointsInCam: [%d, %d]; FGPointsInWorld: [%d, %d].\n",
			FGPointsInCam.rows(), FGPointsInCam.cols(), FGPointsInWorld.rows(),
			FGPointsInWorld.cols());

	// init floor_map generation
	PLANVIEW::init();

	// init the back-projection visualization
	BACK_PROJECTION::init();

	// reInit the location based object tracker;
	head_tracker.reInit(dg_controls::sensor_->GetNumberOfColorFrames());
}

///////////////////////////////////////////////////
// update at each frame
void update(unsigned long long time_stamp, int fid) {
	openni::VideoFrameRef& pDepth = dg_controls::sensor_->GetDepthFrame();
	openni::VideoFrameRef& pColor = dg_controls::sensor_->GetColorFrame();

	// ------------------------------------------------------------
	// update background, or compute foreground
	bgmodel->update((unsigned short *) pDepth.getData(),
			pDepth.getWidth(), pDepth.getHeight(),
			(unsigned char *) pColor.getData(),
			pColor.getWidth(), pColor.getHeight());

	if (floorCalibration.isAvailable()) {
		// ------------------------------------------------------------
		// transform frontground scene points to floor coords
		computeForgroundPointsInWorld();

		// ------------------------------------------------------------
		// generate floor map
		PLANVIEW::update(validFGPointsNumber, FGPointsInCam, FGPointsInWorld);

		// ------------------------------------------------------------
		// head tracker update
		vector<objectLocation3D> newDetections;
		for (int i = 0; i < PLANVIEW::detectedObjects.size(); i++)
			newDetections.push_back(
					objectLocation3D(
							PLANVIEW::detectedObjects[i].headTopInWorld));
		//if we want change the ID, we should change the ID of detected ones too. Because

		head_tracker.update(newDetections, time_stamp, fid);
	}

	// update back-projection visualization
	BACK_PROJECTION::update();
}

///////////////////////////////////////////////////
// release PROCESSOR
void release(void) {

	bgmodel->clear();
	floorCalibration.clear();

	FGPointsInCam.resize(0, 0);
	FGPointsInWorld.resize(0, 0);

	PLANVIEW::release();

	BACK_PROJECTION::release();
}

}
