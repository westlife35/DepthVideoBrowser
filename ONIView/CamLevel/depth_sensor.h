/*
 * depth_sensor.h
 *
 *  Created on: Mar 18, 2015
 *      Author: chenzhen
 */

#ifndef SRC_DEPTH_SENSOR_DEPTH_SENSOR_H_
#define SRC_DEPTH_SENSOR_DEPTH_SENSOR_H_


#include "openni/include/openni/OpenNI.h"
#include "sensor.h"
#include <map>
using namespace openni;

namespace dg_devices {

using namespace std;
class DepthSensor: public Sensor {
public:

	DepthSensor() :
			is_color_enabled_(true), depth_width_(0), depth_height_(0), color_width_(
					0), color_height_(0) {
	}

	virtual ~DepthSensor() {

	}

	bool IsColorEnabled() {
		return is_color_enabled_;
	}
	int GetDepthWidth() {
		return depth_width_;
	}

	int GetDepthHeight() {
		return depth_height_;
	}

	int GetColorWidth() {
		return color_width_;
	}

	int GetColorHeight() {
		return color_height_;
	}

	unsigned char *GetColorData() {
		return (unsigned char *) GetData()[0];
	}

	unsigned short *GetDepthData() {
		return (unsigned short *) GetData()[1];
	}

	// the new methods that all the depth sensor need to implements
	virtual bool ConvertDepthToWorld(float depthX, float depthY, float depthZ,
			float* pWorldX, float* pWorldY, float* pWorldZ) = 0;

	//20150604 10:28 add some function
	virtual void SetDisMode(int nIndex)=0;
	//20150604 15:40 add some video file control
	virtual void* GetPlaybackControl()=0;
	virtual int GetNumberOfDepthFrames ()=0;
	virtual int GetNumberOfColorFrames ()=0;
	virtual bool GetRepeatEnabled()=0;
	virtual float GetSpeed()=0;
	virtual bool IsValid()=0;
	virtual int SeekDepthStream (int frameIndex)=0;
	virtual int SeekColorStream (int frameIndex)=0;
	virtual int SetRepeatEnabled (bool repeat)=0;
	virtual int SetSpeed (float speed)=0;
	virtual VideoFrameRef& GetDepthFrame()=0;
	virtual VideoFrameRef& GetColorFrame()=0;
	virtual VideoStream& GetDepthStream()=0;
	virtual VideoStream& GetColorStream()=0;
protected:

	bool is_color_enabled_;
	int depth_width_;
	int depth_height_;
	int color_width_;
	int color_height_;
};

}

#endif /* SRC_DEPTH_SENSOR_DEPTH_SENSOR_H_ */
