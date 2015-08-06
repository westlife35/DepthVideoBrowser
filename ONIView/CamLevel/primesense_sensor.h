/*
 * primesensor.h
 *
 *  Created on: Mar 5, 2015
 *      Author: chenzhen
 */

#ifndef Primesense_H_
#define Primesense_H_

//#include "openni/include/openni/OpenNI.h" //put this include file to depth_sensor.h to make the "VideoFrameRef" in GetDepthFrame() and GetColorFrame() can be used
//using namespace openni;
#include <stdio.h>
#include "CamLevel/depth_sensor.h"

using namespace openni;

namespace dg_devices {

struct PrimesenseProperties {
	float xzFactor;
	float yzFactor;
	float coeffX;
	float coeffY;
	int resolutionX;
	int resolutionY;
	int halfResX;
	int halfResY;
};

class Primesense: public DepthSensor {
public:
	Primesense(const char *URI, const char* name = NULL);
	~Primesense();

	bool Init();
	bool Start();
	void Update();
	bool StartRecording(const char* path, bool compress = false);
	bool StopRecording();
	bool Stop();
	bool Release();
	bool GetSensorProperties(map<string, float> &properties);
	PlaybackControl* getPlaybackControl();

	// This member function convert depth point to world one. We use a stored
	// sensor cache to perform the transformation so it is much quicker than
	// the OpenNI function CoordinateConverter::convertDepthToWorld()
	bool ConvertDepthToWorld(float depthX, float depthY, float depthZ,
			float* pWorldX, float* pWorldY, float* pWorldZ);

	unsigned long long GetTimestamp() {
		return this->timestamp_;
	}

	//20150604 10:28 add some function
	void SetDisMode(int nIndex);
private :
	bool initSensorProperties();

private:

	bool is_file_;
	bool is_properties_ready_;
	Device device_;
	Recorder* recorder_;
	VideoStream depth_stream_;
	VideoStream color_stream_;
	VideoStream** streams_;
	VideoFrameRef depth_frame_;
	VideoFrameRef color_frame_;
	unsigned long long timestamp_;
	struct PrimesenseProperties properties_;

	//20150604 15:48 add
	PlaybackControl *playbackControl_;

	Status rc_;

	//20150604 15:48 add
public:
	void* GetPlaybackControl();
	int GetNumberOfDepthFrames ();
	int GetNumberOfColorFrames ();
	bool GetRepeatEnabled();
	float GetSpeed();
	bool IsValid();
	int SeekDepthStream (int frameIndex);
	int SeekColorStream (int frameIndex);
	int SetRepeatEnabled (bool repeat);
	int SetSpeed (float speed);
	VideoFrameRef& GetDepthFrame();
	VideoFrameRef& GetColorFrame();
	VideoStream& GetDepthStream();
	VideoStream& GetColorStream();

};
}
#endif /* Primesense_H_ */
