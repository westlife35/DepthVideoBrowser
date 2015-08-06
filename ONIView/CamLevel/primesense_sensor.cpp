/*
 * Primesense.cpp
 *
 *  Created on: 2014年11月6日
 *      Author: Leo
 */

#include <math.h>
#include "glog/logging.h"

#include "primesense_sensor.h"

//add tracking
//#include "TrackingLevel/Tracker.h"

namespace dg_devices {

Primesense::Primesense(const char *URI, const char* name) :
		is_file_(false), is_properties_ready_(false), streams_(
		NULL), timestamp_(0), rc_(STATUS_OK) {
	URI_ = URI;
	name_ = name;
	data_ = new void*[2];
	recorder_ = new Recorder();
}

Primesense::~Primesense() {
	delete[] data_;
	delete recorder_;
}

bool Primesense::Init() {
	rc_ = OpenNI::initialize();
	LOG(INFO)<< "After initialization: " << OpenNI::getExtendedError() << endl;

	if (URI_ == NULL) {
		rc_ = device_.open(ANY_DEVICE);
		is_file_ = false;
	} else {
		rc_ = device_.open(URI_);
		is_file_ = true;
	}
	if (rc_ != STATUS_OK) {
	    //not use the fatal, it will make the progress crashed
		LOG(WARNING)<< "Primesense::Init_Device: Device open failed: " << OpenNI::getExtendedError() << endl;
		OpenNI::shutdown();
		return false;
	}

	DeviceInfo deviceInfo;
	deviceInfo = device_.getDeviceInfo();
	char buf[1024];
	sprintf(buf, "\n  name: %s\n"
			"  vendor: %s\n"
			"  vendor ID: %d\n"
			"  Uri: %s\n"
			"  USB Product ID: %d \n", deviceInfo.getName(),
			deviceInfo.getVendor(), deviceInfo.getUsbVendorId(),
			deviceInfo.getUri(), deviceInfo.getUsbProductId());
	LOG(INFO)<< "Primesense Info: " << buf << endl;

	rc_ = depth_stream_.create(device_, SENSOR_DEPTH);
	if (rc_ != STATUS_OK) {
		LOG(WARNING)<< "Primesense::Init_Device:: Couldn't find depth stream: " << OpenNI::getExtendedError() << endl;
	}

	rc_ = color_stream_.create(device_, SENSOR_COLOR);
	if (rc_ != STATUS_OK) {
		LOG(WARNING)<< "Primesense::Init_Device:: Couldn't find color stream: " << OpenNI::getExtendedError() << endl;
	}

	if (!depth_stream_.isValid() || !color_stream_.isValid()) { // ||->&& ? one of the streams is valid is enough
		LOG(FATAL)<< "Primesense::Init_Device: No valid streams. Exiting" << endl;
		device_.close();
		OpenNI::shutdown();
		return false;
	}

	VideoMode depthVideoMode;
	VideoMode colorVideoMode;

	if (depth_stream_.isValid() && color_stream_.isValid()) {
		depthVideoMode = depth_stream_.getVideoMode();
		colorVideoMode = color_stream_.getVideoMode();

		depth_width_ = depthVideoMode.getResolutionX();
		depth_height_ = depthVideoMode.getResolutionY();
		color_width_ = colorVideoMode.getResolutionX();
		color_height_ = colorVideoMode.getResolutionY();


		if (depth_width_ != color_width_ || depth_height_ != color_height_) {
			LOG(ERROR)<< "Error - expect color and depth to be in same resolution: " <<
			"Depth: " << depth_width_ << "*" << depth_height_ <<
			"Color: " << color_width_ << "*" << color_height_ << endl;
			return false;
		}

		LOG(INFO)<< "Color resolution: " << color_width_ << "*" << color_height_ << endl;
		LOG(INFO)<< "Depth resolution: " << depth_width_ << "*" << depth_height_ << endl;

	} else if (depth_stream_.isValid()) {
		LOG(INFO) << "Depth stream is valid. " << endl;
		depthVideoMode = depth_stream_.getVideoMode();
		depth_width_ = depthVideoMode.getResolutionX();
		depth_height_ = depthVideoMode.getResolutionY();
	} else if (color_stream_.isValid()) {
		LOG(INFO) << "Color stream is valid." << endl;
		colorVideoMode = color_stream_.getVideoMode();
		color_width_ = colorVideoMode.getResolutionX();
		color_height_ = colorVideoMode.getResolutionY();
	} else {
		LOG(FATAL) << "Error - expects at least one of the streams to be valid..." << endl;
		return false;
	}

	streams_ = new VideoStream*[2];
	streams_[0] = &depth_stream_;
	streams_[1] = &color_stream_;

	LOG(INFO)<< "Primesense is successfully initialized." << endl;
	is_init_ = true;
	return true;
}

bool Primesense::Start() {

	rc_ = depth_stream_.start();
	if (rc_ != STATUS_OK) {
		LOG(FATAL)<< "Primesense::Start:: Couldn't start depth stream: " << OpenNI::getExtendedError() << endl;
		depth_stream_.destroy();
		return false;
	}

	rc_ = color_stream_.start();
	if (rc_ != STATUS_OK) {
		LOG(FATAL)<< "Primesense::Start: Couldn't start color stream: " << OpenNI::getExtendedError() << endl;
		color_stream_.destroy();
		return false;
	}

	if (device_.isImageRegistrationModeSupported(
			IMAGE_REGISTRATION_DEPTH_TO_COLOR)) {
		device_.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
		LOG(INFO)<< "Depth channel is registered to color channel. " << endl;
	} else {
		LOG(INFO)<< "Depth to color registration is not supported by this device." << endl;
	}

	LOG(INFO)<< "Primesense is started with both color and depth stream." << endl;
	is_start_ = true;
	return true;

}

void Primesense::Update() {
	int changedIndex;

	Status rc = OpenNI::waitForAnyStream(streams_, 2, &changedIndex);
	if (rc != openni::STATUS_OK) {
		printf("Wait failed\n");
		return;
	}

	//add tracking
//	PROCESSOR::head_tracker.no_update = false;

	depth_stream_.readFrame(&depth_frame_);
	color_stream_.readFrame(&color_frame_);
	if (rc != STATUS_OK) {
		LOG(WARNING)<< "Read frame failed: " << OpenNI::getExtendedError() << endl;
		return;
	}

	if (depth_stream_.getVideoMode().getPixelFormat() != PIXEL_FORMAT_DEPTH_1_MM
			&& depth_stream_.getVideoMode().getPixelFormat()
					!= PIXEL_FORMAT_DEPTH_100_UM) {
		LOG(WARNING)<< "Unexpected frame format" << endl;
		return;
	}
	timestamp_ = depth_frame_.getTimestamp();
	data_[0] = (unsigned char*) color_frame_.getData();
	data_[1] = (unsigned short*) depth_frame_.getData();

}

bool Primesense::Stop() {
	depth_frame_.release();
	color_frame_.release();
	depth_stream_.stop();
	color_stream_.stop();
	depth_stream_.destroy();
	color_stream_.destroy();
	device_.close();
	OpenNI::shutdown();
	is_start_ = false;
	is_init_ = false;
	LOG(INFO)<< "Primesense is successfully shut down." << endl;
	return true;
}

bool Primesense::Release() {
	if (streams_ != NULL)
		delete[] streams_;

	return true;
}

bool Primesense::initSensorProperties() {

	is_properties_ready_ = false;

	OniVideoMode videoMode;
	int size = sizeof(videoMode);

	openni::Status status = depth_stream_.getProperty(
			ONI_STREAM_PROPERTY_VIDEO_MODE, &videoMode, &size);

	if (status != openni::STATUS_OK) {
		LOG(ERROR)<<
		"ERROR: Get Sensor Property ONI_STREAM_PROPERTY_VIDEO_MODE failed" << endl;
		return false;
	}

	size = sizeof(float);

	float horizontalFov;
	float verticalFov;

	status = depth_stream_.getProperty(ONI_STREAM_PROPERTY_HORIZONTAL_FOV,
			&horizontalFov, &size);
	if (status != STATUS_OK) {
		LOG(ERROR)<<
		"ERROR: Get Sensor Property ONI_STREAM_PROPERTY_HORIZONTAL_FOV failed" << endl;
		return false;
	}

	status = depth_stream_.getProperty(ONI_STREAM_PROPERTY_VERTICAL_FOV,
			&verticalFov, &size);
	if (status != STATUS_OK) {
		LOG(ERROR)<<
		"ERROR: Get Sensor Property ONI_STREAM_PROPERTY_VERTICAL_FOV failed" << endl;
		return false;
	}

	properties_.xzFactor = tan(horizontalFov / 2) * 2;
	properties_.yzFactor = tan(verticalFov / 2) * 2;
	properties_.resolutionX = videoMode.resolutionX;
	properties_.resolutionY = videoMode.resolutionY;
	properties_.halfResX = properties_.resolutionX / 2;
	properties_.halfResY = properties_.resolutionY / 2;
	properties_.coeffX = properties_.resolutionX / properties_.xzFactor;
	properties_.coeffY = properties_.resolutionY / properties_.yzFactor;

	is_properties_ready_ = true;

	return true;
}

bool Primesense::ConvertDepthToWorld(float depthX, float depthY, float depthZ,
		float* pWorldX, float* pWorldY, float* pWorldZ) {

	if (!is_properties_ready_) {
		if (!initSensorProperties()) {
			CoordinateConverter::convertDepthToWorld(depth_stream_, depthX,
					depthX, depthX, pWorldX, pWorldX, pWorldX);
		}
	}

	float normalizedX = depthX / properties_.resolutionX - 0.5f;
	float normalizedY = 0.5f - depthY / properties_.resolutionY;

	*pWorldX = normalizedX * depthZ * properties_.xzFactor;
	*pWorldY = normalizedY * depthZ * properties_.yzFactor;
	*pWorldZ = depthZ;

	// the OpenNI function, it's much slower and more time cost.
	//	CoordinateConverter::convertDepthToWorld(depth_stream_, depthX, depthX,
	//			depthX, pWorldX, pWorldX, pWorldX);

	return true;
}

bool Primesense::StartRecording(const char* path, bool compress) {

	Status rc = STATUS_OK;
	if (IsRecording()) {
		LOG(INFO)<< "There is an ongoing recording, please stop it before start a new "
		"one." << endl;
		return false;
	} else {
		// create a recorder
		rc = recorder_->create(path);
		if (rc != STATUS_OK) {
			LOG(WARNING) << "Failed to create the ONI recorder." << endl;
			return false;
		}

		// attach recorder to both color and depth stream
		rc = recorder_->attach(color_stream_, compress);
		if (rc != STATUS_OK) {
			LOG(WARNING) << "Failed to attache recorder to color stream." << endl;
			return false;
		}
		rc = recorder_->attach(depth_stream_, compress);
		if (rc != STATUS_OK) {
			LOG(WARNING) << "Failed to attache recorder to depth stream." << endl;
			return false;
		}

		// start the recorder
		rc = recorder_->start();
		if (rc != STATUS_OK) {
			printf("Failed to start recorder.\n");
			return false;
		}
	}

	LOG(INFO)<< "Start recording to file: " << path;
	if (compress)
		LOG(INFO)<< "with lossy compression." << endl;
		else
		LOG(INFO) << "without lossy compression. Expect very large file size." << endl;

	is_recording_ = true;
	return true;
}

bool Primesense::StopRecording() {

	if (!IsRecording()) {
		LOG(WARNING)<<"There is NO ongoing recording." << endl;
		return false;
	} else {
		recorder_->stop();
		recorder_->destroy();
	}

	LOG(INFO)<< "Complete recording." << endl;
	is_recording_ = false;
	return true;
}

bool Primesense::GetSensorProperties(map<string, float> &prop) {

	if (!is_properties_ready_)
		if (!initSensorProperties())
			return false;

	prop.insert(
			map<string, float>::value_type("xzFactor", properties_.xzFactor));
	prop.insert(
			map<string, float>::value_type("yzFactor", properties_.yzFactor));
	prop.insert(
			map<string, float>::value_type("resolutionX",
					properties_.resolutionX));
	prop.insert(
			map<string, float>::value_type("resolutionY",
					properties_.resolutionY));
	prop.insert(
			map<string, float>::value_type("halfResX", properties_.halfResX));
	prop.insert(
			map<string, float>::value_type("halfResY", properties_.halfResY));
	prop.insert(map<string, float>::value_type("coeffX", properties_.coeffX));
	prop.insert(map<string, float>::value_type("coeffY", properties_.coeffY));

	return true;
}

void Primesense::SetDisMode(int nIndex)
{
  cout<<"Primesense::SetDisMode      ";
  switch (nIndex)
  {
          case '0':
                  //m_eViewState = DISPLAY_MODE_OVERLAY;
                  device_.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
                  break;
          case '1':
                  //m_eViewState = DISPLAY_MODE_DEPTH;
                  device_.setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF);
                  break;
          case '2':
                  //m_eViewState = DISPLAY_MODE_IMAGE;
                  device_.setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF);
                  break;
          case '3':
                  depth_stream_.setMirroringEnabled(!depth_stream_.getMirroringEnabled());
                  color_stream_.setMirroringEnabled(!color_stream_.getMirroringEnabled());
                  break;
   }
}

// 20150604 add
void* Primesense::GetPlaybackControl()
{
  playbackControl_=device_.getPlaybackControl();
  return device_.getPlaybackControl();
}

int Primesense::GetNumberOfDepthFrames ()
{
  return playbackControl_->getNumberOfFrames(depth_stream_);
}

int Primesense::GetNumberOfColorFrames ()
{
  return playbackControl_->getNumberOfFrames(color_stream_);
}

bool Primesense::GetRepeatEnabled()
{
  return playbackControl_->getRepeatEnabled();
}

float Primesense::GetSpeed()
{
  return playbackControl_->getSpeed();
}

bool Primesense::IsValid()
{
  return playbackControl_->isValid();
}

int Primesense::SeekDepthStream (int frameIndex)
{
  //return playbackControl_->seek(depth_stream_,frameIndex);

  openni::Status rc = playbackControl_->seek(depth_stream_,frameIndex);
  //add tracking
//  if( rc==openni::STATUS_OK )
//  {
//    //if (frameId > CurrentId)
//    if(0)
//      PROCESSOR::head_tracker.no_update = false;
//    else
//    {
//      PROCESSOR::head_tracker.no_update = true;
//      framepositions positions;
//      PROCESSOR::head_tracker.getPositions(frameIndex, positions);
//      PROCESSOR::head_tracker.restoreAllPeople(positions);
//    }
//  }
  return (int)rc;
}

int Primesense::SeekColorStream (int frameIndex)
{
  return playbackControl_->seek(color_stream_,frameIndex);
}


int Primesense::SetRepeatEnabled (bool repeat)
{
  return playbackControl_->setRepeatEnabled(repeat);
}

int Primesense::SetSpeed (float speed)
{
  return playbackControl_->setSpeed(speed);
}

VideoFrameRef& Primesense::GetDepthFrame()
{
  return depth_frame_;
}

VideoFrameRef& Primesense::GetColorFrame()
{
  return color_frame_;
}

VideoStream& Primesense::GetDepthStream()
{
  return depth_stream_;
}

VideoStream& Primesense::GetColorStream()
{
  return color_stream_;
}


}
