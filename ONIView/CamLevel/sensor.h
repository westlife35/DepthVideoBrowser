/*
 * sensor.h
 *
 *  Created on: Mar 5, 2015
 *      Author: chenzhen
 */

#ifndef SRC_SENSOR_SENSOR_H_
#define SRC_SENSOR_SENSOR_H_

#include <iostream>
#include <map>

namespace dg_devices {

using namespace std;

/**
 * This class defines the abstract aspects of sensors. Please inherit this class
 * to define a specific sensor ex. Primescene. You need to implements all the
 * virtual methods to make your sensor class work.
 */
class Sensor {
public:

	Sensor() :
			name_(NULL), URI_(NULL), is_init_(false),
			is_start_(false),is_recording_(false), data_(NULL) {
	}

	virtual ~Sensor(){

	}
	void ** GetData() {
		return data_;
	}

	const char* GetSensorURI() {
		return URI_;
	}
	const char* GetSensorName() {
		return name_;
	}
	bool IsInit() {
		return is_init_;
	}
	bool IsStart() {
		return is_start_;
	}

	bool IsRecording() {
		return is_recording_;
	}

	// All the virtual methods you need to implements in your derived class
	virtual bool Init() = 0;
	virtual bool Start() = 0;
	virtual void Update() = 0;
	virtual bool StartRecording(const char* path, bool compress = false) = 0;
	virtual bool StopRecording() = 0;
	virtual bool Stop() = 0;
	virtual bool Release() = 0;
	virtual bool GetSensorProperties(map<string, float> &properties) = 0;

	//20150604 10:28 add some function
	//virtual void SetDisMode(int nIndex)=0;

protected:
	const char *name_;
	const char *URI_;
	bool is_init_;
	bool is_start_;
	bool is_recording_;
	void **data_;
};

/*
 * This class is a simple manager to provide some easy-to-use methods to manage
 * your sensors. In the future, this class will handle all the sensor related stuff
 * to make your life much happier ;)
 *
 * This class is a singleton, so please don't try to instantiate it.
 */
class SensorManager {
public:

	static SensorManager* Instance();
	~SensorManager();
	bool RegisterSensor(string name, Sensor* sensor);
	void RemoveSensor(string name);
	Sensor* FindSensor(string name);
	bool Start();
	void Update(void);
	void Update(double interval);
	bool Stop();

private:
	SensorManager();
	static SensorManager *instance;
	map<string, Sensor *> sensors_;
};

}

#endif /* SRC_SENSOR_SENSOR_H_ */
