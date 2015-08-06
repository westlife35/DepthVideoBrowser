/*
 * sensor.cpp
 *
 *  Created on: Mar 5, 2015
 *      Author: chenzhen
 */
#include "sensor.h"
#include "glog/logging.h"

namespace dg_devices {

SensorManager *SensorManager::instance = NULL;

SensorManager::SensorManager(){

}
SensorManager* SensorManager::Instance() {
	if (instance == NULL)
		instance = new SensorManager();
	return instance;
}

SensorManager::~SensorManager() {
	
}

bool SensorManager::RegisterSensor(string name, Sensor *sensor) {
	Sensor *s = FindSensor(name);
	if (s != NULL) {
		LOG(WARNING)<< "Sensor " << name << " already exists, update." << endl;
	}

	sensors_.insert(pair<string, Sensor*>(name, sensor));
	if (!sensor->IsInit())
	{
		if(!sensor->Init())
		  return false;
	}
	if (!sensor->IsStart())
		sensor->Start();
	return true;
}

Sensor* SensorManager::FindSensor(string name) {
	map<string, Sensor*>::iterator itr = sensors_.find(name);
	if (itr == sensors_.end())
		return NULL;
	return itr->second;
}

void SensorManager::RemoveSensor(string name) {
	Sensor *sensor = FindSensor(name);

	if (sensor == NULL) {
		LOG(WARNING)<< "Sensor " << name << "not registered, did not remove any sensor" << endl;
		return;
	}
	sensor->Stop();
	sensor->Release();
	sensors_.erase(name);
	if (sensor)
		delete sensor;
}

bool SensorManager::Start() {
	map<string, Sensor*>::iterator itr = sensors_.begin();
	while (itr != sensors_.end()) {
		Sensor *sensor = itr->second;
		if (!sensor->IsInit() && !sensor->Init()) {
			LOG(ERROR)<< "Sensor " << itr->first << " init failed" << endl;
			return false;
		}

		if (!sensor->IsStart() && !sensor->Start()) {
			LOG(ERROR)<< "Sensor " << itr->first << " start failed" << endl;
			return false;
		}
		itr++;
	}
	return true;
}

void SensorManager::Update() {
	Update(0);
}

//TODO the interval feature
void SensorManager::Update(double interval) {
	map<string, Sensor*>::iterator itr = sensors_.begin();
	while (itr != sensors_.end()) {
		itr->second->Update();
		itr++;
	}
}

bool SensorManager::Stop() {
	map<string, Sensor*>::iterator itr = sensors_.begin();
	while (itr != sensors_.end()) {

		if (!itr->second->Stop()) {
			LOG(WARNING)<< "Sensor " << itr->first << " stop failed" << endl;
		}
		itr ++;
	}
	return true;
}

}

