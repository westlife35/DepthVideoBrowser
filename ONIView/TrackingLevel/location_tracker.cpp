#include <stdio.h>
#include "location_tracker.h"

////debug cout
//#include <iostream>
//using namespace std;

//add for the PROCESSOR::floorCalibration.cam2floor(p3d_cam, p3d_world);
#include "TrackingLevel/Tracker.h"

//add for getting depth data
#include "LogicLevel/prime_sensor_frame_get.h"

objectLocation3D::objectLocation3D() {
	ID = -1;
	expirationTimeOut = OBJECT_TRACKING_EXPIRE_TIMER_MIN;
	compute_Gaussian_parameters();
	currentlyDetectionState = false;
	lifeTime = 0;
	velocity = Eigen::Vector3f::Zero();
}

objectLocation3D::objectLocation3D(int newID, Eigen::Vector3f position) {
	ID = newID;
	tracked_position = position;
	expirationTimeOut = OBJECT_TRACKING_EXPIRE_TIMER_MIN;
	compute_Gaussian_parameters();
	currentlyDetectionState = false;
	lifeTime = 0;
	velocity = Eigen::Vector3f::Zero();
	tracking_position_buffer.push_back(position);
	expected_position = position;
}

objectLocation3D::objectLocation3D(Eigen::Vector3f position) {
	ID = -1;
	tracked_position = position;
	expirationTimeOut = OBJECT_TRACKING_EXPIRE_TIMER_MIN;
	compute_Gaussian_parameters();
	currentlyDetectionState = false;
	lifeTime = 0;
	velocity = Eigen::Vector3f::Zero();
}

objectLocation3D::~objectLocation3D() {
	detection_position_buffer.clear();
}

Eigen::Vector3f objectLocation3D::getPosition() {
	return tracked_position;
}

void objectLocation3D::updateTracking(Eigen::Vector3f newPosition) {
	// update detection state and lifetime
	currentlyDetectionState = true;
	lifeTime++;

	// update expiration timeout
	if (expirationTimeOut < OBJECT_TRACKING_EXPIRE_TIMER_MAX)
		expirationTimeOut++;

	// update detection_position_buffer
	detection_position_buffer.insert(detection_position_buffer.begin(),
			newPosition);
	if (detection_position_buffer.size() > OBJECT_POSITION_BUFFER_SIZE)
		detection_position_buffer.pop_back();

	// smoothing
	apply_Gaussian_filter();

	// update tracking_position_buffer
	tracking_position_buffer.insert(tracking_position_buffer.begin(),
			tracked_position);
	if (tracking_position_buffer.size() > OBJECT_POSITION_BUFFER_SIZE)
		tracking_position_buffer.pop_back();

	// update velocity
	update_velocity();

	expected_position = tracking_position_buffer[0] + velocity;
}

//add for change tracking position by hand
void objectLocation3D::resetOneFrameTracking(Eigen::Vector3f newPosition)
{
  // update detection state and lifetime
  currentlyDetectionState = true;
  lifeTime++;

  // update expiration timeout
  if (expirationTimeOut < OBJECT_TRACKING_EXPIRE_TIMER_MAX)
          expirationTimeOut++;

  // update detection_position_buffer
  detection_position_buffer.insert(detection_position_buffer.begin(),
                  newPosition);
  if (detection_position_buffer.size() > OBJECT_POSITION_BUFFER_SIZE)
          detection_position_buffer.pop_back();

  // smoothing
  apply_Gaussian_filter();

  // update tracking_position_buffer
  tracking_position_buffer.insert(tracking_position_buffer.begin(),
                  tracked_position);
  if (tracking_position_buffer.size() > OBJECT_POSITION_BUFFER_SIZE)
          tracking_position_buffer.pop_back();

  // update velocity
  update_velocity();

  expected_position = tracking_position_buffer[0] + velocity;

//  // update detection state and lifetime
//  currentlyDetectionState = true;
//  //lifeTime++;

//  // update expiration timeout
//  if (expirationTimeOut < OBJECT_TRACKING_EXPIRE_TIMER_MAX)
//          expirationTimeOut++;

//  //add the pop
//  detection_position_buffer.pop_back();
//  // update detection_position_buffer
//  detection_position_buffer.insert(detection_position_buffer.begin(),
//                  newPosition);
//  if (detection_position_buffer.size() > OBJECT_POSITION_BUFFER_SIZE)
//          detection_position_buffer.pop_back();

//  // smoothing
//  apply_Gaussian_filter();

//  //add the pop
//  tracking_position_buffer.pop_back();
//  // update tracking_position_buffer
//  tracking_position_buffer.insert(tracking_position_buffer.begin(),
//                  tracked_position);
//  if (tracking_position_buffer.size() > OBJECT_POSITION_BUFFER_SIZE)
//          tracking_position_buffer.pop_back();

//  // update velocity
//  update_velocity();

//  expected_position = tracking_position_buffer[0] + velocity;
}

bool objectLocation3D::updateLostTracking() {
	expirationTimeOut--;
	currentlyDetectionState = false;
	lifeTime++;

	// insert a speicial value (0,0,0) when tracking is lost
	detection_position_buffer.insert(detection_position_buffer.begin(),
			Eigen::Vector3f(0.0f, 0.0f, 0.0f));
	if (detection_position_buffer.size() > OBJECT_POSITION_BUFFER_SIZE)
		detection_position_buffer.pop_back();

	// update tracking_position_buffer
	expected_position = tracking_position_buffer[0] + velocity;
	tracked_position = expected_position;
	tracking_position_buffer.insert(tracking_position_buffer.begin(),
			tracked_position);
//	printf("updateLostTracking: ID:%d, [%3.3f, %3.3f, %3.3f]\n",
//			this->ID, tracking_position_buffer[0].x(),
//			tracking_position_buffer[0].y(),
//			tracking_position_buffer[0].z());
	if (tracking_position_buffer.size() > OBJECT_POSITION_BUFFER_SIZE)
		tracking_position_buffer.pop_back();

	//update_velocity();

	if (expirationTimeOut <= 0)
		return false;
	else
		return true;
}

void objectLocation3D::update_velocity() {
	velocity << 0, 0, 0;

	if (tracking_position_buffer[0][0] == 0
			&& tracking_position_buffer[0][1] == 0
			&& tracking_position_buffer[0][2] == 0)
		return;

	if(tracking_position_buffer.size() < VELOCITY_TIME_DELTA)
		return;

	if (tracking_position_buffer[VELOCITY_TIME_DELTA][0] == 0
			&& tracking_position_buffer[VELOCITY_TIME_DELTA][1] == 0
			&& tracking_position_buffer[VELOCITY_TIME_DELTA][2] == 0)
		return;

	for(int i=0; i<3; i++) {
		velocity(i) = tracking_position_buffer[0](i) - tracking_position_buffer[VELOCITY_TIME_DELTA-1](i);
		velocity(i) /= (float)VELOCITY_TIME_DELTA;
	}
	velocity(2) = 0;
}

bool objectLocation3D::isCurrentlyDetected() {
	return currentlyDetectionState;
}

// return the distance between the current tracking position and the "other" position 
float objectLocation3D::distance(objectLocation3D other) {
	Eigen::Vector3f other_pos = other.getPosition();
	float dist = sqrt(
			(tracked_position.x() - other_pos.x())
					* (tracked_position.x() - other_pos.x())
					+ (tracked_position.y() - other_pos.y())
							* (tracked_position.y() - other_pos.y())
					+ (tracked_position.z() - other_pos.z())
							* (tracked_position.z() - other_pos.z()));
	return dist;
}

// return the distance between the current expected position and the "other" position 
float objectLocation3D::expected_distance(objectLocation3D other) {
	Eigen::Vector3f other_pos = other.getPosition();
	float dist = sqrt(
			(expected_position.x() - other_pos.x())	* (expected_position.x() - other_pos.x())
			+ (expected_position.y() - other_pos.y()) * (expected_position.y() - other_pos.y())
			+ (expected_position.z() - other_pos.z()) * (expected_position.z() - other_pos.z()));
	return dist;
}

void objectLocation3D::info() {
	printf("people ID: %d, pos: [%3.3f, %3.3f, %3.3f], velocity: [%3.3f, %3.3f, %3.3f]\n", ID,
			tracked_position.x(), tracked_position.y(), tracked_position.z(),
			velocity.x(), velocity.y(), velocity.z());
}

void objectLocation3D::compute_Gaussian_parameters() {
	float sigma = OBJECT_TRACKING_SMOOTHING_GAUSSIAN_SIGMA;
	float sum = 0.0f;
	for (int i = 0; i < OBJECT_TRACKING_SMOOTHING_FILTERSIZE; i++) {
		float x = (float) i;
		smoothing_filter_parameters[i] = exp(-(x * x) / (2 * sigma * sigma))
				/ (sqrt(2 * PAI) * sigma);
		sum += smoothing_filter_parameters[i];
	}
	// normalize
	for (int i = 0; i < OBJECT_TRACKING_SMOOTHING_FILTERSIZE; i++)
		smoothing_filter_parameters[i] /= sum;
	return;
}

void objectLocation3D::apply_Gaussian_filter() {
	int smoothing_length = min((int) OBJECT_TRACKING_SMOOTHING_FILTERSIZE,
			(int) detection_position_buffer.size());
	float total_smoothing_weight = 0.0f;
	tracked_position << 0.0f, 0.0f, 0.0f;
	for (int i = 0; i < smoothing_length; i++) {
		if (detection_position_buffer[i][0] == 0
				&& detection_position_buffer[i][1] == 0
				&& detection_position_buffer[i][2] == 0)
			continue;

		total_smoothing_weight += smoothing_filter_parameters[i];
		tracked_position[0] += smoothing_filter_parameters[i]
				* detection_position_buffer[i][0];
		tracked_position[1] += smoothing_filter_parameters[i]
				* detection_position_buffer[i][1];
		tracked_position[2] += smoothing_filter_parameters[i]
				* detection_position_buffer[i][2];
	}

	tracked_position /= total_smoothing_weight;

}

int objectLocation3D::getID() {
	return ID;
}

void objectLocation3D::setID(int nNewID)
{
  ID=nNewID;
}

int objectLocation3D::getAge() {
	return lifeTime;
}

bool objectLocation3D::isMature() {
	return lifeTime > TRACKING_MATRURITY_TH;
}

void objectLocation3D::getTrajectoryPoints(vector<Eigen::Vector3f>& output) {
	output.clear();

	for (int i = 0; i < tracking_position_buffer.size(); i++) {
		if (tracking_position_buffer[i][0] == 0
				&& tracking_position_buffer[i][1] == 0
				&& tracking_position_buffer[i][2] == 0)
			continue;
		output.push_back(tracking_position_buffer[i]);
	}
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

Location3DTracker::Location3DTracker() {
	newID = 0;
	all_people.clear();
	//the max size =1000 is a big limit to the lengh of #.oni
	all_trajectory.resize(5000);
	isRecorderOn = false;
}

void Location3DTracker::reInit(int nNumOfTotalFrame)
{
  if(nNumOfTotalFrame>500000)
  {
    cout<<"the total frame number of oni is bigger than 500000!"<<endl;
    exit(0);
  }
  if(nNumOfTotalFrame<0)
  {
      cout<<"the total frame number of oni is less than 0!"<<endl;
      exit(0);
  }
  newID = 0;
  all_people.clear();
  all_trajectory.resize(nNumOfTotalFrame);
  isRecorderOn = false;
}

Location3DTracker::~Location3DTracker() {
	all_people.clear();
}

int Location3DTracker::getPeopleNum() {
	return all_people.size();
}

bool Location3DTracker::getPeople(int i, objectLocation3D& output) {
	if (i >= all_people.size())
		return false;

	output = all_people[i];
	return true;
}

void Location3DTracker::getAllPeople(vector<objectLocation3D> &output)
{
  output=all_people;
}

bool Location3DTracker::getPositions(int frameId, framepositions& positions)
{
	if (frameId > all_trajectory.size())
		return false;

	positions = all_trajectory[frameId-1];
	return true;
}

//add for adjust the tracking pos by hand
void Location3DTracker::setPosition(int i,int x,int y)
{
  Point3D p3d_cam, p3d_world;
  //const openni::DepthPixel* pDepth = (openni::DepthPixel*) getDepthFrame().getData();
  const openni::DepthPixel* pDepth = (openni::DepthPixel*)( dg_controls::sensor_->GetDepthData() );
  //unsigned short depth_value = *(pDepth + y*640 + x);
  unsigned short depth_value = *(pDepth + y*dg_controls::sensor_->GetColorWidth() + x);
  dg_controls::ONICapture::Get3Dfrom2D(x, y, depth_value, p3d_cam);
  PROCESSOR::floorCalibration.cam2floor(p3d_cam, p3d_world);

  //objectLocation3D new_people = objectLocation3D(id, p3d_world);
  Eigen::Vector3f posInWorld;
  posInWorld(0)=p3d_world.x;
  posInWorld(1)=p3d_world.y;
  posInWorld(2)=p3d_world.z;

  //int debug=all_people[i].getID();
//  objectLocation3D new_people = objectLocation3D(all_people[i].getID(), posInWorld);
//  all_people[i]=new_people;

  all_people[i].resetOneFrameTracking(posInWorld);
}

//add for adjust the tracking ID by hand
void Location3DTracker::setID(int nID,int nIndex)
{
  all_people[nIndex].setID(nID);
}

void Location3DTracker::addPosition(int x,int y)
{
  Point3D p3d_cam, p3d_world;
  //const openni::DepthPixel* pDepth = (openni::DepthPixel*) getDepthFrame().getData();
  const openni::DepthPixel* pDepth = (openni::DepthPixel*)dg_controls::sensor_->GetDepthData();
  unsigned short depth_value = *(pDepth + y*dg_controls::sensor_->GetColorWidth() + x);
  dg_controls::ONICapture::Get3Dfrom2D(x, y, depth_value, p3d_cam);
  PROCESSOR::floorCalibration.cam2floor(p3d_cam, p3d_world);
  Eigen::Vector3f posInWorld;
  posInWorld(0)=p3d_world.x;
  posInWorld(1)=p3d_world.y;
  posInWorld(2)=p3d_world.z;

  //for adding the right ID, we choose the biggest ID from all_people
  int nBiggestID=-1;
  for(int i=0;i<all_people.size();i++)
  {
      if(nBiggestID<all_people[i].getID())
      {
          nBiggestID=all_people[i].getID();
      }
  }
  //in the occusion of non people, add the new person with ID 0
  if(nBiggestID==-1)
    nBiggestID=0;

  //objectLocation3D new_people = objectLocation3D(all_people[all_people.size()-1].getID() +1, posInWorld);
  objectLocation3D new_people = objectLocation3D(nBiggestID +1, posInWorld);
  all_people.push_back(new_people);
}



void Location3DTracker::restoreAllPeople(framepositions& position)
{
	all_people.clear();

	int num = position.object_num;
  	vector<objectPosition> &pos = position.positions;

  	for (int i = 0; i < num; ++i)
  	{
  		int id = pos[i].id;
  		Eigen::Vector3f p3d = pos[i].p3d;

  		objectLocation3D new_people = objectLocation3D(id, p3d);
		all_people.push_back(new_people);
  	}
}

bool Location3DTracker::getPeopleWithID(int id, objectLocation3D& output) {
	for (int i = 0; i < all_people.size(); i++) {
		if (id == all_people[i].getID()) {
			output = all_people[i];
			return true;
		}
	}

	return false;
}

bool Location3DTracker::deletePeopleWithID(int id)
{
	int i;
	for (i = 0; i < all_people.size(); i++) {
		if (id == all_people[i].getID()) {
				all_people.erase(all_people.begin() + i);
				return true;
			}
	}

	return false;
}

bool Location3DTracker::newPeopleWithID(int id)
{
	objectLocation3D new_people = objectLocation3D(id, p3d);
	all_people.push_back(new_people);
	printf("new id: %d --> (%f, %f, %f).\n", id, p3d(0), p3d(1), p3d(2));
}

void Location3DTracker::update(vector<objectLocation3D> detected_people,
		unsigned long long time_stamp, int fid) {

	cur_time = time_stamp;
	curCompletedTrackingResults.clear();

	vector<bool> existing_people_tracked;
	vector<bool> detected_people_tracked;

	existing_people_tracked.resize(all_people.size());
	detected_people_tracked.resize(detected_people.size());

	for (int i = 0; i < existing_people_tracked.size(); i++)
		existing_people_tracked[i] = false;
	for (int i = 0; i < detected_people_tracked.size(); i++)
		detected_people_tracked[i] = false;

	// compare detected people with all existing tracked people, find matching
	vector<float> pairwise_distance;
	pairwise_distance.resize(detected_people.size() * all_people.size());
	for (int detected_people_id = 0; detected_people_id < detected_people.size(); detected_people_id++) {
		for (int existing_people_id = 0; existing_people_id < all_people.size();
				existing_people_id++) {
			//pairwise_distance[detected_people_id * all_people.size() + existing_people_id] = detected_people[detected_people_id].distance(all_people[existing_people_id]);
			pairwise_distance[detected_people_id * all_people.size()
					+ existing_people_id] = all_people[existing_people_id].expected_distance(detected_people[detected_people_id]);
		}
	}


	// for each existing people, find their best match in detected_people
	for (int existing_people_id = 0; existing_people_id < all_people.size();
			existing_people_id++) {
		// find the current min
		float current_min = INTER_OBJECT_DISTANCE_MAX;
		int current_min_idx = -1;
		for (int idx = 0; idx < detected_people.size() * all_people.size();
				idx++) {
			if (current_min > pairwise_distance[idx]) {
				current_min_idx = idx;
				current_min = pairwise_distance[idx];
			}
		}

//		for (int idx = 0; idx < detected_people.size();
//				idx++) {
//			if (current_min > pairwise_distance[idx*all_people.size()+existing_people_id]) {
//				current_min_idx = idx*all_people.size()+existing_people_id;
//				current_min = pairwise_distance[idx*all_people.size()+existing_people_id];
//			}
//		}

		if (current_min < OBJECT_CONTINUITY_DISPLACEMENT_MAX) {
			int detected_people_id = current_min_idx / all_people.size();
			int existing_people_id = current_min_idx % all_people.size();
			// for matched existing people, update the position;
			all_people[existing_people_id].updateTracking(
					detected_people[detected_people_id].getPosition());
			// clear data in the table about this matched people
			for (int idx = 0; idx < detected_people.size(); idx++)
				pairwise_distance[idx * all_people.size() + existing_people_id] =
						INTER_OBJECT_DISTANCE_MAX;
			for (int idx = 0; idx < all_people.size(); idx++)
				pairwise_distance[detected_people_id * all_people.size() + idx] =
						INTER_OBJECT_DISTANCE_MAX;

			existing_people_tracked[existing_people_id] = true;
			detected_people_tracked[detected_people_id] = true;
		}
	}

	// for unmatched existing people, update their status too.
	for (int existing_people_id = 0; existing_people_id < all_people.size();
			existing_people_id++) {
		if (!existing_people_tracked[existing_people_id]) {
			// if tracking expires, remove this person
			if (!all_people[existing_people_id].updateLostTracking()) {

				// store the completed tracking into curCompletedTrackingResults
				objectTrajectory completed_tracking;
				completed_tracking.ID = all_people[existing_people_id].getID();
				all_people[existing_people_id].getTrajectoryPoints(completed_tracking.trajectory);

				if(completed_tracking.trajectory.size() >= VALID_TRACKING_TRAJECTORY_MIN)
				{
					curCompletedTrackingResults.push_back(completed_tracking);
					record_tracking_result_per_people(existing_people_id);
				}

				// remove this expired object
				all_people.erase(all_people.begin() + existing_people_id);
				existing_people_id--;
			}
		}
	}

	// add unmatched people as new people
	for (int detected_people_id = 0;
			detected_people_id < detected_people.size(); detected_people_id++) {
		if (!detected_people_tracked[detected_people_id]) {
			newID++;
			objectLocation3D new_people = objectLocation3D(newID,
					detected_people[detected_people_id].getPosition());

			//printf("new people ID: %d.\n", new_people.getID());
			all_people.push_back(new_people);
		}
	}

//	// print info
//	for(int all_people_cnt=0; all_people_cnt<all_people.size(); all_people_cnt++) {
//	  all_people[all_people_cnt].info();
//	}

	pairwise_distance.clear();

	// recorder
	record_tracking_result_per_frame();

	// updateLabeling(fid);
}

//////////////////////////////////////////////////////
// once recording is started, tracking result is recorded in two different ways
// 1) per frame: whenever there is an object in tracking, data will be recorder in the order of frame
// 2) per object: whenever the tracking of an object is completed, the whole trajectory will be stored.
void Location3DTracker::startRecording(char* per_frame_filename, char* per_people_filename) {
	// create an empty file
	recorderFilePerPeople = fopen(per_people_filename, "wt"); //FILE* input = stdin;
	if (recorderFilePerPeople == NULL) {
		fprintf(stderr,
				"Cannot open a tracking record file location_tracking_log.\n");
		return;
	}

	recorderFilePerFrame = fopen(per_frame_filename, "wt"); //FILE* input = stdin;
	if (recorderFilePerFrame == NULL) {
		fprintf(stderr,
				"Cannot open a tracking record file location_tracking_log.\n");
		return;
	}

	isRecorderOn = true;
	printf("Location tracking recording is started.\n");

}

void Location3DTracker::stopRecording(void) {
	isRecorderOn = false;
	fclose(recorderFilePerPeople);
	fclose(recorderFilePerFrame);
	printf("Location tracking recording is turned off.\n");
}

void Location3DTracker::record_tracking_result_per_people(int id) {
	// ignore recording if the recorder is off
	if (!isRecorderOn)
		return;

	// ignore recording if the ID is off the boundary.
	if (id < 0 || id >= all_people.size())
		return;

	vector<Eigen::Vector3f> trajectory;
	all_people[id].getTrajectoryPoints(trajectory);

	///////////////////////////////////
	// record data into file
	fprintf(recorderFilePerPeople, "************************************\n");

	// first is the object ID and recording time:
	fprintf(recorderFilePerPeople, "ID: %d\n", all_people[id].getID());
	fprintf(recorderFilePerPeople, "time: %ld\n", cur_time);

	// then all the trajectory data
	int trajectory_len = trajectory.size();
	fprintf(recorderFilePerPeople, "size: %d\n", trajectory_len);
	for(int i=0; i<trajectory_len; i++)
	{

		fprintf(recorderFilePerPeople, "%f %f %f\n",
				trajectory[i][0], trajectory[i][1], trajectory[i][2]);
	}
	fprintf(recorderFilePerPeople, "************************************\n");

}

void Location3DTracker::record_tracking_result_per_frame() {
	// ignore recording if the recorder is off
	if (!isRecorderOn)
		return;

	// ignore recording if there is nothing record
	if (all_people.size() == 0)
		return;

	///////////////////////////////////
	// record data into file
	fprintf(recorderFilePerFrame, "************************************\n");
	// first is the time stamp:
	fprintf(recorderFilePerFrame, "time: %ld\n", cur_time);

	// then all the object data
	fprintf(recorderFilePerFrame, "size: %d\n", all_people.size());
	for(int i=0; i<all_people.size(); i++)
	{
		Eigen::Vector3f pos = all_people[i].getPosition();
		fprintf(recorderFilePerFrame, "%d %f %f %f\n", all_people[i].getID(),
				pos(0), pos(1), pos(2));
	}
	fprintf(recorderFilePerFrame, "************************************\n");
}

void Location3DTracker::saveLabeling(char *save_file)
{
	fs = FileStorage(save_file, FileStorage::WRITE);

	int count = all_trajectory.size();

	vector<Point3f> point_vect;
	vector<int> id_vect;

	fs << "total_frame" << count;
	fs << "frame" << "[";
	for (int i = 0; i < count; ++i)
	{
		fs << "{:";
		fs <<"frame_id" << all_trajectory[i].frame_id;
		fs <<"object_num" << all_trajectory[i].object_num;

		point_vect.clear();
		id_vect.clear();
		for (int j = 0; j < all_trajectory[i].object_num; ++j)
		{
			Point3f p;
			p.x = all_trajectory[i].positions[j].p3d(0);
			p.y = all_trajectory[i].positions[j].p3d(1);
			p.z = all_trajectory[i].positions[j].p3d(2);
			point_vect.push_back(p);

			int id = all_trajectory[i].positions[j].id;
			id_vect.push_back(id);
		}

		fs <<"id" << id_vect;
		fs <<"position" << point_vect;

		fs<< "}";
	}
	fs << "]";
	printf("save xml to %s\n", save_file);

	fs.release();
}

void Location3DTracker::updateLabeling(int fid)
{
	framepositions &poss = all_trajectory[fid-1];
	poss.frame_id = fid;
	poss.object_num = all_people.size();
	poss.positions.clear();
	//there is a random crash
	//vector<objectPosition>().swap(poss.positions);
	//vector<objectPosition> t;
	//poss.positions.swap(t);
	for (int i = 0; i < all_people.size(); ++i)
	{
		objectPosition pos;
		pos.id = all_people[i].getID();
		pos.p3d = all_people[i].getPosition();
		poss.positions.push_back(pos);
	}
}

void Location3DTracker::setTrackingState(vector<objectLocation3D> all_peopleIn)
{
  all_people=all_peopleIn;
}

int Location3DTracker::getnewID()
{
  return newID;
}

void Location3DTracker::setnewID(int newIDtoSet)
{
  newID=newIDtoSet;
}

void Location3DTracker::deletePeopleByIndex(int nIndex)
{
  std::vector<objectLocation3D>::iterator it = all_people.begin()+nIndex;
  all_people.erase(it);
}
