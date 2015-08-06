/*************************************************************************
	> File Name: prime_sensor_frame_get.cpp
	> Author: 
	> Mail: 
	> Created Time: Mon 01 Jun 2015 11:30:57 AM CST
 ************************************************************************/
#include "LogicLevel/prime_sensor_frame_get.h"


namespace dg_controls {

//make the file depth_MoG_background.cpp can use the sensor_ to get the depthstream
DepthSensor *sensor_=NULL;
vector<RGBDFrame> rgbdFrame;

float g_pointCam3D[640*480*3];
unsigned char pColorBuf[640*480*3];

ONICapture::ONICapture()
{
  sensor_=NULL;
  bIsOpened=false;
  isFileOrCam=false;
  nLastFrameNo=0;
  bIsHaveUpdateOnce=false;
  bIsBackPlayOrForwardPlay=true;
}

//ONICapture::Open(string name, Sensor *sensor)
bool ONICapture::Open(std::string name, std::string path)
{
	SensorManager::Instance()->RegisterSensor(name,new Primesense(path.data(),name.data()) );
	SensorManager::Instance()->Start();
	bIsOpened=SensorManager::Instance()->Start();
	if(!bIsOpened)
	{
	    return false;
	}
	sensor_ =(DepthSensor *) SensorManager::Instance()->FindSensor(name);
	if(sensor_==NULL)
	{
	    return false;
	}
	sensorName_=name;
	colorFromDepthdata=new unsigned char[sensor_->GetDepthHeight()*sensor_->GetDepthWidth()*3];
	//frameStorage_.colorData=new unsigned char[sensor_->GetColorHeight()*sensor_->GetColorWidth()*3];
	isFileOrCam=true;
	GetPlaybackControl();

//	//tracking init
//	PROCESSOR::init();

//	//set the size
//	vecFrameNoAndall_people.resize(GetNumberOfDepthFrames());

	return true;
}

bool ONICapture::Open()
{
  bool bIsRegistered=SensorManager::Instance()->RegisterSensor( "NO.0_cam",new Primesense(NULL,"NO.0_cam") );
  if(!bIsRegistered)
  {
    return false;
  }
  bIsOpened=SensorManager::Instance()->Start();
  if(!bIsOpened)
  {
      return false;
  }
  sensor_ =(DepthSensor *) SensorManager::Instance()->FindSensor("NO.0_cam");
  if(sensor_==NULL)
  {
      return false;
  }
  sensorName_="NO.0_cam";
  colorFromDepthdata=new unsigned char[sensor_->GetDepthHeight()*sensor_->GetDepthWidth()*3];
  //frameStorage_.colorData=new unsigned char[sensor_->GetColorHeight()*sensor_->GetColorWidth()*3];
  isFileOrCam=false;

//  //tracking init
//  PROCESSOR::init();

//  //set the size
//  vecFrameNoAndall_people.resize(GetNumberOfDepthFrames());

  return true;
}

ONICapture::~ONICapture()
{
  if(bIsOpened)
  {
    delete colorFromDepthdata;
    //delete frameStorage_.colorData;
  }
}

void ONICapture::SetNewOniFlag()
{
  bIsHaveUpdateOnce=false;
}

void ONICapture::Update()
{

	if(bIsHaveUpdateOnce)
	{
	  nLastFrameNo=GetFrameIndex();
	}
	else
	{
	   bIsHaveUpdateOnce=true;
	}
	sensor_->Update();
	//int a=GetFrameIndex();
	//fix the bug of the number of tracked ID will bigger than before the binary is auto replay
	int debug=GetFrameIndex();
	if( GetFrameIndex() < nLastFrameNo-10 )//set a number minus
	{
	  // remove tracking
	  //PROCESSOR::release();
	  //PROCESSOR::init();
	}
	nLastFrameNo=GetFrameIndex();


//	RGBDFrame tempRgbdFrame;
//	tempRgbdFrame.setIndexAndSize(GetFrameIndex(),GetColorWidth(),GetColorHeight());
//	tempRgbdFrame.initMem();
//	tempRgbdFrame.setData( (unsigned short*)sensor_->GetData()[1], (unsigned char*)sensor_->GetData()[0], (unsigned char*)GetColorFromDepthData() );

	//rgbdFrame.push_back(tempRgbdFrame);
	//if(rgbdFrame.size()>1000)
	//  rgbdFrame.erase(rgbdFrame.begin());

	//DrawObjectTracker();
}

int ONICapture::GetDepthHeight()
{
	return sensor_->GetDepthHeight();
}

int ONICapture::GetDepthWidth()
{
	return sensor_->GetDepthWidth();
	
}

int ONICapture::GetColorHeight()
{
	return sensor_->GetColorHeight();
}

int ONICapture::GetColorWidth()
{
	return sensor_->GetColorWidth();
	
}

const char* ONICapture::GetSensorName() 
{
	return sensor_->GetSensorName();
}

const char* ONICapture::GetSensorURI() 
{
	return sensor_->GetSensorURI();
}



bool ONICapture::GetSensorProperties(map<string, float> &prop) 
{

	sensor_->GetSensorProperties(prop);

	return true;
}

bool ONICapture::StartRecording()
{
	if (sensor_->IsRecording()) 
        {
		//cout << "This sensor is recording now." << endl;
		return false;
	}
	else
	{
		sensor_->StartRecording("recordingdata.oni", false);
		return true;
	}
}

bool ONICapture::StopRecording()
{
	if (!sensor_->IsRecording()) 
	{
		cout << "This sensor is not recording now." << endl;
		return false;
	}
	else
	{
		sensor_->StopRecording();
		return true;
	}
}

bool ONICapture::ConvertDepthToWorld(float depthX, float depthY, float depthZ,
			float* pWorldX, float* pWorldY, float* pWorldZ)
{
	sensor_->ConvertDepthToWorld(depthX, depthY, depthZ, pWorldX, pWorldY,pWorldZ);
	return true;
}

void ** ONICapture::GetData()
{
	return sensor_->GetData();
}

bool ONICapture::Close()
{
	//return sensor_->Stop();
	SensorManager::Instance()->RemoveSensor(sensorName_);
//	// processor release
//	PROCESSOR::head_tracker.saveLabeling("ONIView_trajectory.xml");
//	PROCESSOR::release();
	return true;
}

void ONICapture::SetMode(int nIndex)
{
  //LOG(WARNING)<<"ONICapture::SetMode   "<< endl;
  sensor_->SetDisMode(nIndex);
}

void ONICapture::GetColorFromDepth(unsigned char* rgb)
{
  unsigned short *depthData=(unsigned short *)sensor_->GetData()[1];
  unsigned char* rgbTemp=rgb;
  unsigned short* depthTemp=depthData;
  for(int i=0;i<sensor_->GetDepthHeight()*sensor_->GetColorWidth();i++)
  {
      UInt16RGB2Jet(*depthTemp,1000,4000,rgbTemp);
      depthTemp++;
      rgbTemp += 3;
  }
}

void* ONICapture::GetColorData()
{
  return sensor_->GetData()[0];
}

void* ONICapture::GetColorFromDepthData()
{
  GetColorFromDepth(colorFromDepthdata);
  return (void*)colorFromDepthdata;
}

// 20150604 add

void ONICapture::GetPlaybackControl()
{
  sensor_->GetPlaybackControl();
}

int ONICapture::GetNumberOfDepthFrames ()
{
  return sensor_->GetNumberOfDepthFrames();
}

int ONICapture::GetNumberOfColorFrames ()
{
  return sensor_->GetNumberOfColorFrames();
}

bool ONICapture::GetRepeatEnabled()
{
  return sensor_->GetRepeatEnabled();
}

float ONICapture::GetSpeed()
{
  return sensor_->GetSpeed();
}

bool ONICapture::IsValid()
{
  return sensor_->IsValid();
}

Status ONICapture::SeekDepthStream (int frameIndex)
{
  return (Status)sensor_->SeekDepthStream(frameIndex);
}

Status ONICapture::SeekColorStream (int frameIndex)
{
  return (Status)sensor_->SeekColorStream(frameIndex);
}


Status ONICapture::SetRepeatEnabled (bool repeat)
{
  return (Status)sensor_->SetRepeatEnabled(repeat);
}

Status ONICapture::SetSpeed (float speed)
{
  return (Status)sensor_->SetSpeed(speed);
}

//void ONICapture::DrawObjectTracker()
//{
//	if (!sensor_->GetDepthFrame().isValid())
//	{
//		return;
//	}

//	if (!PROCESSOR::head_tracker.no_update)
//	{
//		// compute 3d point cloud
//		ConvertDepth2Cam3D();

//		// tracker update
//		PROCESSOR::update(0, sensor_->GetDepthFrame().getFrameIndex());
//	}

//	PROCESSOR::head_tracker.no_update = true;

//	//PROCESSOR::head_tracker.updateLabeling(getDepthFrame().getFrameIndex());
//	PROCESSOR::head_tracker.updateLabeling(sensor_->GetDepthFrame().getFrameIndex());

//	// tracker visualization
//	int num = PROCESSOR::head_tracker.getPeopleNum();
//	//cout<<num<<endl;
//	for (int i = 0; i < num; i++)
//	{
//		objectLocation3D curHead;
//		Eigen::Vector3f posInWorld;
//		Eigen::Vector3f posInCam;

//		PROCESSOR::head_tracker.getPeople(i, curHead);

//		// if (!curHead.isCurrentlyDetected() || !curHead.isMature())
//		// 	continue;

//		int id = curHead.getID();
//		posInWorld = curHead.getPosition();
//		PROCESSOR::floorCalibration.floor2cam(posInWorld, posInCam);

//		// convert from 3D to image plane
//		Point3D pt3d_cam;
//		Point2Di pt2d_img;
//		unsigned short depth_value;
//		pt3d_cam.x = posInCam(0);
//		pt3d_cam.y = posInCam(1);
//		pt3d_cam.z = posInCam(2);
//		Get2Dfrom3D(pt3d_cam, pt2d_img, depth_value);

//		int x = pt2d_img.x;
//		int y = pt2d_img.y;

//		DrawTracker(x, y, id);
//	}

//	//store all_people information. If there have an existing data, replace it.
//	vector<objectLocation3D> all_people;
//	PROCESSOR::head_tracker.getAllPeople(all_people);
//	FrameNoAndall_people tempFrameNoAndall_people;
//	tempFrameNoAndall_people.currentAll_people=all_people;
//	tempFrameNoAndall_people.nFrameNo=GetFrameIndex();
//	tempFrameNoAndall_people.newID=PROCESSOR::head_tracker.getnewID();

//	if(!bIsBackPlayOrForwardPlay)
//	{
//	    //if back play, just read. if forward play, merge previous result
//	    for(int i=0;i<vecFrameNoAndall_people.size();i++)
//	    {
//	      if(vecFrameNoAndall_people[i].nFrameNo==tempFrameNoAndall_people.nFrameNo)
//	      {
//		  vecFrameNoAndall_people[i].currentAll_people=all_people;
//		  vecFrameNoAndall_people[i].newID=PROCESSOR::head_tracker.getnewID();
//		  cout<<"merge"<<endl;
//		  return;
//	      }
//	    }
//	    vecFrameNoAndall_people.push_back(tempFrameNoAndall_people);
//	    cout<<"add new info into vec"<<endl;
//	}

//}

//void ONICapture::ConvertDepth2Cam3D()
//{
//	//const openni::DepthPixel* pDepth = (openni::DepthPixel*) g_depthFrame.getData();
//	const openni::DepthPixel* pDepth = (openni::DepthPixel*) sensor_->GetDepthFrame().getData();
//	//const openni::DepthPixel* pDepth = (openni::DepthPixel*) GetDepthData();

//	//const openni::RGB888Pixel* pColors = (openni::RGB888Pixel*) g_colorFrame.getData();
//	const openni::RGB888Pixel* pColors = (openni::RGB888Pixel*) sensor_->GetColorFrame().getData();
//	//const openni::RGB888Pixel* pColors = (openni::RGB888Pixel*) GetColorData();

//	//comment unused data
//	//unsigned int g_nYRes = 480;
//	//unsigned int g_nXRes = 640;
//	unsigned int nValue = 0;
//	unsigned int nX = 0;
//	unsigned int nY = 0;
//	int i = 0;

//	// if user generator is available,
//	// valid_point_num = 480 * 640;
//	unsigned char* pixelColor = (unsigned char*) pColors;

//	for (nY = 0; nY < 480; nY++)
//	{
//		for (nX = 0; nX < 640; nX++)
//		{
//			nValue = *pDepth;

//			float x, y, z;

//			if (nValue == 0)
//			{
//				x = 0.0f;
//				y = 0.0f;
//				z = 0.0f;
//			}
//			else
//				//openni::CoordinateConverter::convertDepthToWorld(g_depthStream,(int) nX, (int) nY, (int) nValue, &x, &y, &z);
//				openni::CoordinateConverter::convertDepthToWorld(sensor_->GetDepthStream(),(int) nX, (int) nY, (int) nValue, &x, &y, &z);
//			g_pointCam3D[3 * i + 0] = x;
//			g_pointCam3D[3 * i + 1] = y;
//			g_pointCam3D[3 * i + 2] = z;

//			pColorBuf[3 * i + 0] = pixelColor[0];
//			pColorBuf[3 * i + 1] = pixelColor[1];
//			pColorBuf[3 * i + 2] = pixelColor[2];

//			i++;
//			pDepth++;
//			pixelColor += 3;
//		}
//	}
//}

//void ONICapture::Get2Dfrom3D(Point3D p3d, Point2Di& p2d, unsigned short& depth_value)
//{
//	int x, y;
//	unsigned short d;
//	//openni::CoordinateConverter::convertWorldToDepth(g_depthStream, p3d.x, p3d.y, p3d.z, &x, &y,&d);
//	openni::CoordinateConverter::convertWorldToDepth(sensor_->GetDepthStream(), p3d.x, p3d.y, p3d.z, &x, &y,&d);

//	depth_value = d;
//	p2d.x = x;
//	p2d.y = y;

//}

//void ONICapture::Get3Dfrom2D(int x, int y, unsigned short depth_value, Point3D& output)
//{
//	Point3D projectivePoint;
////	Point3D realWorldPoint;
//	projectivePoint.x = (float) x;
//	projectivePoint.y = (float) y;
//	projectivePoint.z = depth_value;

//	//float x, y, z;
//	openni::CoordinateConverter::convertDepthToWorld(sensor_->GetDepthStream(), projectivePoint.x,
//			projectivePoint.y, projectivePoint.z, &output.x, &output.y,
//			&output.z);

//}

//void ONICapture::DrawTracker(int x, int y, int id)
//{
//	//char msg[100];
//	//sprintf(msg,"id: %d", id);
//	//DrawMessage(GLUT_BITMAP_TIMES_ROMAN_24, x, y, msg, 1, 0, 0);
//	//just paint simple ID on the head of the person
//	stringstream ss;
//	string strID;
//	ss<<id;
//	ss>>strID;
//	Mat mTempColorForPaintID=Mat(GetColorHeight(),GetColorWidth(),CV_8UC3);
//	mTempColorForPaintID.data=(uchar*)GetColorData();
//	//cvtColor(mTempColorForPaintID,mTempColorForPaintID,CV_RGB2BGR);
//	putText(mTempColorForPaintID,strID,Point(x,y),FONT_HERSHEY_PLAIN,1.0,Scalar(0,255,0),1,8,false);
////	imshow("x",mTempColorForPaintID);
////	waitKey(0);
//}

int ONICapture::GetFrameIndex()
{
  return sensor_->GetDepthFrame().getFrameIndex();
}

//void ONICapture::AddTrackPosByHand(int x,int y)
//{
//    PROCESSOR::head_tracker.addPosition(x,y);
//    cout<<"add"<<endl;

////  //cout<<"pos: "<<x<<" "<<y<<endl;
////  framepositions originPositions;
////  //PROCESSOR::head_tracker.getPositions(GetFrameIndex(),originPositions);
////  vector<objectLocation3D> all_people;
////  PROCESSOR::head_tracker.getAllPeople(all_people);
////  //find the nearst pos and if the distance betweem them is enough small, delete it. Otherwise, add a new ID and position
////  //int nIndex=FindNearstTrackGoal(originPositions,x,y);
////  int nIndex=FindNearstTrackGoal(all_people,x,y);
////  if(nIndex>=0)
////  {
////      //decide to change the ID by hand, so there is a problem that when the ID is uesd by other person, how to handle it?
////       PROCESSOR::head_tracker.setPosition(nIndex,x,y);
////       cout<<"set ID: "<<nIndex<<endl;
////  }
////  else
////  {
////       PROCESSOR::head_tracker.addPosition(x,y);
////       cout<<"add"<<endl;
////  }
//}

//void ONICapture::SetTrackIDByHand(int nID,int nIndex)
//{
//  //decide to change the ID by hand, so there is a problem that when the ID is uesd by other person, how to handle it?
//  PROCESSOR::head_tracker.setID(nID,nIndex);

//  vector<objectLocation3D> all_people;
//  PROCESSOR::head_tracker.getAllPeople(all_people);
//  cout<<"set a new ID: "<<nID<<"  after that the ID is: "<<all_people[nIndex].getID()<<endl;
//}

//int ONICapture::IsFindNearstTrackGoal(int x,int y)
//{
//  //cout<<"pos: "<<x<<" "<<y<<endl;
//  framepositions originPositions;
//  //PROCESSOR::head_tracker.getPositions(GetFrameIndex(),originPositions);
//  vector<objectLocation3D> all_people;
//  PROCESSOR::head_tracker.getAllPeople(all_people);
//  //find the nearst pos and if the distance betweem them is enough small, delete it. Otherwise, add a new ID and position
//  //int nIndex=FindNearstTrackGoal(originPositions,x,y);
//  int nIndex=FindNearstTrackGoal(all_people,x,y);
//  return nIndex;
//}

//int ONICapture::FindNearstTrackGoal(vector<objectLocation3D> all_people,int x,int y)
//{
//  double dMinDis=50;
//  int nIndex=-1;
//  for(int i=0;i<all_people.size();i++)
//  {
//      // convert from 3D to image plane
//      Point3D pt3d_cam;
//      Point2Di pt2d_img;
//      unsigned short depth_value;
//      Eigen::Vector3f posInWorld=all_people[i].getPosition();
//      Eigen::Vector3f posInCam;
//      PROCESSOR::floorCalibration.floor2cam(posInWorld,posInCam);
//      pt3d_cam.x=posInCam(0);
//      pt3d_cam.y=posInCam(1);
//      pt3d_cam.z=posInCam(2);

//      Get2Dfrom3D(pt3d_cam, pt2d_img, depth_value);

//      double dDis=ComputeEuclidean(pt2d_img,x,y);
//      if( dDis<dMinDis )
//      {
//        dMinDis=dDis;
//        nIndex=i;
//      }
//  }
//  //return the index of all_people[index]
//  return nIndex;
//}

//int ONICapture::FindNearstTrackGoal(framepositions originPositions,int x,int y)
//{
//  double dMinDis=50;
//  int nIndex=-1;
//  for(int i=0;i<originPositions.object_num;i++)
//  {
//      // convert from 3D to image plane
//      Point3D pt3d_cam;
//      Point2Di pt2d_img;
//      unsigned short depth_value;
//      pt3d_cam.x = originPositions.positions[i](0);
//      pt3d_cam.y = originPositions.positions[i](1);
//      pt3d_cam.z = originPositions.positions[i](2);
//      Get2Dfrom3D(pt3d_cam, pt2d_img, depth_value);

//      int nDis=ComputeEuclidean(pt2d_img,x,y);
//      if( nDis<dMinDis )
//      {
//        dMinDis=nDis;
//        nIndex=i;
//      }
//  }
//  return nIndex;
//}

//int ONICapture::ComputeEuclidean(Point2Di pt2d_img,int x,int y)
//{
//  return sqrt(  pow( pt2d_img.x-x ,2.0) +   pow( pt2d_img.y-y ,2.0)     );
//}

//void ONICapture::SetTrackingStateToCurrentFrame()
//{
//  bIsBackPlayOrForwardPlay=true;
//  int nCurrentFrameNo=GetFrameIndex();
//  vector<objectLocation3D> all_people;
//  if( GetTrackingState(nCurrentFrameNo,all_people) )
//  {
//      PROCESSOR::head_tracker.setTrackingState(all_people);
//  }
//}

//bool ONICapture::GetTrackingState(int nCurrentFrameNo,vector<objectLocation3D> &all_people)
//{
//  int nCurrentFrameNoGetFromClassInner=GetFrameIndex();
//  for(int i=0;i<vecFrameNoAndall_people.size();i++)
//  {
//      //if(vecFrameNoAndall_people[i].nFrameNo==nCurrentFrameNo)
//      if(vecFrameNoAndall_people[i].nFrameNo==nCurrentFrameNoGetFromClassInner)
//      {
//          all_people=vecFrameNoAndall_people[i].currentAll_people;
//          PROCESSOR::head_tracker.setnewID(vecFrameNoAndall_people[i].newID);
//          return true;
//      }
//  }
//  return false;
//}

//void ONICapture::DeletePeopleByIndex(int nIndex)
//{
//  PROCESSOR::head_tracker.deletePeopleByIndex(nIndex);
//}

//void ONICapture::DrawMessage(void* font, int x, int y, const char* message, float fRed, float fGreen, float fBlue)
//{
//	const XnUInt32 nMaxLines = 5;
//	XnChar buf[512];
//	XnChar* aLines[nMaxLines];
//	XnUInt32 anLinesWidths[nMaxLines];
//	XnUInt32 nLine = 0;
//	XnUInt32 nLineLengthChars = 0;
//	XnInt32 nLineLengthPixels = 0;
//	XnInt32 nMaxLineLength = 0;

//	aLines[0] = buf;

//	// parse message to lines
//	const char* pChar = message;
//	for (;;)
//	{
//		if (*pChar == '\n' || *pChar == '\0')
//		{
//			if (nLineLengthChars > 0)
//			{
//				aLines[nLine][nLineLengthChars++] = '\0';
//				aLines[nLine+1] = &aLines[nLine][nLineLengthChars];
//				anLinesWidths[nLine] = nLineLengthPixels;
//				nLine++;
//				if (nLineLengthPixels > nMaxLineLength)
//				{
//					nMaxLineLength = nLineLengthPixels;
//				}
//				nLineLengthPixels = 0;
//				nLineLengthChars = 0;
//			}

//			if (nLine >= nMaxLines || *pChar == '\0')
//			{
//				break;
//			}
//		}
//		else
//		{
//			aLines[nLine][nLineLengthChars++] = *pChar;
//			nLineLengthPixels += glutBitmapWidth(font, *pChar);
//		}
//		pChar++;
//	}

//	XnUInt32 nHeight = 20;
//	int nXLocation = x + 640;
//	int nYLocation = y;

//	// Draw black background
//	glEnable(GL_BLEND);
//	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

//	glBegin(GL_QUADS);
//	glColor4f(0, 0, 0, 0.3);
//	glVertex2i(nXLocation - nMaxLineLength/2, nYLocation - nHeight);
//	glVertex2i(nXLocation + nMaxLineLength/2, nYLocation - nHeight);
//	glVertex2i(nXLocation + nMaxLineLength/2, nYLocation + nHeight);
//	glVertex2i(nXLocation - nMaxLineLength/2, nYLocation + nHeight);
//	glEnd();

//	glDisable(GL_BLEND);

//	// show message
//	glColor3f(fRed, fGreen, fBlue);
//	for (XnUInt32 i = 0; i < nLine; ++i)
//	{
//		glRasterPos2i(nXLocation - nMaxLineLength/2 + (nMaxLineLength - anLinesWidths[i])/2, nYLocation + i * nHeight + nHeight/2);
//		glPrintString(font, aLines[i]);
//	}
//}


}
