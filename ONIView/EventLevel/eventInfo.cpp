#include "eventInfo.h"



EventInfo::EventInfo()
{
    nTotalFrame=0;
}

void EventInfo::setTotalFrame(int nInput)
{
    nTotalFrame=nInput;
}

void EventInfo::addNewEventInfo(string strEventTypeIn,int nLooseStartTimeIn,int nTightenedStartTimeIn,int nTightenedEndTimeIn,int nLooseEndTimeIn)
{
    eventInfoStruct tempEventInfoStruct;
    tempEventInfoStruct.strEventType=strEventTypeIn;
    tempEventInfoStruct.nLooseStartTime=nLooseStartTimeIn;
    tempEventInfoStruct.nTightenedStartTime=nTightenedStartTimeIn;
    tempEventInfoStruct.nTightenedEndTime=nTightenedEndTimeIn;
    tempEventInfoStruct.nLooseEndTime=nLooseEndTimeIn;
    vecEventInfo.push_back(tempEventInfoStruct);
}

void EventInfo::saveEventInfo(const string &fileName)
{
    FileStorage fs = FileStorage(fileName, FileStorage::WRITE);

    int count = vecEventInfo.size();

    vector<Point3f> point_vect;
    vector<int> id_vect;

    fs << "total_frame" << nTotalFrame;
    fs << "event" << "[";
    for (int i = 0; i < count; ++i)
    {
        fs << "{:";
        fs <<"event_type" << vecEventInfo[i].strEventType;
        vector<int> vecTempIntTime;
        vecTempIntTime.push_back(vecEventInfo[i].nLooseStartTime);
        vecTempIntTime.push_back(vecEventInfo[i].nTightenedStartTime);
        vecTempIntTime.push_back(vecEventInfo[i].nTightenedEndTime);
        vecTempIntTime.push_back(vecEventInfo[i].nLooseEndTime);
        fs <<"event_time"<<vecTempIntTime;
        //fs <<"event_time" <<vecEventInfo[i].nLooseStartTime<<vecEventInfo[i].nTightenedStartTime<<vecEventInfo[i].nTightenedEndTime<<vecEventInfo[i].nLooseEndTime;
        fs<< "}";
    }
    fs << "]";
    printf("save xml to %s\n", fileName.c_str());

    fs.release();
}

void EventInfo::clearInfo()
{
    vecEventInfo.clear();
}
