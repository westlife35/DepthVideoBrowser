#ifndef _EVENTINFO_H
#define _EVENTINFO_H

#include <vector>
#include <string>
#include "cv.h"
#include "highgui.h"
using namespace cv;
using namespace std;

struct eventInfoStruct
{
    string strEventType;
    int nLooseStartTime;
    int nTightenedStartTime;
    int nTightenedEndTime;
    int nLooseEndTime;
};

class EventInfo
{
private:
    vector<eventInfoStruct> vecEventInfo;
    int nTotalFrame;

public:
    EventInfo();
    void setTotalFrame(int nInput);
    void addNewEventInfo(string strEventTypeIn,int nLooseStartTimeIn,int nTightenedStartTimeIn,int nTightenedEndTimeIn,int nLooseEndTimeIn);
    void saveEventInfo(const string & fileName);
    void clearInfo();

};

#endif
