#include "tools.h"

namespace tools {
  
	//------------------------------------------------------------
	float dist_2d( float x1, float y1, float x2, float y2 ) {
	  return sqrt( pow2(x1-x2)+pow2(y1-y2) );
	}

	//------------------------------------------------------------
	float scale_to_range( float value, float src_min, float src_max, float dst_min, float dst_max ) {
	  float temp = (value-src_min)/(src_max-src_min)*(dst_max-dst_min)+dst_min;
	  if (temp>dst_max) return dst_max;
	  if (temp<dst_min) return dst_min;
	  else return temp;
	}

}  // namespace tools
