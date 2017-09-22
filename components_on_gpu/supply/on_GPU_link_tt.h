/*
 * linkTravelTimes.h
 */

#ifndef GPU_LINK_TT_
#define GPU_LINK_TT_

#include "../../components_on_cpu/util/configurations_on_cpu.h"

class GPULinkTravelTimes {

public:
	float travelTimes[kNumTTInfo+1];
};

#endif /* GPU_LINK_TT_ */
