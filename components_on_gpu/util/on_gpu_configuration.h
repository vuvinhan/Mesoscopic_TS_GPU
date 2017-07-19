/*
 * OnGPUCONFIGURATION.h
 *
 *  Created on: Feb 9, 2014
 *      Author: xuyan
 */

#ifndef ONGPUCONFIGURATION_H_
#define ONGPUCONFIGURATION_H_

#include "../util/shared_gpu_include.h"

class GPUSharedParameter {
public:

	//for small network
	int kOnGPULaneSize;
	int kOnGPUSegmentSize;
	int kOnGPUNodeSize;

	int kOnGPUStartTimeStep;
	int kOnGPUEndTimeStep;
	int kOnGPUUnitTimeStep; //sec
	int kOnGPUTotalTimeSteps;

	//Length Related
	int kOnGPUVehicleLength; //meter

	int kOnGPUMaxRouteLength;

	int kOnGPUGPUToCPUSimulationResultsCopyBufferSize;

	//additional buffer space
	int kOnGPUTotalVehicleSpace;

};

#endif /* ONGPUCONFIGURATION_H_ */
