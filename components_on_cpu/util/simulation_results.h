/*
 * SimulationResults.h
 *
 *  Created on: Jan 3, 2014
 *      Author: xuyan
 */

#ifndef SIMULATIONRESULTS_H_
#define SIMULATIONRESULTS_H_

#include "../util/configurations_on_cpu.h"

class SimulationResults {
public:

	float flow[kSegmentSize];
	float density[kSegmentSize];
	float speed[kSegmentSize];
	float queue_length[kSegmentSize];
	int counts[kSegmentSize];

	int states[kSegmentSize];

};

#endif /* SIMULATIONRESULTS_H_ */
