/*
 * Lane.h
 *
 *  Created on: Jan 1, 2017
 *      Author: Jenny
 */

#ifndef LANE_H_
#define LANE_H_

#include "../util/shared_cpu_include.h"
#include "node.h"

class Lane {
public:
	int lane_id;
	int seg_id;
	int veh_start_index;
	int buff_veh_start_index;
	int max_vehs;
};

#endif /* LANE_H_ */

