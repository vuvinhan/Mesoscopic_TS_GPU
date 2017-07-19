/*
 * segment.h
 *
 *  Created on: Jan 1, 2017
 *      Author: Jenny
 */

#ifndef SEGMENT_H_
#define SEGMENT_H_

#include "../util/shared_cpu_include.h"
#include "node.h"

class Segment {
public:
	int seg_id;
	int lane_start_index;
	int num_lanes;
	float alpha;
	float beta;
	float max_density;
	float min_density;
	float MAX_speed;
	float MIN_speed;
	int capacity;
	float length; //unit: meters
};

#endif /* SEGMENT_H_ */
