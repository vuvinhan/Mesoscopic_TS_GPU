/*
 * OnGPUNodePool.h
 *
 *  Created on: Jan 2, 2014
 *      Author: xuyan
 */

#ifndef ONGPUNODEPOOL_H_
#define ONGPUNODEPOOL_H_

#include "../../components_on_cpu/util/configurations_on_cpu.h"

class NodePool {
public:
	int node_ID[kNodeSize];

	//if -1, means no such lane
	int upstream_seg_start_index[kNodeSize];
	int upstream_seg_end_index[kNodeSize];
	bool vnode[kNodeSize];
	//bool enode[kNodeSize];
	bool processed[kNodeSize];
	int cur_veh[kNodeSize];
	int cur_lane[kNodeSize];
	int upstream_start_lane_index[kNodeSize];
	int upstream_end_lane_index[kNodeSize];
};

#endif /* ONGPUNODEPOOL_H_ */
