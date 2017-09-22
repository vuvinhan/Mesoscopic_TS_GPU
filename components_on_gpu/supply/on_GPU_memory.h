/*
 * OnGPUMemory.h
 *
 *  Created on: Jan 2, 2014
 *      Author: xuyan
 */

#ifndef ONGPUMEMORY_H_
#define ONGPUMEMORY_H_

#include "../util/shared_gpu_include.h"
#include "on_GPU_lane_pool.h"
#include "on_GPU_lane_vehicle_pool.h"

#include "on_GPU_new_lane_vehicles.h"
#include "on_GPU_node_pool.h"

#include "on_GPU_segment_pool.h"
#include "on_GPU_od_path.h"
#include "on_GPU_link_tt.h"
#include "on_GPU_path_link_seg.h"

class GPUMemory {
public:

	LanePool lane_pool;
	NodePool node_pool;
	SegmentPool seg_pool;

	LaneVehiclePool lane_vehicle_pool;
	//LaneBufferedVehiclePool lane_buffered_vehicle_pool;
	int node_status[kNodeSize+1];
	bool num_processed_blocks;

	NewLaneVehicles new_vehicles_every_time_step[kTotalTimeSteps];
	int cur_interval_new_vehicles[kMaxNumVehPerInterval];

	int current_paths[kNumPaths*kTTInterval/kUnitTimeStep];
	float current_paths_time[kNumPaths*kTTInterval/kUnitTimeStep];

	GPUODPath od_path_mapping;
	GPUPathLinkSeg path_links_segs[kNumPaths];

	GPULinkTravelTimes link_tt[kLinkSize];

public:

	size_t total_size() {
		return sizeof(LanePool) + sizeof(NodePool) + sizeof(SegmentPool) + sizeof(LaneVehiclePool) + sizeof(NewLaneVehicles) * kTotalTimeSteps + sizeof(int)*(kNodeSize+1) + sizeof(bool)
				+ sizeof(int)*kSegmentSize*kMaxSegmentInputCapacityPerTimeStep + sizeof(int)*kNumPaths*kTTInterval/kUnitTimeStep + sizeof(float)*kNumPaths*kTTInterval/kUnitTimeStep
				+ sizeof(GPUODPath) + sizeof(GPUPathLinkSeg)*kNumPaths
				+ sizeof(GPULinkTravelTimes)*kLinkSize;
	}
};

#endif /* ONGPUMEMORY_H_ */
