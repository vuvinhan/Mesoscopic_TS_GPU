#ifndef SEGMENTS_H_
#define SEGMENTS_H_

#include "../../components_on_cpu/util/configurations_on_cpu.h"
#include "on_GPU_vehicle.h"

class SegmentPool {
public:
	//network
	int seg_ID[kSegmentSize];
	int lane_start_index[kSegmentSize];
	int num_lanes[kSegmentSize];
	int lane_end_index[kSegmentSize];

	//measurement
	float flow[kSegmentSize];
	float density[kSegmentSize];
	float speed[kSegmentSize];
	float queue_length[kSegmentSize];

	//for density calculation
	int veh_counts[kSegmentSize];
	int max_vehicles[kSegmentSize];
	float seg_length[kSegmentSize];
	int capacity[kSegmentSize];
	int output_capacity[kSegmentSize];
	int input_capacity[kSegmentSize];
	int empty_space[kSegmentSize];

	//for speed calculation
	float alpha[kSegmentSize];
	float beta[kSegmentSize];
	float max_density[kSegmentSize];
	float min_density[kSegmentSize];
	float MAX_speed[kSegmentSize];
	float MIN_speed[kSegmentSize];

	//For accumulated length estimation
	float speed_history[kTotalTimeSteps][kSegmentSize];

	//For queue length prediction
	//float last_time_empty_space[kSegmentSize];
	//float his_queue_length[kQueueLengthHistory][kSegmentSize];
	//float his_queue_length_weighting[kQueueLengthHistory][kSegmentSize];

	//float predicted_queue_length[kSegmentSize];
	//float predicted_empty_space[kSegmentSize];

	//For empty space update
	//int leaving_vehicle_counts[kSegmentSize];

	//Temp Variables
	bool blocked[kSegmentSize];
	//int blocked_in[kSegmentSize]; 	//-1 block input
								// > 0 value to check among all conditions
	//int blocked_out[kSegmentSize]; //-1 block output
								// > 0 value to check among all conditions
	//for debug, not used on GPU
	int processed[kSegmentSize]; //initial value is set to num_lanes
								 //every time a lane finished processing, decrease this value by 1
								 //when all lanes finished processing, this value is 0
	//float debug_data[kSegmentSize];

};

#endif /* SEGMENTS_H_ */
