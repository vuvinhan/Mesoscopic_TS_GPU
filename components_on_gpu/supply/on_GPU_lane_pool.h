#ifndef LANES_H_
#define LANES_H_

#include "../../components_on_cpu/util/configurations_on_cpu.h"
#include "on_GPU_vehicle.h"

class LanePool {
public:
	//network
	int lane_ID[kLaneSize];
	int Seg_ID[kLaneSize];

	//ETS framework
	int Tp[kLaneSize];
	int Tq[kLaneSize];
	float accumulated_offset[kLaneSize];

	//measurement
	//float flow[kLaneSize];
	//float density[kLaneSize];
	//float speed[kLaneSize];
	float queue_length[kLaneSize];

	//for density calculation
	//float lane_length[kLaneSize];
	int max_vehicles[kLaneSize];
	//int output_capacity[kLaneSize];
	//int input_capacity[kLaneSize];
	//float empty_space[kLaneSize];

	//for speed calculation
	//float alpha[kLaneSize];
	//float beta[kLaneSize];
	//float max_density[kLaneSize];
	//float min_density[kLaneSize];
	//float MAX_speed[kLaneSize];
	//float MIN_speed[kLaneSize];

	//for ring data structure
	int vehicle_counts[kLaneSize];
	int vehicle_start_index[kLaneSize];
	int first_veh_index[kLaneSize];

	int buffered_vehicle_counts[kLaneSize];
	int buffered_first_veh_index[kLaneSize];
	int vehicle_passed_to_the_lane_counts[kLaneSize];
	int ring_buffer_size[kLaneSize];

	//For accumulated length estimation
	//float speed_history[kTotalTimeSteps][kLaneSize];

	//For queue length prediction
	//float last_time_empty_space[kLaneSize];
	//float his_queue_length[kQueueLengthHistory][kLaneSize];
	//float his_queue_length_weighting[kQueueLengthHistory][kLaneSize];

	//float predicted_queue_length[kLaneSize];
	//float predicted_empty_space[kLaneSize];

	//For empty space update
	//int new_vehicle_join_counts[kLaneSize];
	int leaving_vehicle_counts[kLaneSize];

	//Temp Variables
	bool blocked[kLaneSize];
	//int blocked_in[kLaneSize]; 	//-1 block input
								// > 0 value to check among all conditions
	//int blocked_out[kLaneSize]; //-1 block output
								// > 0 value to check among all conditions
	int timediff[kLaneSize];
//for debug, not used on GPU
	//float debug_data[kLaneSize];

};

#endif /* LANES_H_ */
