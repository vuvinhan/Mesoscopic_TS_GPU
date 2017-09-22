#ifndef ONGPU_KERNAL_H_
#define ONGPU_KERNAL_H_

#include "../components_on_gpu/supply/on_GPU_memory.h"
#include "../components_on_gpu/supply/on_GPU_vehicle.h"
#include "../components_on_gpu/supply/on_GPU_new_lane_vehicles.h"
#include "../components_on_gpu/util/shared_gpu_include.h"
#include "../components_on_gpu/util/on_gpu_configuration.h"
#include "../components_on_gpu/on_GPU_Macro.h"

#include "../components_on_cpu/util/simulation_results.h"

//Supply Function
__global__ void SupplySimulationPreVehiclePassing(GPUMemory* gpu_data, int time_step, int segment_length, GPUSharedParameter* data_setting_gpu, GPUVehicle *vpool_gpu);
__global__ void SupplySimulationVehiclePassing(GPUMemory* gpu_data, int time_step, int node_length, GPUSharedParameter* data_setting_gpu, GPUVehicle *vpool_gpu);
__global__ void SupplySimulationAfterVehiclePassing(GPUMemory* gpu_data, int time_step, int segment_length, GPUSharedParameter* data_setting_gpu);
__global__ void supply_simulated_results_to_buffer(GPUMemory* gpu_data, int time_step, int segment_length, SimulationResults* buffer, GPUSharedParameter* data_setting_gpu);
__device__ int GetNextVehicleAtNode(GPUMemory* gpu_data, int node_index, int* lane_index, GPUSharedParameter* data_setting_gpu, GPUVehicle *vpool_gpu);

//Utility Function
__device__ float MinOnDevice(float one_value, float the_other);
__device__ float MaxOnDevice(float one_value, float the_other);

/*
  * Supply Function Implementation
  */__global__ void SupplySimulationPreVehiclePassing(GPUMemory* gpu_data, int time_step, int segment_length, GPUSharedParameter* data_setting_gpu, GPUVehicle *vpool_gpu) {

 	int lane_index = blockIdx.x * blockDim.x + threadIdx.x;
 	if (lane_index >= segment_length)
 		return;

 	int time_index = time_step;

 	int veh_cnt = gpu_data->lane_pool.vehicle_counts[lane_index];
 	float lane_length = gpu_data->lane_pool.lane_length[lane_index];

 	//load newly generated vehicles to the back of the lane
 	int can_insert_veh = MinOnDevice(gpu_data->new_vehicles_every_time_step[time_index].new_vehicle_size[lane_index], (gpu_data->lane_pool.max_vehicles[lane_index]-veh_cnt));
 	int new_start_idx = gpu_data->lane_pool.vehicle_start_index[lane_index] + veh_cnt;
 	for (int i = 0; i < can_insert_veh; i++) {
 		gpu_data->lane_vehicle_pool.vehicle_space[new_start_idx + i]  = gpu_data->new_vehicles_every_time_step[time_index].new_vehicles[lane_index][i];
 	}
 	veh_cnt = veh_cnt + can_insert_veh;

 	//update speed and density
 	float density_ = 0.0f;
 	float speed_ = 0.0f;

 	density_ = 1.0 * data_setting_gpu->kOnGPUVehicleLength * veh_cnt / lane_length;

 	if (density_ < data_setting_gpu->kOnGPUMinDensity)
 		speed_ = data_setting_gpu->kOnGPUMaxSpeed;
 	else {

 		speed_ = data_setting_gpu->kOnGPUMaxSpeed
 				* powf(1.0 - powf((density_ - data_setting_gpu->kOnGPUMinDensity) / data_setting_gpu->kOnGPUMaxDensity, data_setting_gpu->kOnGPUBeta), data_setting_gpu->kOnGPUAlpha);

 //		speed_ = data_setting_gpu->kOnGPUMaxSpeed
 //				- data_setting_gpu->kOnGPUMaxSpeed / (data_setting_gpu->kOnGPUMaxDensity - data_setting_gpu->kOnGPUMinDensity) * (density_ - data_setting_gpu->kOnGPUMinDensity);
 	}
 //		gpu_data->lane_pool.speed[lane_index] = ( gpu_data->lane_pool.MAX_SPEED[lane_index] - gpu_data->lane_pool.MIN_SPEED ) / gpu_data->lane_pool.max_density[lane_index] * ( gpu_data->lane_pool.max_density[lane_index] - 0 );

 	if (speed_ < data_setting_gpu->kOnGPUMinSpeed)
 		speed_ = data_setting_gpu->kOnGPUMinSpeed;

 //update speed history
 	gpu_data->lane_pool.speed_history[time_index][lane_index] = speed_;

 	gpu_data->lane_pool.density[lane_index] = density_;
 	gpu_data->lane_pool.speed[lane_index] = speed_;
 //estimated empty_space

 	float prediction_queue_length_ = 0.0f;

 	if (time_step < data_setting_gpu->kOnGPUStartTimeStep + 4 * data_setting_gpu->kOnGPUUnitTimeStep) {
 //		gpu_data->lane_pool.predicted_empty_space[lane_index] = gpu_data->lane_pool.his_queue_length[0][lane_index];
 //		gpu_data->lane_pool.predicted_queue_length[lane_index] = 0;

 		gpu_data->lane_pool.predicted_empty_space[lane_index] = MinOnDevice(
 				gpu_data->lane_pool.last_time_empty_space[lane_index] + (gpu_data->lane_pool.speed[lane_index] * data_setting_gpu->kOnGPUUnitTimeStep),
 				1.0f * lane_length);
 	} else {
 		prediction_queue_length_ = gpu_data->lane_pool.his_queue_length[0][lane_index];
 		prediction_queue_length_ += (gpu_data->lane_pool.his_queue_length[0][lane_index] - gpu_data->lane_pool.his_queue_length[1][lane_index])
 				* gpu_data->lane_pool.his_queue_length_weighting[0][lane_index];

 //		prediction_empty_space_ += (gpu_data->lane_pool.his_queue_length[1][lane_index] - gpu_data->lane_pool.his_queue_length[2][lane_index])
 //				* gpu_data->lane_pool.his_queue_length_weighting[1][lane_index];
 //
 //		prediction_empty_space_ += (gpu_data->lane_pool.his_queue_length[2][lane_index] - gpu_data->lane_pool.his_queue_length[3][lane_index])
 //				* gpu_data->lane_pool.his_queue_length_weighting[2][lane_index];

 		gpu_data->lane_pool.predicted_empty_space[lane_index] = MinOnDevice(
 				gpu_data->lane_pool.last_time_empty_space[lane_index] + (gpu_data->lane_pool.speed[lane_index] * data_setting_gpu->kOnGPUUnitTimeStep),
 				(lane_length - prediction_queue_length_));
 	}

 //	gpu_data->lane_pool.debug_data[lane_index] = gpu_data->lane_pool.predicted_empty_space[lane_index];
 //update Tp

 	float acc_offset = gpu_data->lane_pool.accumulated_offset[lane_index]+gpu_data->lane_pool.speed[lane_index] * data_setting_gpu->kOnGPUUnitTimeStep; //meter
 	int curr_tp=gpu_data->lane_pool.Tp[lane_index];

 	while (acc_offset >= lane_length) {
 		acc_offset -= gpu_data->lane_pool.speed_history[curr_tp][lane_index] * data_setting_gpu->kOnGPUUnitTimeStep;
 		curr_tp += data_setting_gpu->kOnGPUUnitTimeStep;
 	}
 	gpu_data->lane_pool.Tp[lane_index] = curr_tp;
 	gpu_data->lane_pool.accumulated_offset[lane_index] = acc_offset;

 	//update queue length
 	int queue_start = gpu_data->lane_pool.queue_length[lane_index] / data_setting_gpu->kOnGPUVehicleLength;
 	for (int queue_index = queue_start; queue_index < veh_cnt; queue_index++) {
 		int vehicle_index = gpu_data->lane_vehicle_pool.vehicle_space[gpu_data->lane_pool.vehicle_start_index[lane_index] + queue_index];
 //		if (gpu_data->lane_pool.vehicle_space[queue_index][lane_index]->entry_time <= gpu_data->lane_pool.Tp[lane_index]) {
 		if (vpool_gpu[vehicle_index].entry_time <= gpu_data->lane_pool.Tp[lane_index]) {
 			gpu_data->lane_pool.queue_length[lane_index] += data_setting_gpu->kOnGPUVehicleLength;
 		} else {
 			break;
 		}
 	}
 	//update veh counts
 	gpu_data->lane_pool.vehicle_counts[lane_index] = veh_cnt;
 	gpu_data->lane_pool.blocked_in[lane_index] = MinOnDevice(gpu_data->lane_pool.predicted_empty_space[lane_index]/data_setting_gpu->kOnGPUVehicleLength, data_setting_gpu->kOnGPULaneInputCapacityPerTimeStep);
 	gpu_data->lane_pool.blocked_out[lane_index] = MinOnDevice(data_setting_gpu->kOnGPULaneOutputCapacityPerTimeStep, veh_cnt);
 }


__global__ void SupplySimulationVehiclePassing(GPUMemory* gpu_data, int time_step, int node_length, GPUSharedParameter* data_setting_gpu, GPUVehicle *vpool_gpu) {

	int node_index = blockIdx.x * blockDim.x + threadIdx.x;
  	if (node_index >= node_length)
	  	return;

	int maximum_waiting_time = -1;
	int the_one_veh = -1;
	int lane_index = -1;

	int upstream_start_lane = gpu_data->node_pool.upstream_lane_start_index[node_index];
	int upstream_end_lane = gpu_data->node_pool.upstream_lane_end_index[node_index];

	//no upstream links, so, return -1, no vehicle
	if (upstream_start_lane < 0 || upstream_end_lane < 0)
		return;

	int no_ulanes = upstream_end_lane - upstream_start_lane + 1;
	int upstream_lanes_timediff[10]={-1};//double check max number of upstream lanes
	//initialize the array of time differences for all lane
	//and find the vehicle to move first
	for (int one_lane_index = upstream_start_lane; one_lane_index <= upstream_end_lane; one_lane_index++) {
		if (gpu_data->lane_pool.blocked_out[one_lane_index]>0) {
			int first_vehicle_ID = gpu_data->lane_vehicle_pool.vehicle_space[gpu_data->lane_pool.vehicle_start_index[one_lane_index]];
			int time_diff = gpu_data->lane_pool.Tp[one_lane_index] - vpool_gpu[first_vehicle_ID].entry_time;
			if (time_diff >= 0){
				upstream_lanes_timediff[one_lane_index-upstream_start_lane] = time_diff;
				if(time_diff>maximum_waiting_time){
					maximum_waiting_time = time_diff;
					lane_index = one_lane_index;
					the_one_veh = first_vehicle_ID;
				}
			}
			else{
				upstream_lanes_timediff[one_lane_index-upstream_start_lane] = -2; //blocked
			}
		}
	}

		/*
		 * Condition 1: The Lane is not NULL
		 * ----      2: Has Output Capacity
		 * ---       3: Is not blocked
		 * ---       4: Has vehicles
		 * ---       5: The vehicle can pass
		 */
	//get vehicle one by one until node max flow
	int max_flow = gpu_data->node_pool.max_acc_flow[node_index];
	while (max_flow>0) {
		if (the_one_veh < 0) {
					return;
		}
		int lane_arr_index = lane_index-upstream_start_lane;
		//move the_one_veh
		if ((vpool_gpu[the_one_veh].next_path_index) >= (vpool_gpu[the_one_veh].whole_path_length)) {
			//vehicle finished trip
			gpu_data->lane_pool.leaving_vehicle_counts[lane_index]++;
			max_flow--;
		}else{
			int next_lane = vpool_gpu[the_one_veh].path_code[vpool_gpu[the_one_veh].next_path_index];
			if (gpu_data->lane_pool.blocked_in[next_lane]-gpu_data->lane_pool.vehicle_passed_to_the_lane_counts[next_lane] > 0) {
				//lane choice is simplified
				vpool_gpu[the_one_veh].next_path_index++;

				//it is very critical to update the entry time when passing
				vpool_gpu[the_one_veh].entry_time = time_step;

				//add the vehicle to the next lane
				int buffer_vehicle_index = gpu_data->lane_pool.buffered_vehicle_start_index[next_lane] + gpu_data->lane_pool.vehicle_passed_to_the_lane_counts[next_lane];
				gpu_data->lane_buffered_vehicle_pool.buffered_vehicle_space[buffer_vehicle_index] = the_one_veh;

				gpu_data->lane_pool.vehicle_passed_to_the_lane_counts[next_lane]++;
				gpu_data->lane_pool.leaving_vehicle_counts[lane_index]++;
				max_flow--;
				upstream_lanes_timediff[lane_arr_index] = -1; //reset

			} else {
				upstream_lanes_timediff[lane_arr_index] = -2; //blocked
				gpu_data->lane_pool.blocked_in[next_lane] = -1;
			}
		}
		//fetch next vehicle in the lane that just has one vehicle passing to next lane
		if (upstream_lanes_timediff[lane_arr_index] != -2
				&& gpu_data->lane_pool.blocked_out[lane_index]-gpu_data->lane_pool.leaving_vehicle_counts[lane_index] > 0) {
			int first_vehicle_ID = gpu_data->lane_vehicle_pool.vehicle_space[gpu_data->lane_pool.vehicle_start_index[lane_index]+gpu_data->lane_pool.leaving_vehicle_counts[lane_index]];
			int time_diff = gpu_data->lane_pool.Tp[lane_index] - vpool_gpu[first_vehicle_ID].entry_time;
			if(time_diff >= 0){
				upstream_lanes_timediff[lane_arr_index] = time_diff;
			}else{
				upstream_lanes_timediff[lane_arr_index] = -2;
			}
		}
		//search for lane with largest time_diff in the time_diff array
		maximum_waiting_time = -1;
		lane_index = -1;
		for(int j=0; j<no_ulanes; j++){
			if(upstream_lanes_timediff[j]>maximum_waiting_time){
				maximum_waiting_time = upstream_lanes_timediff[j];
				lane_index = j+upstream_start_lane;
			}
		}
		if(maximum_waiting_time<0)
			return;
		the_one_veh = gpu_data->lane_vehicle_pool.vehicle_space[gpu_data->lane_pool.vehicle_start_index[lane_index]+gpu_data->lane_pool.leaving_vehicle_counts[lane_index]];
		//printf("6 %d %d %d %d %d \n", the_one_veh, lane_index, upstream_start_lane, i, gpu_data->node_pool.max_acc_flow[node_index]);
	}
}

__global__ void SupplySimulationAfterVehiclePassing(GPUMemory* gpu_data, int time_step, int segment_length, GPUSharedParameter* data_setting_gpu) {
	int lane_index = blockIdx.x * blockDim.x + threadIdx.x;
	if (lane_index >= segment_length)
		return;

	int veh_cnt = gpu_data->lane_pool.vehicle_counts[lane_index];
	int leaving_veh_cnt = gpu_data->lane_pool.leaving_vehicle_counts[lane_index];
	veh_cnt -= leaving_veh_cnt;

	//update vehicle list after passing vehicles to next lane
	int start_vehicle_pool_index = gpu_data->lane_pool.vehicle_start_index[lane_index];
	for (int j = 0; j < veh_cnt; j++) {
		gpu_data->lane_vehicle_pool.vehicle_space[start_vehicle_pool_index + j] = gpu_data->lane_vehicle_pool.vehicle_space[start_vehicle_pool_index + j + leaving_veh_cnt];
	}
	gpu_data->lane_pool.queue_length[lane_index] -= leaving_veh_cnt*data_setting_gpu->kOnGPUVehicleLength;
	gpu_data->lane_pool.flow[lane_index] += leaving_veh_cnt;
	gpu_data->lane_pool.leaving_vehicle_counts[lane_index] = 0;

	//update the queue history
	for (int i = 3; i > 0; i--) {
		gpu_data->lane_pool.his_queue_length[i][lane_index] = gpu_data->lane_pool.his_queue_length[i - 1][lane_index];
	}
	gpu_data->lane_pool.his_queue_length[0][lane_index] = gpu_data->lane_pool.queue_length[lane_index];

	//update the empty space
	gpu_data->lane_pool.empty_space[lane_index] = gpu_data->lane_pool.empty_space[lane_index] + gpu_data->lane_pool.speed[lane_index] * data_setting_gpu->kOnGPUUnitTimeStep;
	gpu_data->lane_pool.empty_space[lane_index] = MinOnDevice(gpu_data->lane_pool.lane_length[lane_index] - gpu_data->lane_pool.queue_length[lane_index], gpu_data->lane_pool.empty_space[lane_index]);

	//load passed vehicles to the back of the lane
	int buff_veh_start_idx = gpu_data->lane_pool.buffered_vehicle_start_index[lane_index];

	//vehicles which can pass
	int can_pass_veh = MinOnDevice(gpu_data->lane_pool.vehicle_passed_to_the_lane_counts[lane_index], (gpu_data->lane_pool.max_vehicles[lane_index]-veh_cnt));
	int new_start_idx  = start_vehicle_pool_index + veh_cnt;

	for (int i = 0; i < can_pass_veh; i++) {
		//pass the vehicle
		gpu_data->lane_vehicle_pool.vehicle_space[new_start_idx + i] = gpu_data->lane_buffered_vehicle_pool.buffered_vehicle_space[i + buff_veh_start_idx];
	}
	veh_cnt = veh_cnt + can_pass_veh;

	//compute empty space
	if (can_pass_veh > 0) {
		int empty_space = MinOnDevice(gpu_data->lane_pool.speed[lane_index] * data_setting_gpu->kOnGPUUnitTimeStep, gpu_data->lane_pool.empty_space[lane_index])
				- can_pass_veh * data_setting_gpu->kOnGPUVehicleLength;

		if (empty_space > 0)
			gpu_data->lane_pool.empty_space[lane_index] = empty_space;
		else
			gpu_data->lane_pool.empty_space[lane_index] = 0;
	}

	gpu_data->lane_pool.last_time_empty_space[lane_index] = gpu_data->lane_pool.empty_space[lane_index];
	gpu_data->lane_pool.vehicle_passed_to_the_lane_counts[lane_index] = 0;
	gpu_data->lane_pool.vehicle_counts[lane_index] = veh_cnt;

}

__global__ void supply_simulated_results_to_buffer(GPUMemory* gpu_data, int time_step, int segment_length, SimulationResults* buffer, GPUSharedParameter* data_setting_gpu) {
	int buffer_index = time_step % data_setting_gpu->kOnGPUGPUToCPUSimulationResultsCopyBufferSize;

	int lane_index = blockIdx.x * blockDim.x + threadIdx.x;
	if (lane_index >= segment_length)
		return;

	buffer[buffer_index].flow[lane_index] = gpu_data->lane_pool.flow[lane_index];
	buffer[buffer_index].density[lane_index] = gpu_data->lane_pool.density[lane_index];
	buffer[buffer_index].speed[lane_index] = gpu_data->lane_pool.speed[lane_index];
	buffer[buffer_index].queue_length[lane_index] = gpu_data->lane_pool.queue_length[lane_index];
	buffer[buffer_index].counts[lane_index] = gpu_data->lane_pool.vehicle_counts[lane_index];

}

__device__ float MinOnDevice(float one_value, float the_other) {
	if (one_value < the_other)
		return one_value;
	else
		return the_other;
}

__device__ float MaxOnDevice(float one_value, float the_other) {
	if (one_value > the_other)
		return one_value;
	else
		return the_other;
}
#endif /* ONGPU_KERNAL_H_ */
