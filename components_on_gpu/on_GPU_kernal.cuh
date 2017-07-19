#ifndef ONGPU_KERNAL_H_
#define ONGPU_KERNAL_H_

#include "../components_on_gpu/supply/on_GPU_memory.h"
#include "../components_on_gpu/supply/on_GPU_vehicle.h"
#include "../components_on_gpu/supply/on_GPU_new_lane_vehicles.h"
#include "../components_on_gpu/util/shared_gpu_include.h"
#include "../components_on_gpu/util/on_gpu_configuration.h"
#include "../components_on_gpu/on_GPU_Macro.h"

#include "../components_on_cpu/util/simulation_results.h"
#include "cuPrintf.cu"
#include "cuPrintf.cuh"
#include "stdio.h"

//Supply Function
__global__ void SupplySimulationPreVehiclePassing(GPUMemory* gpu_data, int time_step, int segment_length, GPUSharedParameter* data_setting_gpu, GPUVehicle *vpool_gpu);
__global__ void SupplySimulationVehiclePassingFirst(GPUMemory* gpu_data, int time_step, int node_length, GPUSharedParameter* data_setting_gpu, GPUVehicle *vpool_gpu);
__global__ void SupplySimulationVehiclePassing(GPUMemory* gpu_data, int time_step, int node_length, GPUSharedParameter* data_setting_gpu, GPUVehicle *vpool_gpu);
//__global__ void SupplySimulationVehiclePassingAll(GPUMemory* gpu_data, int time_step, int node_length, GPUSharedParameter* data_setting_gpu, GPUVehicle *vpool_gpu, SimulationResults* buffer);
__global__ void SupplySimulationVehiclePassingVNode(GPUMemory* gpu_data, int time_step, int node_length, GPUSharedParameter* data_setting_gpu, GPUVehicle *vpool_gpu);
__global__ void SupplySimulationAfterVehiclePassing(GPUMemory* gpu_data, int time_step, int lane_length, GPUSharedParameter* data_setting_gpu);
__global__ void supply_simulated_results_to_buffer(GPUMemory* gpu_data, int time_step, int segment_length, SimulationResults* buffer, GPUSharedParameter* data_setting_gpu);
__device__ int GetNextVehicleAtNode(GPUMemory* gpu_data, int node_index, int* lane_index, GPUSharedParameter* data_setting_gpu, GPUVehicle *vpool_gpu);

//Utility Function
__device__ void initTimeDiffArray(GPUMemory* gpu_data, GPUVehicle *vpool_gpu, int upstream_start_lane, int upstream_end_lane);
__device__ void updateTimeDiffArray(GPUMemory* gpu_data, GPUVehicle *vpool_gpu, int lane_id, int seg_id);
__device__ void getFirstVehicleID(GPUMemory* gpu_data, GPUVehicle *vpool_gpu, int* veh_id, int* lane_index, int upstream_start_lane, int upstream_end_lane);
__device__ void updateAfterVehicleFinishingTrip(GPUMemory* gpu_data, GPUSharedParameter* data_setting_gpu, int lane_id, int seg_id);
__device__ void updateAfterVehicleMovingToBuffer(GPUMemory* gpu_data, GPUVehicle *vpool_gpu, GPUSharedParameter* data_setting_gpu, int time_step, int veh_id, int lane_id, int seg_id, int next_seg_id);
__device__ void updateAfterVehicleMovingToNextSeg(GPUMemory* gpu_data, GPUVehicle *vpool_gpu, GPUSharedParameter* data_setting_gpu, int time_step, int veh_id, int lane_id, int seg_id, int next_seg_id);

__device__ bool atomicDecrease(int* address);
__device__ float MinOnDevice(float one_value, float the_other);
__device__ float MaxOnDevice(float one_value, float the_other);

/*
  * Supply Function Implementation
  */
__global__ void SupplySimulationPreVehiclePassing(GPUMemory* gpu_data, int time_step, int segment_length, GPUSharedParameter* data_setting_gpu, GPUVehicle *vpool_gpu) {

 	int seg_index = blockIdx.x * blockDim.x + threadIdx.x;
 	if (seg_index >= segment_length)
 		return;

 	int seg_veh_cnt = 0;

 	//update segment vehicle counts and input capacity after advance phase
 	for(int lane_index = gpu_data->seg_pool.lane_start_index[seg_index]; lane_index<=gpu_data->seg_pool.lane_end_index[seg_index]; ++lane_index){
 		seg_veh_cnt += gpu_data->lane_pool.vehicle_counts[lane_index];
 		gpu_data->seg_pool.input_capacity[seg_index] -= gpu_data->lane_pool.buffered_vehicle_counts[lane_index];
 	}
 	float seg_length = gpu_data->seg_pool.seg_length[seg_index];
 	gpu_data->seg_pool.input_capacity[seg_index] = gpu_data->seg_pool.capacity[seg_index];

 	//load newly generated vehicles to the back of the segment
 	int max_veh = gpu_data->seg_pool.max_vehicles[seg_index];
 	int can_insert_veh = MinOnDevice(gpu_data->new_vehicles_every_time_step[time_step].new_vehicle_size[seg_index], (max_veh-seg_veh_cnt));

 	//insert new vehicles into the segment
 	for (int i = 0; i < can_insert_veh; i++) {
 		//lane selection
 		//choose lane with shortest queue
 		//each segment has at least 1 lane
 		int lane_index = gpu_data->seg_pool.lane_end_index[seg_index];
 		int que = gpu_data->lane_pool.queue_length[lane_index];
 		for(int j=gpu_data->seg_pool.lane_end_index[seg_index]-1; j>=gpu_data->seg_pool.lane_start_index[seg_index]; --j){
 			int tmp = gpu_data->lane_pool.queue_length[j];
 			if(tmp < que){
 				que = tmp;
 				lane_index = j;
 			}
 		}
 		//insert new vehicle into the selected lane
 		gpu_data->lane_vehicle_pool.vehicle_space[gpu_data->lane_pool.vehicle_start_index[lane_index]+gpu_data->lane_pool.vehicle_counts[lane_index]]
									= gpu_data->new_vehicles_every_time_step[time_step].new_vehicles[seg_index][i];

 		gpu_data->lane_pool.vehicle_counts[lane_index]++;
 	}
 	//update segment vehicle count after loading new vehicles
 	seg_veh_cnt += can_insert_veh;

 	//update segment speed and density
 	float density_ = 0.0f;
 	float speed_ = 0.0f;

 	density_ = (1.0 * data_setting_gpu->kOnGPUVehicleLength * seg_veh_cnt) / (seg_length * gpu_data->seg_pool.num_lanes[seg_index]);

 	if (density_ <= gpu_data->seg_pool.min_density[seg_index])
 		speed_ = gpu_data->seg_pool.MAX_speed[seg_index];
 	else if(density_ >= gpu_data->seg_pool.max_density[seg_index]){
 		speed_ = gpu_data->seg_pool.MIN_speed[seg_index];
 	}else {
 		//use speed-density relationship
 		speed_ = gpu_data->seg_pool.MAX_speed[seg_index]
 				* powf(1.0 - powf((density_ - gpu_data->seg_pool.min_density[seg_index]) / gpu_data->seg_pool.max_density[seg_index], gpu_data->seg_pool.beta[seg_index]), gpu_data->seg_pool.alpha[seg_index]);
 	}

 	if (speed_ < gpu_data->seg_pool.MIN_speed[seg_index])
 		speed_ = gpu_data->seg_pool.MIN_speed[seg_index];

 	//update speed history
 	gpu_data->seg_pool.speed_history[time_step][seg_index] = speed_;
 	gpu_data->seg_pool.density[seg_index] = density_;
 	gpu_data->seg_pool.speed[seg_index] = speed_;

 	//calculate empty space
 	gpu_data->seg_pool.empty_space[seg_index] = (int)(seg_length/data_setting_gpu->kOnGPUVehicleLength)*gpu_data->seg_pool.num_lanes[seg_index] - seg_veh_cnt;

 	//update ETSF parameters for each lane in the segment
 	for(int lane_index = gpu_data->seg_pool.lane_start_index[seg_index]; lane_index<=gpu_data->seg_pool.lane_end_index[seg_index]; ++lane_index){
 		float acc_offset = gpu_data->lane_pool.accumulated_offset[lane_index]+gpu_data->seg_pool.speed[seg_index] * data_setting_gpu->kOnGPUUnitTimeStep; //meter
 		int curr_tp=gpu_data->lane_pool.Tp[lane_index];

 		while (acc_offset >= seg_length) {
 			acc_offset -= gpu_data->seg_pool.speed_history[curr_tp][seg_index] * data_setting_gpu->kOnGPUUnitTimeStep;
 			curr_tp += data_setting_gpu->kOnGPUUnitTimeStep;
 		}
 		gpu_data->lane_pool.Tp[lane_index] = curr_tp;
 		gpu_data->lane_pool.accumulated_offset[lane_index] = acc_offset;

 		//update queue length
 		int queue_index = gpu_data->lane_pool.queue_length[lane_index] / data_setting_gpu->kOnGPUVehicleLength;

 		for (; queue_index < gpu_data->lane_pool.vehicle_counts[lane_index]; queue_index++) {
 			int vehicle_index = gpu_data->lane_vehicle_pool.vehicle_space[gpu_data->lane_pool.vehicle_start_index[lane_index]+queue_index];
 			if (vpool_gpu[vehicle_index].entry_time > gpu_data->lane_pool.Tp[lane_index]) {
 				break;
 			}
 		}
 		gpu_data->lane_pool.queue_length[lane_index] = queue_index*data_setting_gpu->kOnGPUVehicleLength;

 		//prepare for the next kernel function
 		//gpu_data->lane_pool.blocked_in[lane_index] = 0;
 		gpu_data->lane_pool.blocked[lane_index] = false;
 		gpu_data->lane_pool.timediff[lane_index] = -1;
 	}

 	//update segment veh counts
 	gpu_data->seg_pool.veh_counts[seg_index] = seg_veh_cnt;
 	gpu_data->seg_pool.output_capacity[seg_index] = gpu_data->seg_pool.capacity[seg_index];
 	gpu_data->seg_pool.processed[seg_index] = gpu_data->seg_pool.num_lanes[seg_index];
 	gpu_data->num_processed_blocks = true;
 }

__device__ void initTimeDiffArray(GPUMemory* gpu_data, GPUVehicle *vpool_gpu, int upstream_start_lane, int upstream_end_lane){
	for (int one_lane_index = upstream_start_lane; one_lane_index <= upstream_end_lane; one_lane_index++) {
		//if(gpu_data->lane_pool.timediff[one_lane_index]<0){
			int seg_index = gpu_data->lane_pool.Seg_ID[one_lane_index];

			//if(gpu_data->seg_pool.processed[seg_index]==0 || gpu_data->lane_pool.blocked[one_lane_index]){
			//	continue;
			//}

			if ((gpu_data->lane_pool.vehicle_counts[one_lane_index]>0)
						&& gpu_data->seg_pool.output_capacity[seg_index]>0) {
				int first_vehicle_ID = gpu_data->lane_vehicle_pool.vehicle_space[gpu_data->lane_pool.vehicle_start_index[one_lane_index]];
				int time_diff = gpu_data->lane_pool.Tp[one_lane_index] - vpool_gpu[first_vehicle_ID].entry_time;
				if (time_diff >= 0){ //vehicle ready to move
					gpu_data->lane_pool.timediff[one_lane_index] = time_diff;
				}
				else{ //no vehicle ready to move, block the lane
					gpu_data->lane_pool.blocked[one_lane_index] = true; //blocked
					gpu_data->seg_pool.processed[seg_index]--;
				}
			}else{ //no vehicle ready to move, block lane
				gpu_data->lane_pool.blocked[one_lane_index] = true; //blocked
				gpu_data->seg_pool.processed[seg_index]--;
			}
		//}
	}
}

__device__ void updateTimeDiffArray(GPUMemory* gpu_data, GPUVehicle *vpool_gpu, int lane_id, int seg_id){
	if (!gpu_data->lane_pool.blocked[lane_id]
			&& gpu_data->lane_pool.vehicle_counts[lane_id] > 0
			&& gpu_data->seg_pool.output_capacity[seg_id]>0)
	{
		int first_vehicle_ID = gpu_data->lane_vehicle_pool.vehicle_space[gpu_data->lane_pool.vehicle_start_index[lane_id]];
		int time_diff = gpu_data->lane_pool.Tp[lane_id] - vpool_gpu[first_vehicle_ID].entry_time;
		if(time_diff >= 0){
			gpu_data->lane_pool.timediff[lane_id] = time_diff;
		}else{
			gpu_data->lane_pool.blocked[lane_id] = true;
			gpu_data->seg_pool.processed[seg_id]--;
		}
	}else{
		gpu_data->lane_pool.blocked[lane_id] = true;
		gpu_data->seg_pool.processed[seg_id]--;
	}
}

__device__ void getFirstVehicleID(GPUMemory* gpu_data, GPUVehicle *vpool_gpu, int* veh_id, int* lane_index, int upstream_start_lane, int upstream_end_lane){
	int maximum_waiting_time = -1;
	*veh_id = -1;
	*lane_index = -1;

	for(int j=upstream_start_lane; j<=upstream_end_lane; j++){
		if(gpu_data->lane_pool.timediff[j]>maximum_waiting_time){
			maximum_waiting_time = gpu_data->lane_pool.timediff[j];
			*lane_index = j;
		}
	}
	if(*lane_index>=0){
		*veh_id = gpu_data->lane_vehicle_pool.vehicle_space[gpu_data->lane_pool.vehicle_start_index[*lane_index]];
	}
}
__device__ void updateAfterVehicleFinishingTrip(GPUMemory* gpu_data, GPUSharedParameter* data_setting_gpu, int lane_id, int seg_id){
	gpu_data->lane_pool.vehicle_counts[lane_id]--;
	//shift remaining vehicles to queue front
	int start_vehicle_pool_index = gpu_data->lane_pool.vehicle_start_index[lane_id];
	for (int j = 1; j < gpu_data->lane_pool.vehicle_counts[lane_id]; j++) {
		gpu_data->lane_vehicle_pool.vehicle_space[start_vehicle_pool_index + j - 1] = gpu_data->lane_vehicle_pool.vehicle_space[start_vehicle_pool_index + j];
	}
	gpu_data->seg_pool.output_capacity[seg_id] --;
	atomicAdd(&gpu_data->seg_pool.empty_space[seg_id],1);
	gpu_data->seg_pool.flow[seg_id]++;
	gpu_data->lane_pool.queue_length[lane_id] -= data_setting_gpu->kOnGPUVehicleLength;
	gpu_data->lane_pool.timediff[lane_id] = -1; //reset
}
__device__ void updateAfterVehicleMovingToBuffer(GPUMemory* gpu_data, GPUVehicle *vpool_gpu, GPUSharedParameter* data_setting_gpu, int time_step, int veh_id, int lane_id, int seg_id, int next_seg_id){
	vpool_gpu[veh_id].next_path_index++;

	//lane selection
	//choose lane with shortest queue
	//each segment has at least 1 lane
	int next_lane = gpu_data->seg_pool.lane_end_index[next_seg_id];
	int que = gpu_data->lane_pool.queue_length[next_lane];
	for(int i=gpu_data->seg_pool.lane_end_index[next_seg_id]-1; i>=gpu_data->seg_pool.lane_start_index[next_seg_id]; --i){
		int tmp = gpu_data->lane_pool.queue_length[i];
		if(tmp < que){
			que = tmp;
			next_lane = i;
		}
	}

	//it is very critical to update the entry time when passing
	vpool_gpu[veh_id].entry_time = time_step;

	//add the vehicle to the next buffer
	int new_vehicle_index = gpu_data->lane_pool.buffered_first_veh_index[next_lane] + gpu_data->lane_pool.buffered_vehicle_counts[next_lane];
	gpu_data->lane_vehicle_pool.buffer_vehicle_space[new_vehicle_index] = veh_id;

	//update next lane/segment parameters
	gpu_data->lane_pool.buffered_vehicle_counts[next_lane]++;
	gpu_data->seg_pool.input_capacity[next_seg_id]--;

	//update current lane/segment parameters
	gpu_data->lane_pool.vehicle_counts[lane_id]--;

	//shift remaining vehicles to queue front
	int start_vehicle_pool_index = gpu_data->lane_pool.vehicle_start_index[lane_id];
	for (int j = 1; j < gpu_data->lane_pool.vehicle_counts[lane_id]; j++) {
		gpu_data->lane_vehicle_pool.vehicle_space[start_vehicle_pool_index + j - 1] = gpu_data->lane_vehicle_pool.vehicle_space[start_vehicle_pool_index + j];
	}
	gpu_data->lane_pool.queue_length[lane_id] -= data_setting_gpu->kOnGPUVehicleLength;

	gpu_data->seg_pool.output_capacity[seg_id]--;
	atomicAdd(&gpu_data->seg_pool.empty_space[seg_id],1);
	gpu_data->seg_pool.flow[seg_id]++;

	gpu_data->lane_pool.timediff[lane_id] = -1; //reset
}

__device__ void updateAfterVehicleMovingToNextSeg(GPUMemory* gpu_data, GPUVehicle *vpool_gpu, GPUSharedParameter* data_setting_gpu, int time_step, int veh_id, int lane_id, int seg_id, int next_seg_id){
	vpool_gpu[veh_id].next_path_index++;

	//lane selection
	//choose lane with shortest queue
	//each segment has at least 1 lane
	int next_lane = gpu_data->seg_pool.lane_end_index[next_seg_id];
	int que = gpu_data->lane_pool.queue_length[next_lane];
	for(int i=gpu_data->seg_pool.lane_end_index[next_seg_id]-1; i>=gpu_data->seg_pool.lane_start_index[next_seg_id]; --i){
		int tmp = gpu_data->lane_pool.queue_length[i];
		if(tmp < que){
			que = tmp;
			next_lane = i;
		}
	}

	//it is very critical to update the entry time when passing
	vpool_gpu[veh_id].entry_time = time_step;

	//add the vehicle to the next lane next seg
	int new_vehicle_index = gpu_data->lane_pool.vehicle_start_index[next_lane] + gpu_data->lane_pool.vehicle_counts[next_lane];
	gpu_data->lane_vehicle_pool.vehicle_space[new_vehicle_index] = veh_id;

	//update next lane/segment parameters
	gpu_data->lane_pool.vehicle_counts[next_lane]++;
	gpu_data->seg_pool.input_capacity[next_seg_id]--;
	gpu_data->seg_pool.empty_space[next_seg_id]++;

	//update current lane/segment parameters
	gpu_data->lane_pool.vehicle_counts[lane_id]--;
	//shift remaining vehicles to queue front
	int start_vehicle_pool_index = gpu_data->lane_pool.vehicle_start_index[lane_id];
	for (int j = 1; j < gpu_data->lane_pool.vehicle_counts[lane_id]; j++) {
		gpu_data->lane_vehicle_pool.vehicle_space[start_vehicle_pool_index + j - 1] = gpu_data->lane_vehicle_pool.vehicle_space[start_vehicle_pool_index + j];
	}
	gpu_data->lane_pool.queue_length[lane_id] -= data_setting_gpu->kOnGPUVehicleLength;

	gpu_data->seg_pool.output_capacity[seg_id]--;
	atomicAdd(&gpu_data->seg_pool.empty_space[seg_id],1);
	gpu_data->seg_pool.flow[seg_id]++;

	gpu_data->lane_pool.timediff[lane_id] = -1; //reset
}

__global__ void SupplySimulationVehiclePassingVNode(GPUMemory* gpu_data, int time_step, int node_length, GPUSharedParameter* data_setting_gpu, GPUVehicle *vpool_gpu){
	int node_index = blockIdx.x * blockDim.x + threadIdx.x;
	if (node_index >= node_length)
		return;

	//if no upstream segments, stop
	if (gpu_data->node_pool.upstream_seg_start_index[node_index] < 0 || gpu_data->node_pool.upstream_seg_end_index[node_index] < 0){
		return;
	}

	//if(!gpu_data->node_pool.vnode[node_index]){ //only process virtual nodes now
	//	return;
	//}
	//virtual node processing
	int upstream_start_lane = gpu_data->seg_pool.lane_start_index[gpu_data->node_pool.upstream_seg_start_index[node_index]];
	int upstream_end_lane = gpu_data->seg_pool.lane_end_index[gpu_data->node_pool.upstream_seg_end_index[node_index]];

	initTimeDiffArray(gpu_data, vpool_gpu, upstream_start_lane, upstream_end_lane);

	int the_one_veh = -1;
	int lane_index = -1;

	getFirstVehicleID(gpu_data, vpool_gpu, &the_one_veh, &lane_index, upstream_start_lane, upstream_end_lane);
	while(the_one_veh >= 0){
		int seg_index = gpu_data->lane_pool.Seg_ID[lane_index];
		if ((vpool_gpu[the_one_veh].next_path_index) >= (vpool_gpu[the_one_veh].whole_path_length)) {
			updateAfterVehicleFinishingTrip(gpu_data, data_setting_gpu, lane_index, seg_index);
		}
		else{
			int next_seg = vpool_gpu[the_one_veh].path_code[vpool_gpu[the_one_veh].next_path_index];
			if (gpu_data->seg_pool.input_capacity[next_seg]>0) //start if next segment has input capacity
			{
				updateAfterVehicleMovingToBuffer(gpu_data, vpool_gpu, data_setting_gpu, time_step, the_one_veh, lane_index, seg_index, next_seg);
			}
			else{
				gpu_data->lane_pool.blocked[lane_index] = true; //block upstream lane
				//gpu_data->seg_pool.processed[seg_index]--;
				gpu_data->lane_pool.timediff[lane_index] = -1; //reset
			}
		}

		//fech next vehicle
		updateTimeDiffArray(gpu_data, vpool_gpu, lane_index, seg_index);
		getFirstVehicleID(gpu_data, vpool_gpu, &the_one_veh, &lane_index, upstream_start_lane, upstream_end_lane);
	}
	//atomicAdd(&gpu_data->num_processed_blocks,1);
}

__global__ void SupplySimulationVehiclePassingFirst(GPUMemory* gpu_data, int time_step, int node_length, GPUSharedParameter* data_setting_gpu, GPUVehicle *vpool_gpu) {
	int node_index = blockIdx.x * blockDim.x + threadIdx.x;
  	if (node_index >= node_length)
	  	return;

	gpu_data->node_pool.processed[node_index] = false;

	if (gpu_data->node_pool.upstream_seg_start_index[node_index] < 0){
		gpu_data->node_pool.processed[node_index] = true;
		return;
	}

	int the_one_veh;
	int lane_index;

	initTimeDiffArray(gpu_data, vpool_gpu, gpu_data->node_pool.upstream_start_lane_index[node_index], gpu_data->node_pool.upstream_end_lane_index[node_index]);
	getFirstVehicleID(gpu_data, vpool_gpu, &the_one_veh, &lane_index, gpu_data->node_pool.upstream_start_lane_index[node_index], gpu_data->node_pool.upstream_end_lane_index[node_index]);

	gpu_data->node_pool.cur_lane[node_index] = lane_index;
	gpu_data->node_pool.cur_veh[node_index] = -1;

	if (the_one_veh>=0) {
		//move the_one_veh
		int seg_index = gpu_data->lane_pool.Seg_ID[lane_index]; //current segment

		if ((vpool_gpu[the_one_veh].next_path_index) >= (vpool_gpu[the_one_veh].whole_path_length)) {
			updateAfterVehicleFinishingTrip(gpu_data, data_setting_gpu, lane_index, seg_index);
		}else{
			//next segment
			int next_seg = vpool_gpu[the_one_veh].path_code[vpool_gpu[the_one_veh].next_path_index];

			if (gpu_data->seg_pool.input_capacity[next_seg]>0) //start if next segment has input capacity
			{
				if(gpu_data->node_pool.vnode[node_index]){
					updateAfterVehicleMovingToBuffer(gpu_data, vpool_gpu, data_setting_gpu, time_step, the_one_veh, lane_index, seg_index, next_seg);
				}
				else if(gpu_data->seg_pool.processed[next_seg]<=0){
					if(gpu_data->seg_pool.empty_space[next_seg]>0){
						updateAfterVehicleMovingToNextSeg(gpu_data, vpool_gpu, data_setting_gpu, time_step, the_one_veh, lane_index, seg_index, next_seg);
					}else{
						gpu_data->lane_pool.blocked[lane_index] = true; //block upstream segment
						gpu_data->seg_pool.processed[seg_index]--;
						gpu_data->lane_pool.timediff[lane_index] = -1; //reset
					}
				}
				else if(atomicDecrease(&gpu_data->seg_pool.empty_space[next_seg])>0){
					updateAfterVehicleMovingToNextSeg(gpu_data, vpool_gpu, data_setting_gpu, time_step, the_one_veh, lane_index, seg_index, next_seg);
				}else{
					gpu_data->node_pool.cur_veh[node_index] = the_one_veh;
				}
			}
			else{
				gpu_data->lane_pool.blocked[lane_index] = true; //block upstream lane
				gpu_data->seg_pool.processed[seg_index]--;
				gpu_data->lane_pool.timediff[lane_index] = -1; //reset
			}
		}

		if(gpu_data->node_pool.cur_veh[node_index] == -1){
			updateTimeDiffArray(gpu_data, vpool_gpu, lane_index, seg_index);
			getFirstVehicleID(gpu_data, vpool_gpu, &gpu_data->node_pool.cur_veh[node_index], &gpu_data->node_pool.cur_lane[node_index], gpu_data->node_pool.upstream_start_lane_index[node_index], gpu_data->node_pool.upstream_end_lane_index[node_index]);
			if(gpu_data->node_pool.cur_veh[node_index]>=0){
				gpu_data->num_processed_blocks = true;
			}else{
				gpu_data->node_pool.processed[node_index]=true;
			}
		}
	}else{
		gpu_data->node_pool.processed[node_index]=true;
	}
}

__global__ void SupplySimulationVehiclePassing(GPUMemory* gpu_data, int time_step, int node_length, GPUSharedParameter* data_setting_gpu, GPUVehicle *vpool_gpu) {
	int node_index = blockIdx.x * blockDim.x + threadIdx.x;
  	if (node_index >= node_length)
	  	return;

	if(gpu_data->node_pool.processed[node_index]){
		return;
	}

	int the_one_veh = gpu_data->node_pool.cur_veh[node_index];
	int lane_index = gpu_data->node_pool.cur_lane[node_index];

	//reset
	gpu_data->node_pool.cur_veh[node_index] = -1;

	//if (the_one_veh>=0) {
		//move the_one_veh
		int seg_index = gpu_data->lane_pool.Seg_ID[lane_index]; //current segment

		if ((vpool_gpu[the_one_veh].next_path_index) >= (vpool_gpu[the_one_veh].whole_path_length)) {
			updateAfterVehicleFinishingTrip(gpu_data, data_setting_gpu, lane_index, seg_index);
		}else{
			//next segment
			int next_seg = vpool_gpu[the_one_veh].path_code[vpool_gpu[the_one_veh].next_path_index];

			if (gpu_data->seg_pool.input_capacity[next_seg]>0) //start if next segment has input capacity
			{
				if(gpu_data->node_pool.vnode[node_index]){
					updateAfterVehicleMovingToBuffer(gpu_data, vpool_gpu, data_setting_gpu, time_step, the_one_veh, lane_index, seg_index, next_seg);
				}
				else if(gpu_data->seg_pool.processed[next_seg]<=0){
					if(gpu_data->seg_pool.empty_space[next_seg]>0){
						updateAfterVehicleMovingToNextSeg(gpu_data, vpool_gpu, data_setting_gpu, time_step, the_one_veh, lane_index, seg_index, next_seg);
					}else{
						gpu_data->lane_pool.blocked[lane_index] = true; //block upstream segment
						gpu_data->seg_pool.processed[seg_index]--;
						gpu_data->lane_pool.timediff[lane_index] = -1; //reset
					}
				}
				else if(atomicDecrease(&gpu_data->seg_pool.empty_space[next_seg])>0){
					updateAfterVehicleMovingToNextSeg(gpu_data, vpool_gpu, data_setting_gpu, time_step, the_one_veh, lane_index, seg_index, next_seg);
				}else{
					gpu_data->node_pool.cur_veh[node_index] = the_one_veh;
				}
			}
			else{
				gpu_data->lane_pool.blocked[lane_index] = true; //block upstream lane
				gpu_data->seg_pool.processed[seg_index]--;
				gpu_data->lane_pool.timediff[lane_index] = -1; //reset
			}
		}

		if(gpu_data->node_pool.cur_veh[node_index] == -1){
			updateTimeDiffArray(gpu_data, vpool_gpu, lane_index, seg_index);
			getFirstVehicleID(gpu_data, vpool_gpu, &gpu_data->node_pool.cur_veh[node_index], &gpu_data->node_pool.cur_lane[node_index],
					gpu_data->node_pool.upstream_start_lane_index[node_index], gpu_data->node_pool.upstream_end_lane_index[node_index]);
			if(gpu_data->node_pool.cur_veh[node_index]>=0){
				gpu_data->num_processed_blocks = true;
			}else{
				gpu_data->node_pool.processed[node_index]=true;
			}
		}
	//}else{
	//	gpu_data->node_pool.processed[node_index]=true;
	//}
}
__global__ void SupplySimulationAfterVehiclePassing(GPUMemory* gpu_data, int time_step, int lane_length, GPUSharedParameter* data_setting_gpu){
	int lane_index = blockIdx.x * blockDim.x + threadIdx.x;
	if (lane_index >= lane_length)
		return;
//
//	if(gpu_data->lane_pool.buffered_vehicle_counts[lane_index]==0)
//		return;
//
//	//gpu_data->lane_pool.vehicle_counts[lane_index] += gpu_data->lane_pool.vehicle_passed_to_the_lane_counts[lane_index] - gpu_data->lane_pool.leaving_vehicle_counts[lane_index];
//	//gpu_data->lane_pool.first_veh_index[lane_index] = (gpu_data->lane_pool.first_veh_index[lane_index]+gpu_data->lane_pool.leaving_vehicle_counts[lane_index])%gpu_data->lane_pool.ring_buffer_size[lane_index];
//
//	//vehicles which can pass
//	int can_pass_veh = MinOnDevice(gpu_data->lane_pool.buffered_vehicle_counts[lane_index], (gpu_data->lane_pool.max_vehicles[lane_index]-gpu_data->lane_pool.vehicle_counts[lane_index]));
//	gpu_data->lane_pool.vehicle_counts[lane_index] += can_pass_veh;
//
//	//update ring buffer indexes
//	gpu_data->lane_pool.buffered_first_veh_index[lane_index]=(gpu_data->lane_pool.first_veh_index[lane_index]+gpu_data->lane_pool.vehicle_counts[lane_index])%gpu_data->lane_pool.ring_buffer_size[lane_index];
//	gpu_data->lane_pool.buffered_vehicle_counts[lane_index]-=can_pass_veh;
//
//	//gpu_data->lane_pool.leaving_vehicle_counts[lane_index] = 0;
//	//gpu_data->lane_pool.vehicle_passed_to_the_lane_counts[lane_index] = 0;
	if(gpu_data->lane_pool.buffered_vehicle_counts[lane_index]!=0){
		//vehicles which can pass
		int can_pass_veh = MinOnDevice(gpu_data->lane_pool.buffered_vehicle_counts[lane_index], (gpu_data->lane_pool.max_vehicles[lane_index]-gpu_data->lane_pool.vehicle_counts[lane_index]));

		//move vehicles from virtual lane to lane
		for(int i=0; i<can_pass_veh; i++){
			int new_vehicle_index = gpu_data->lane_pool.vehicle_start_index[lane_index] + gpu_data->lane_pool.vehicle_counts[lane_index] + i;
			gpu_data->lane_vehicle_pool.vehicle_space[new_vehicle_index] = gpu_data->lane_vehicle_pool.buffer_vehicle_space[gpu_data->lane_pool.buffered_first_veh_index[lane_index]];

			//remove vehicle from virtual lane and shift existing vehicles to the front of the virtual queue
			int start_vehicle_pool_index = gpu_data->lane_pool.buffered_first_veh_index[lane_index];
			for (int j = 1; j < gpu_data->lane_pool.buffered_vehicle_counts[lane_index]; j++) {
				gpu_data->lane_vehicle_pool.buffer_vehicle_space[start_vehicle_pool_index + j - 1] = gpu_data->lane_vehicle_pool.buffer_vehicle_space[start_vehicle_pool_index + j];
			}
			gpu_data->lane_pool.buffered_vehicle_counts[lane_index]-=can_pass_veh;
		}
		gpu_data->lane_pool.vehicle_counts[lane_index] += can_pass_veh;
	}
}
__global__ void supply_simulated_results_to_buffer(GPUMemory* gpu_data, int time_step, int segment_length, SimulationResults* buffer, GPUSharedParameter* data_setting_gpu) {
	int buffer_index = time_step % data_setting_gpu->kOnGPUGPUToCPUSimulationResultsCopyBufferSize;

	int seg_index = blockIdx.x * blockDim.x + threadIdx.x;
	if (seg_index >= segment_length)
		return;

	buffer[buffer_index].flow[seg_index] = gpu_data->seg_pool.flow[seg_index];
	buffer[buffer_index].density[seg_index] = gpu_data->seg_pool.density[seg_index];
	buffer[buffer_index].speed[seg_index] = gpu_data->seg_pool.speed[seg_index];
	buffer[buffer_index].queue_length[seg_index] = gpu_data->seg_pool.queue_length[seg_index];
	buffer[buffer_index].counts[seg_index] = gpu_data->seg_pool.veh_counts[seg_index];
	if(seg_index<3186)
		buffer[buffer_index].states[seg_index] = gpu_data->node_pool.vnode[seg_index];

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

__device__ bool atomicDecrease(int* address)
{
	int old = *address, assumed;
	do {
		assumed = old;
		if(old<=0){
			return false;
		}else{
			old = atomicCAS(address, assumed, assumed-1);
		}
	}while (assumed != old);

	return true;
}

#endif /* ONGPU_KERNAL_H_ */
