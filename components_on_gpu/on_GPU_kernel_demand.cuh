#ifndef ONGPU_KERNEL_DEMAND_H_
#define ONGPU_KERNEL_DEMAND_H_

#include "../components_on_gpu/supply/on_GPU_memory.h"
#include "../components_on_gpu/supply/on_GPU_vehicle.h"
#include "../components_on_gpu/supply/on_GPU_new_lane_vehicles.h"
#include "../components_on_gpu/util/shared_gpu_include.h"
#include "../components_on_gpu/util/on_gpu_configuration.h"
#include "../components_on_gpu/on_GPU_Macro.h"

#include "../components_on_cpu/util/simulation_results.h"

#include "../components_on_cpu/network/linkTravelTimes.h"
#include "stdio.h"
# include <curand_kernel.h>

//Demand Function
__global__ void initPathCost(GPUMemory* gpu_data, int num_paths);
__global__ void markPaths(GPUMemory* gpu_data, GPUVehicle *vpool_gpu, int num_vehicles, int start_time, GPUSharedParameter* data_setting_gpu);
__global__ void computePathCosts(GPUMemory* gpu_data, int num_paths, GPUSharedParameter* data_setting_gpu);
__global__ void pathSelection(GPUMemory* gpu_data, GPUVehicle *vpool_gpu, int num_vehicles, int start_time, GPUSharedParameter* data_setting_gpu, curandState *state);

__device__ bool atomicIncrease(int* address, int* index, int limit);

/*
  * Demand Function Implementation
  */
__global__ void initPathCost(GPUMemory* gpu_data, int num_paths){
	int path_index = blockIdx.x * blockDim.x + threadIdx.x;
	if(path_index >= num_paths)
		return;
	gpu_data->current_paths[path_index] = -1;
}
__global__ void markPaths(GPUMemory* gpu_data, GPUVehicle *vpool_gpu, int num_vehicles, int start_time, GPUSharedParameter* data_setting_gpu) {

 	int veh_index = blockIdx.x * blockDim.x + threadIdx.x;
 	if (veh_index >= num_vehicles)
 		return;

 	int od_id = vpool_gpu[gpu_data->cur_interval_new_vehicles[veh_index]].od_id;
 	int path_start_index = gpu_data->od_path_mapping.path_start_index[od_id];
 	int path_end_index = path_start_index + gpu_data->od_path_mapping.num_paths[od_id]-1;
 	int entry_time = vpool_gpu[gpu_data->cur_interval_new_vehicles[veh_index]].entry_time;
 	int time_step = entry_time - start_time;
 	for(int pathId=path_start_index; pathId<=path_end_index; pathId++){
 		gpu_data->current_paths[time_step*data_setting_gpu->kOnGPUNumPaths+pathId] = pathId;
 		gpu_data->current_paths_time[time_step*data_setting_gpu->kOnGPUNumPaths+pathId] = entry_time;
 	}
 }
__global__ void computePathCosts(GPUMemory* gpu_data, int num_paths, GPUSharedParameter* data_setting_gpu){
	int path_index_in_array = blockIdx.x * blockDim.x + threadIdx.x;
	if(path_index_in_array >= num_paths)
		return;

	int path_index = gpu_data->current_paths[path_index_in_array];
	float total_tt = 0;
	float entry_time = gpu_data->current_paths_time[path_index];
	for(int i=0; i<gpu_data->path_links_segs[path_index].num_links; i++){
		int link_id = gpu_data->path_links_segs[path_index].path_links[i];
		int time_interval = entry_time/data_setting_gpu->kOnGPUTTInterval;

		float link_tt;
		if(time_interval >= data_setting_gpu->kOnGPUNumTTInfo){//exceed the range of TT info
			//use free flow TT
			link_tt = gpu_data->link_tt[link_id].travelTimes[0];
		}else{
			link_tt = gpu_data->link_tt[link_id].travelTimes[time_interval+1];
		}

		total_tt += link_tt;
		entry_time += link_tt;
	}
	gpu_data->current_paths_time[path_index] = total_tt;
}

__global__ void pathSelection(GPUMemory* gpu_data, GPUVehicle *vpool_gpu, int num_vehicles, int start_time, GPUSharedParameter* data_setting_gpu, curandState *state){
	int veh_index = blockIdx.x * blockDim.x + threadIdx.x;
	if (veh_index >= num_vehicles)
		return;

	int veh_pool_id = gpu_data->cur_interval_new_vehicles[veh_index];
	int od_id = vpool_gpu[veh_pool_id].od_id;

	int entry_time = vpool_gpu[veh_pool_id].entry_time;
	int time_step = entry_time - start_time;

	int path_start_index = gpu_data->od_path_mapping.path_start_index[od_id];
	int path_end_index = path_start_index + gpu_data->od_path_mapping.num_paths[od_id]-1;

	float logsum = 0;
	for(int pathId=path_start_index; pathId<=path_end_index; pathId++){
		int pathIndexInArray = time_step*data_setting_gpu->kOnGPUNumPaths+pathId;
		if(gpu_data->current_paths_time[pathIndexInArray] <= data_setting_gpu->kOnGPUMaxUtil){
			gpu_data->current_paths_time[pathIndexInArray] = exp(gpu_data->current_paths_time[pathIndexInArray]);
		}else{
			gpu_data->current_paths_time[pathIndexInArray] = data_setting_gpu->kOnGPUMaxFloat;
		}
		logsum += gpu_data->current_paths_time[pathIndexInArray];
	}

	if(logsum > data_setting_gpu->kOnGPUMaxFloat){
		logsum = data_setting_gpu->kOnGPUMaxFloat;
	}

	//select path based on the path utility
	curand_init (1234 , veh_index , 0 , &state[veh_index]);
	float random = curand_uniform(&state[veh_index]);
	float upperProb = 0;
	float currentProb;
	//default path
	int selected_path = path_end_index;

	for(int pathId = path_end_index; pathId >=  path_start_index; --pathId){
		int pathIndexInArray = time_step*data_setting_gpu->kOnGPUNumPaths+pathId;
		currentProb = gpu_data->current_paths_time[pathIndexInArray]/logsum;
		upperProb += currentProb;

		if(upperProb >= random){
			selected_path = pathId;
		}
	}
	//set path_id for this vehicle
	vpool_gpu[veh_pool_id].path_code = selected_path;

	//ready for the next lane, so next_path_index is set to 1, if the next_path_index == whole_path_length, it means cannot find path any more, can exit;
	vpool_gpu[veh_pool_id].next_path_index = 1;
	vpool_gpu[veh_pool_id].whole_path_length = gpu_data->path_links_segs[selected_path].num_segs;

	//insert new vehicle for each lane, each interval
	int first_seg = gpu_data->path_links_segs[selected_path].path_segs[0];
	int index;
	if(atomicIncrease(&gpu_data->new_vehicles_every_time_step[entry_time].new_vehicle_size[first_seg], &index, gpu_data->seg_pool.capacity[first_seg])){
		gpu_data->new_vehicles_every_time_step[entry_time].new_vehicles[first_seg][index] = veh_pool_id;
	}
}

__device__ bool atomicIncrease(int* address, int* index, int limit)
{
	int old = *address, assumed;
	do {
		assumed = old;
		if(old>=limit){
			return false;
		}else{
			old = atomicCAS(address, assumed, assumed+1);
		}
	}while (assumed != old);

	*index = old;
	return true;
}


#endif /* ONGPU_KERNEL_DEMAND_H_ */
