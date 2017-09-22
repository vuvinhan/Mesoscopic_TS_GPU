/**
 *This project targets to check GPU is an option for DynaMIT.
 *This project also targets for a paper "Mesoscopic Traffic Simulation on GPU"
 */

#include "../components_on_cpu/network/network.h"
#include "../components_on_cpu/network/linkTravelTimes.h"
#include "../components_on_cpu/demand/od_mapping.h"
#include "../components_on_cpu/demand/od_path_mapping.h"
#include "../components_on_cpu/demand/vehicle.h"
#include "../components_on_cpu/demand/new_vehicles_per_interval.h"
#include "../components_on_cpu/util/time_tools.h"
#include "../components_on_cpu/util/string_tools.h"
#include "../components_on_cpu/util/simulation_results.h"
#include "../components_on_cpu/util/shared_cpu_include.h"
#include "../components_on_gpu/on_GPU_kernal_safe.cuh"
#include "../components_on_gpu/on_GPU_kernel_demand.cuh"
#include "../components_on_gpu/supply/on_GPU_memory.h"
#include "../components_on_gpu/supply/on_GPU_vehicle.h"
#include "../components_on_gpu/supply/on_GPU_new_lane_vehicles.h"
#include "../components_on_gpu/util/shared_gpu_include.h"
#include "../components_on_gpu/util/on_gpu_configuration.h"
#include "../components_on_gpu/on_GPU_Macro.h"
#include <math.h>
#include "../components_on_cpu/demand/path.h"
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
 #include <thrust/device_ptr.h>
#include <thrust/remove.h>
#include <thrust/fill.h>
#include <thrust/execution_policy.h>
#include <curand_kernel.h>

using namespace std;

/**
 * CUDA Execution Configuration
 */
int lane_blocks;
const int lane_threads_in_a_block = 128;
int node_blocks;
const int node_threads_in_a_block = 128;
int segment_blocks;
const int segment_threads_in_a_block = 128;

const int vehicle_threads_in_a_block = 128;

const int path_threads_in_a_block = 128;

/*
 * Demand
 */
Network* the_network;
std::vector<Path*> all_paths;
std::vector<Vehicle*> all_vehicles;
std::vector<ODPathMapping*> od_path_map;
ODMapping* all_od_pairs[kNumOD];
NewVehiclesPerInterval* new_interval_vehicles[kTotalTimeSteps/kTTInterval];
/*
 * Travel Time Table
 */
LinkTravelTimes* the_tt_table[kLinkSize];

/*
 * Path Input Config
 */
std::string network_file_path = "data_inputs/New_NW_SG/NewNetWork2.dat";
std::string demand_file_path = "data_inputs/New_NW_SG/newDemand.dat";
std::string od_pair_paths_file_path = "data_inputs/New_NW_SG/paths.dat";
std::string path_link_file_path = "data_inputs/New_NW_SG/path_link.dat";
std::string tt_file_path = "data_inputs/New_NW_SG/tt.dat";

/*
 * All data in GPU
 */
GPUMemory* gpu_data;
GPUMemory* data_local;

GPUSharedParameter* parameter_setting_on_gpu;
#ifdef ENABLE_CONSTANT_MEMORY
__constant__ GPUSharedParameter data_setting_gpu_constant;
#endif

//A large memory space is pre-defined in order to copy to GPU
GPUVehicle* vpool_cpu;
GPUVehicle* vpool_gpu;

//int *vpool_cpu_index;
//int *vpool_gpu_index;

/**
 * Simulation Results
 */
std::string simulation_output_file_path = "output/test3.txt";
std::map<int, SimulationResults*> simulation_results_pool;
ofstream simulation_results_output_file;

//buffer is only used when kGPUToCPUSimulationResultsCopyBufferSize > 1
SimulationResults* simulation_results_buffer_on_gpu;

//Used for buffer at CPU side
SimulationResults* one_buffer = NULL;

/*
 * GPU Streams
 * stream1: GPU Supply Simulation
 */
cudaStream_t stream_gpu_supply;
cudaStream_t stream_gpu_io;
cudaEvent_t gpu_supply_one_tick_simulation_done_trigger_event;

/*
 * Time Management
 */
long simulation_start_time;
long simulation_end_time;
long simulation_time_step;

/*
 * simulation_time is already finished time;
 * simulation_time + 1 might be the current simulating time on GPU
 */
long to_simulate_time;

/*
 * simulation_results_outputed_time is already outputted time;
 * simulation_results_outputed_time + 1 might be the outputing time on CPU
 */
long to_output_simulation_result_time;

/*
 */
//std::map<int, int> link_ID_to_link_Index;
//std::map<int, int> link_Index_to_link_ID;
//std::map<int, int> node_ID_to_node_Index;
//std::map<int, int> node_index_to_node_ID;
/*
 * Define Major Functions
 */
bool InitParams(int argc, char* argv[]);
bool LoadInNetwork();
bool LoadInDemand();
bool LoadInTravelTime();
bool InitilizeCPU();
bool InitilizeGPU();
bool InitGPUParameterSetting(GPUSharedParameter* data_setting_gpu);
bool InitGPUData(GPUMemory* data_local, NewVehiclesPerInterval** new_interval_vehicles);
bool StartSimulation();
bool StartDemandSimulation();
bool StartSimulationOptimizeWarp();
bool StartSimulationVP();
bool StartSimulationSynch();
bool DestroyResources();
int findODID(int orig, int dest);

/*
 * Define Helper Functions
 */
StringTools* str_tools;
bool CopySimulatedResultsToCPU(int time_step);
bool CopyBufferSimulatedResultsToCPU(int time_step);
bool OutputSimulatedResults(int time_step);
bool OutputBufferedSimulatedResults(int time_step);

inline int TimestepToArrayIndex(int time_step) {
	return (time_step - kStartTimeSteps) / kUnitTimeStep;
}

/*
 * MAIN
 */
int main(int argc, char* argv[]) {

	cudaDeviceSetCacheConfig(cudaFuncCachePreferL1);
	cout << "GPU Program Starts" << endl;

	if (InitParams(argc, argv) == false) {
		cout << "InitParams fails" << endl;
		return 0;
	}

	if (LoadInNetwork() == false) {
		cout << "Loading network fails" << endl;
		return 0;
	}

	if (LoadInDemand() == false) {
		cout << "Loading demand fails" << endl;
		return 0;
	}

	if (LoadInTravelTime() == false) {
		cout << "Loading Travel Time fails" << endl;
		return 0;
	}
	cout<<"Finished loading input files"<<endl;
	if (InitilizeCPU() == false) {
		cout << "InitilizeCPU fails" << endl;
		return 0;
	}
	cout<<"Finished initializing CPU"<<endl;
	if (InitilizeGPU() == false) {
		cout << "InitilizeGPU fails" << endl;
		return 0;
	}
	cout << "Finished initializing GPU"<<endl;
	//create streams
	//cudaStreamCreate(&stream_gpu_supply);
	//cudaStreamCreate(&stream_gpu_io);

	//create a event
	//cudaEventCreate(&gpu_supply_one_tick_simulation_done_trigger_event);

	std::cout << "Simulation Starts" << std::endl;

	//TimeTools profile;
	//profile.start_profiling();

	//Start Simulation (ETSF implemented inside)
	if (StartDemandSimulation() == false) {
		cout << "Simulation Fails" << endl;
		DestroyResources();
		return 0;
	}

	//profile.end_profiling();
	//profile.output();

	DestroyResources();
	cout << "Simulation Succeed!" << endl;

#ifdef _WIN32
	system("pause");
#endif

	return 0;
}

/**
 *
 */
bool InitParams(int argc, char* argv[]) {
	if (argc == 4) {
		network_file_path = argv[1];
		demand_file_path = argv[2];
		od_pair_paths_file_path = argv[3];
		std::cout << "network_file_path: "<<network_file_path<< std::endl;
		std::cout << "demand_file_path: "<<demand_file_path<< std::endl;
		std::cout << "od_pair_paths_file_path: "<<od_pair_paths_file_path<< std::endl;
	} else if (argc == 6){
		network_file_path = argv[1];
		demand_file_path = argv[2];
		od_pair_paths_file_path = argv[3];
		path_link_file_path = argv[4];
		tt_file_path = argv[5];
		std::cout << "network_file_path: "<<network_file_path<< std::endl;
		std::cout << "demand_file_path: "<<demand_file_path<< std::endl;
		std::cout << "od_pair_paths_file_path: "<<od_pair_paths_file_path<< std::endl;
		std::cout << "path_link_file_path: "<<path_link_file_path<< std::endl;
		std::cout << "tt_file_path: "<<tt_file_path<< std::endl;
	}
	return true;
}
bool LoadInNetwork() {
	the_network = new Network();
	return Network::load_network(*the_network, network_file_path);
}

bool LoadInDemand() {
	if (Path::load_in_all_Paths(all_paths, all_od_pairs, od_path_map, od_pair_paths_file_path, path_link_file_path) == false) {
		return false;
	}
	if (Vehicle::load_in_all_vehicles(all_vehicles, demand_file_path) == false) {
		return false;
	}
	return true;
}

bool LoadInTravelTime() {
	return LinkTravelTimes::load_in_all_link_tt(the_tt_table, tt_file_path);
}

bool InitilizeCPU() {
	simulation_start_time = 0;
	simulation_end_time = kEndTimeSteps-kStartTimeSteps; // 1 hour
	simulation_time_step = kUnitTimeStep;

	assert(simulation_time_step == 1);

	to_simulate_time = 0;
	to_output_simulation_result_time = 0;

	lane_blocks = kLaneSize / lane_threads_in_a_block + 1;
	node_blocks = kNodeSize / node_threads_in_a_block + 1;
	segment_blocks = kSegmentSize / segment_threads_in_a_block + 1;

	simulation_results_pool.clear();
	simulation_results_output_file.open(simulation_output_file_path.c_str());
	simulation_results_output_file << "##TIME STEP" << ":Segment ID:" << ":(" << "COUNTS" << ":" << "flow" << ":" << "density" << ":" << "speed" << ":" << "queue_length" << ")" << endl;
	str_tools = new StringTools();

	return true;
}

bool InitilizeGPU() {
	gpu_data = NULL;
	parameter_setting_on_gpu = NULL;

	data_local = new GPUMemory();

	InitGPUData(data_local, new_interval_vehicles);

	GPUSharedParameter* data_setting_gpu = new GPUSharedParameter();
	InitGPUParameterSetting(data_setting_gpu);

#ifdef ENABLE_CONSTANT_MEMORY
	GPUSharedParameter data_setting_cpu_constant;
	InitGPUParameterSetting(&data_setting_cpu_constant);
#endif
	//apply memory on GPU
	size_t memory_space_for_vehicles = all_vehicles.size() * sizeof(GPUVehicle);
	if (cudaMalloc((void**) &vpool_gpu, memory_space_for_vehicles) != cudaSuccess) {
		cerr << "cudaMalloc((void**) &vpool_gpu, memory_space_for_vehicles) failed" << endl;
	}

//	size_t memory_space_for_rebuild_index = kTotalTimeSteps * kLaneSize * kLaneInputCapacityPerTimeStep * sizeof(int);
//	if (cudaMalloc((void**) &vpool_gpu_index, memory_space_for_rebuild_index) != cudaSuccess) {
//		cerr << "cudaMalloc((void**) &vpool_gpu_index, memory_space_for_rebuild_index) failed" << endl;
//	}

	if (cudaMalloc((void**) &gpu_data, data_local->total_size()) != cudaSuccess) {
		cerr << "cudaMalloc(&gpu_data, sizeof(GPUMemory)) failed" << endl;
	}
	if (cudaMalloc((void**) &parameter_setting_on_gpu, sizeof(GPUSharedParameter)) != cudaSuccess) {
		cerr << "cudaMalloc(&GPUSharedParameter, sizeof(GPUSharedParameter)) failed" << endl;
	}

#ifdef ENABLE_CONSTANT_MEMORY
	cudaMemcpyToSymbol(data_setting_gpu_constant, &data_setting_cpu_constant, sizeof(GPUSharedParameter));
#endif

	//apply a buffer space for GPU outputs
	if (kGPUToCPUSimulationResultsCopyBufferSize > 1) {
		size_t memory_space_for_buffer_outputs = sizeof(SimulationResults) * kGPUToCPUSimulationResultsCopyBufferSize;
		if (cudaMalloc((void**) &simulation_results_buffer_on_gpu, memory_space_for_buffer_outputs) != cudaSuccess) {
			cerr << "cudaMalloc((void**) &simulation_results_buffer_on_gpu, memory_space_for_buffer_outputs) failed" << endl;
		}
	}
	cudaMemcpy(vpool_gpu, vpool_cpu, memory_space_for_vehicles, cudaMemcpyHostToDevice);
	cudaMemcpy(gpu_data, data_local, data_local->total_size(), cudaMemcpyHostToDevice);
	cudaMemcpy(parameter_setting_on_gpu, data_setting_gpu, sizeof(GPUSharedParameter), cudaMemcpyHostToDevice);

//	int GRID_SIZE = 1;
//	int BLOCK_SIZE = kTotalTimeSteps;
//
//	LinkGPUData<<<GRID_SIZE, BLOCK_SIZE>>>(gpu_data, kTotalTimeSteps, vpool_gpu, vpool_gpu_index, parameter_seeting_on_gpu);
//
//	//wait for all CUDA related operations to finish;
//	std::cout << "LinkGPUData begins" << std::endl;
//	cudaDeviceSynchronize();
//	std::cout << "LinkGPUData ends" << std::endl;

#ifdef ENABLE_OUTPUT_GPU_BUFFER
	cudaMallocHost((void **) &one_buffer, sizeof(SimulationResults) * kGPUToCPUSimulationResultsCopyBufferSize);
#endif

	return true;
}

/*
 * Copy the parameter setting to GPU memory
 */
bool InitGPUParameterSetting(GPUSharedParameter* data_setting_gpu) {
	data_setting_gpu->kOnGPULaneSize = kLaneSize;
	data_setting_gpu->kOnGPUNodeSize = kNodeSize;
	data_setting_gpu->kOnGPUSegmentSize = kSegmentSize;

	data_setting_gpu->kOnGPUEndTimeStep = kEndTimeSteps;
	data_setting_gpu->kOnGPUStartTimeStep = kStartTimeSteps;
	data_setting_gpu->kOnGPUTotalTimeSteps = kTotalTimeSteps;
	data_setting_gpu->kOnGPUUnitTimeStep = kUnitTimeStep;
	data_setting_gpu->kOnGPUVehicleLength = kVehicleLength;

	data_setting_gpu->kOnGPUMaxRouteLength = kMaxRouteLength;

	data_setting_gpu->kOnGPUGPUToCPUSimulationResultsCopyBufferSize = kGPUToCPUSimulationResultsCopyBufferSize;

	data_setting_gpu->kOnGPUTotalVehicleSpace = kTotalVehicleSpace;

	data_setting_gpu->kOnGPUTTInterval = kTTInterval;
	data_setting_gpu->kOnGPUNumTTInfo = kNumTTInfo;
	data_setting_gpu->kOnGPUNumPaths = kNumPaths;

	data_setting_gpu->kOnGPUMaxUtil = log(3.40282347e+38);
	data_setting_gpu->kOnGPUMaxFloat = 3.40282347e+38;

	data_setting_gpu->betaT = -0.0108879;

	return true;
}

/*
 * Build a GPU data from the network data
 */
bool InitGPUData(GPUMemory* data_local, NewVehiclesPerInterval** new_interval_vehicles) {
	data_local->num_processed_blocks = 0;
	/**
	 * First Part: Lane
	 */

	for (int i = 0; i < the_network->lane_size; i++) {
		Lane* one_lane = the_network->all_lanes[i];

		//
		assert(one_lane->lane_id == i);
		data_local->lane_pool.lane_ID[i] = one_lane->lane_id;
		data_local->lane_pool.Seg_ID[i] = one_lane->seg_id;

		data_local->lane_pool.Tp[i] = simulation_start_time - simulation_time_step;
		data_local->lane_pool.Tq[i] = simulation_start_time - simulation_time_step;
		data_local->lane_pool.accumulated_offset[i] = 0;

		//data_local->lane_pool.flow[i] = 0;
		//data_local->lane_pool.density[i] = 0;
		//data_local->lane_pool.speed[i] = 0;
		//data_local->lane_pool.queue_length[i] = 0;

		/*
		 * for density calculation
		 */
		data_local->lane_pool.max_vehicles[i] = one_lane->max_vehs; //number of vehicles

		/*
		 * for speed calculation
		 */

		data_local->lane_pool.vehicle_counts[i] = 0;
		data_local->lane_pool.vehicle_passed_to_the_lane_counts[i] = 0;
		data_local->lane_pool.leaving_vehicle_counts[i] = 0;

		data_local->lane_pool.vehicle_start_index[i] = one_lane->veh_start_index;
		//data_local->lane_pool.first_veh_index[i] = 0;

		data_local->lane_pool.buffered_first_veh_index[i] = 0;
		data_local->lane_pool.buffered_vehicle_counts[i] = 0;

		data_local->lane_pool.ring_buffer_size[i] = one_lane->max_vehs + kMaxSegmentInputCapacityPerTimeStep;

		data_local->lane_pool.queue_length[i] = 0;
	}
	std::cout << "Lane Pool size: "<<sizeof(data_local->lane_pool) << std::endl;

	/*
	 * Segment
	 */
	for(int i=0; i<the_network->seg_size; i++){
		Segment* one_seg = the_network->all_segs[i];
		data_local->seg_pool.seg_ID[i] = one_seg->seg_id;
		data_local->seg_pool.lane_start_index[i] = one_seg->lane_start_index;
		data_local->seg_pool.num_lanes[i] = one_seg->num_lanes;
		data_local->seg_pool.lane_end_index[i] = one_seg->lane_start_index + one_seg->num_lanes - 1;

		data_local->seg_pool.alpha[i] = one_seg->alpha;
		data_local->seg_pool.beta[i] = one_seg->beta;
		data_local->seg_pool.min_density[i] = one_seg->min_density;
		data_local->seg_pool.max_density[i] = one_seg->max_density;
		data_local->seg_pool.MIN_speed[i] = one_seg->MIN_speed;
		data_local->seg_pool.MAX_speed[i] = one_seg->MAX_speed;
		data_local->seg_pool.input_capacity[i] = one_seg->capacity;
		data_local->seg_pool.output_capacity[i] = one_seg->capacity;
		data_local->seg_pool.capacity[i] = one_seg->capacity;
		data_local->seg_pool.seg_length[i] = one_seg->length;

		data_local->seg_pool.density[i] = 0;
		data_local->seg_pool.speed[i] = 0;
		data_local->seg_pool.queue_length[i] = 0;
		data_local->seg_pool.flow[i] = 0;
		data_local->seg_pool.empty_space[i] = (int)(one_seg->length/kVehicleLength)*one_seg->num_lanes;
		data_local->seg_pool.veh_counts[i] = 0;
		data_local->seg_pool.max_vehicles[i] = one_seg->num_lanes*(int)(one_seg->length/kVehicleLength);
		data_local->seg_pool.processed[i] = one_seg->num_lanes;

		for (int j = 0; j < kTotalTimeSteps; j++) {
			data_local->seg_pool.speed_history[j][i] = -1;
		}

		//it is assumed that QUEUE_LENGTH_HISTORY = 4;
//		assert(kQueueLengthHistory == 4);
//		float weight[kQueueLengthHistory];
//		weight[0] = 1.0;
//		weight[1] = 0;
//		weight[2] = 0;
//		weight[3] = 0;
//
//		for (int j = 0; j < kQueueLengthHistory; j++) {
//			data_local->seg_pool.his_queue_length[j][i] = -1;
//			data_local->seg_pool.his_queue_length_weighting[j][i] = weight[j];
//		}
//
//		data_local->seg_pool.predicted_empty_space[i] = 0;
//		data_local->seg_pool.predicted_queue_length[i] = 0;
//		data_local->seg_pool.last_time_empty_space[i] = 0;
	}
	std::cout << "Segment Pool size: "<<sizeof(data_local->seg_pool) << std::endl;

	/**
	 * Third Part: Node
	 */
	for (int i = 0; i < the_network->node_size; i++) {
		Node* one_node = the_network->all_nodes[i];

		data_local->node_pool.node_ID[i] = one_node->node_id;
		data_local->node_pool.upstream_seg_start_index[i] = one_node->up_seg_start_index;
		data_local->node_pool.upstream_seg_end_index[i] = one_node->up_seg_end_index;
		data_local->node_pool.vnode[i] = one_node->vnode;
		//data_local->node_pool.enode[i] = one_node->enode;
		if(one_node->up_seg_start_index>=0){
			data_local->node_pool.upstream_start_lane_index[i] = data_local->seg_pool.lane_start_index[one_node->up_seg_start_index];
			data_local->node_pool.upstream_end_lane_index[i] = data_local->seg_pool.lane_end_index[one_node->up_seg_end_index];
		}
	}
	std::cout << "Node Pool Size: "<<sizeof(data_local->node_pool)<<std::endl;

	/*
	 * OD-Path Mapping
	 */
	int start_index = 0;
	for(int i=0; i<kNumOD; i++){
		ODPathMapping* one_mapping = od_path_map[i];
		data_local->od_path_mapping.od_id[i] = one_mapping->od_id;
		data_local->od_path_mapping.path_start_index[i] = start_index;
		data_local->od_path_mapping.num_paths[i] = one_mapping->path_ids.size();
		//std::cout<<"od:"<<all_od_pairs[i]->orig<<"-"<<all_od_pairs[i]->dest<<" start index:"<<start_index<<" num paths:"<<one_mapping->path_ids.size()<<"\n";
		start_index += one_mapping->path_ids.size();
	}
	std::cout << "od_path_mapping size: "<<sizeof(data_local->od_path_mapping)<<std::endl;
	/*
	 * path-link-seg mapping
	 */
	for(int i=0; i<kNumPaths; i++){
		Path* aPath = all_paths[i];
		data_local->path_links_segs[i].path_id = aPath->path_id;
		data_local->path_links_segs[i].num_links = aPath->link_ids.size();
		for(int j=0; j<data_local->path_links_segs[i].num_links; j++){
			data_local->path_links_segs[i].path_links[j] = aPath->link_ids[j];
		}
		data_local->path_links_segs[i].num_segs = aPath->seg_ids.size();
		for(int j=0; j<data_local->path_links_segs[i].num_segs; j++){
			data_local->path_links_segs[i].path_segs[j] = aPath->seg_ids[j];
		}
	}
	std::cout << "path table size: "<<sizeof(data_local->path_links_segs)<<std::endl;

	/*
	 * link travel times
	 */
	for(int i=0; i<kLinkSize; i++){
		LinkTravelTimes* oneLink = the_tt_table[i];
		if(oneLink){
			for(int j=0; j<=kNumTTInfo; j++){
				data_local->link_tt[i].travelTimes[j] = oneLink->travelTimes[j];
			}
		}
	}
	std::cout << "Travel Time table size: "<<sizeof(data_local->link_tt)<<std::endl;
	/**
	 * Vehicles
	 */

//Init VehiclePool
	for (int i = kStartTimeSteps; i < kEndTimeSteps; i += kUnitTimeStep) {
		for (int j = 0; j < kSegmentSize; j++) {
			data_local->new_vehicles_every_time_step[i-kStartTimeSteps].new_vehicle_size[j] = 0;
			data_local->new_vehicles_every_time_step[i-kStartTimeSteps].seg_ID[j] = -1;
		}
	}

//init host vehicle pool data /*xiaosong*/
	int memory_space_for_vehicles = all_vehicles.size() * sizeof(GPUVehicle);
	vpool_cpu = (GPUVehicle*) malloc(memory_space_for_vehicles);
	if (vpool_cpu == NULL)
		exit(1);

	for (int i = kStartTimeSteps; i < kEndTimeSteps; i += kUnitTimeStep) {
		for (int j = 0; j < kSegmentSize; j++) {
			for (int z = 0; z < kMaxSegmentInputCapacityPerTimeStep; z++) {
				//init as no vehicle
				data_local->new_vehicles_every_time_step[i-kStartTimeSteps].new_vehicles[j][z] = -1;
			}
		}
	}
//init new vehicles per interval
	for(int i = 0; i<kTotalTimeSteps/kTTInterval; i++){
		new_interval_vehicles[i] = new NewVehiclesPerInterval();
	}

//	int nVehiclePerTick = kLaneInputCapacityPerTimeStep * kLaneSize;
//	std::cout << "init all_vehicles" << std::endl;

	int total_inserted_vehicles = 0;

//Insert Vehicles
	for (int i = 0; i < all_vehicles.size(); i++) {
		Vehicle* one_vehicle = all_vehicles[i];

		int time_index = one_vehicle->entry_time;
		int time_index_convert = TimestepToArrayIndex(time_index);
		//assert(time_index == time_index_convert);

		//try to load vehicles beyond the simulation border
		if (time_index_convert >= kTotalTimeSteps || time_index_convert<0)
			continue;

		//int seg_ID = all_paths[one_vehicle->path_id]->seg_ids[0];
		//int seg_Index = seg_ID; //the same for the SG Expressway case

		//if (data_local->new_vehicles_every_time_step[time_index_convert].new_vehicle_size[seg_Index] < data_local->seg_pool.capacity[seg_Index]) {
			//int last_vehicle_index = data_local->new_vehicles_every_time_step[time_index_convert].new_vehicle_size[seg_Index];

			vpool_cpu[total_inserted_vehicles].vehicle_ID = one_vehicle->vehicle_id;
			vpool_cpu[total_inserted_vehicles].entry_time = time_index_convert;
			vpool_cpu[total_inserted_vehicles].od_id = findODID(one_vehicle->orig, one_vehicle->dest);
			//vpool_cpu[i].current_seg_ID = seg_Index;

			//assert(kMaxRouteLength > all_od_paths[one_vehicle->path_id]->seg_ids.size());
//			int max_copy_length = kMaxRouteLength > all_paths[one_vehicle->path_id]->seg_ids.size() ? all_paths[one_vehicle->path_id]->seg_ids.size() : kMaxRouteLength;
//
//			for (int p = 0; p < max_copy_length; p++) {
//				vpool_cpu[total_inserted_vehicles].path_code[p] = all_paths[one_vehicle->path_id]->seg_ids[p];
//			}
//
//			//ready for the next lane, so next_path_index is set to 1, if the next_path_index == whole_path_length, it means cannot find path any more, can exit;
//			vpool_cpu[total_inserted_vehicles].next_path_index = 1;
//			vpool_cpu[total_inserted_vehicles].whole_path_length = all_paths[one_vehicle->path_id]->seg_ids.size();

			//will be re-writen by GPU
			//insert new vehicle for each lane, each interval
//			data_local->new_vehicles_every_time_step[time_index_convert].new_vehicles[seg_Index][last_vehicle_index] = total_inserted_vehicles;//vpool_cpu[i].vehicle_ID;
//			data_local->new_vehicles_every_time_step[time_index_convert].new_vehicle_size[seg_Index]++;

			new_interval_vehicles[time_index_convert/kTTInterval]->veh_ids[new_interval_vehicles[time_index_convert/kTTInterval]->new_vehicle_size] = total_inserted_vehicles;
			new_interval_vehicles[time_index_convert/kTTInterval]->new_vehicle_size++;

			total_inserted_vehicles++;
		//} else {
//			std::cout << "Loading Vehicles Exceeds The Loading Capacity: Time:" << time_index_covert << ", Lane_ID:" << lane_ID << ",i:" << i << ",ID:" << one_vehicle->vehicle_id << std::endl;
		//}
	}

//	int max_n = 0;
//	for(int i=0; i<kTotalTimeSteps/kTTInterval; i++){
//		if(max_n < new_interval_vehicles[i]->new_vehicle_size){
//			max_n = new_interval_vehicles[i]->new_vehicle_size;
//		}
//	}
//	std::cout<<"max_n: "<<max_n<<"\n";

	std::cout << "init all_vehicles:" << total_inserted_vehicles << std::endl;
	std::cout << "vpool.size():" << total_inserted_vehicles * sizeof(GPUVehicle)<< std::endl;
	std::cout << "total global mem: "<< data_local->total_size()<<std::endl;

	return true;
}

int findODID(int orig, int dest){
	for(int i=0; i<kNumOD; i++){
		if(orig==all_od_pairs[i]->orig && dest==all_od_pairs[i]->dest){
			return i;
		}
	}
	return -1;
}

bool DestroyResources() {
	simulation_results_output_file.flush();
	simulation_results_output_file.close();

	if (vpool_cpu != NULL)
		delete vpool_cpu;
	if (str_tools != NULL)
		delete str_tools;

	cudaDeviceReset();
	return true;
}

bool StartDemandSimulation() {
	//=========================================DEMAND======================================================//
	int numPathsPerTTInterval = kNumPaths*kTTInterval/kUnitTimeStep;
	thrust::device_ptr<int> current_paths_start_ptr(gpu_data->current_paths);

	while(to_simulate_time < 30){
		std::cout<<to_simulate_time<<"\n";

		//Create in GPU a list of new vehicles departing this interval
		int cur_interval_num_vehicles = new_interval_vehicles[to_simulate_time]->new_vehicle_size;
		std::cout<<cur_interval_num_vehicles<<"\n";
		cudaMemcpy(gpu_data->cur_interval_new_vehicles, new_interval_vehicles[to_simulate_time]->veh_ids, sizeof(int)*cur_interval_num_vehicles, cudaMemcpyHostToDevice);

		//init current paths array to -1
		thrust::fill(current_paths_start_ptr, current_paths_start_ptr+numPathsPerTTInterval, -1);

		//Mark paths for path cost computation
		markPaths<<<ceil(cur_interval_num_vehicles/vehicle_threads_in_a_block), vehicle_threads_in_a_block, 0>>>(gpu_data, vpool_gpu, cur_interval_num_vehicles, to_simulate_time*kTTInterval, parameter_setting_on_gpu);

		//compact current_paths array
		thrust::device_ptr<int> new_end = thrust::remove(current_paths_start_ptr, current_paths_start_ptr + numPathsPerTTInterval, -1);

//		thrust::host_vector<int> cpu_current_paths(current_paths_start_ptr, new_end);
//		for(int i=0; i<cpu_current_paths.size(); i++){
//			std::cout<<cpu_current_paths[i]<<" ";
//		}
//		std::cout<<"\n";
//		std::cout<<new_end-current_paths_start_ptr<<"\n";
//		std::cout<<cpu_current_paths.size()<<"\n";

		//compute path cost
		computePathCosts<<<ceil((new_end-current_paths_start_ptr)/path_threads_in_a_block), path_threads_in_a_block, 0>>>(gpu_data, new_end-current_paths_start_ptr, parameter_setting_on_gpu);

		curandState* devStates;
		cudaMalloc ( &devStates, cur_interval_num_vehicles*sizeof( curandState ) );

		pathSelection<<<ceil(cur_interval_num_vehicles/vehicle_threads_in_a_block), vehicle_threads_in_a_block, 0>>>(gpu_data, vpool_gpu, cur_interval_num_vehicles, to_simulate_time*kTTInterval, parameter_setting_on_gpu, devStates);

		to_simulate_time++;
	}
	TimeTools profile;
	profile.start_profiling();

	while (to_simulate_time < simulation_end_time/kTTInterval) {
		//Create in GPU a list of new vehicles departing this interval
		int cur_interval_num_vehicles = new_interval_vehicles[to_simulate_time]->new_vehicle_size;
		cudaMemcpy(gpu_data->cur_interval_new_vehicles, new_interval_vehicles[to_simulate_time]->veh_ids, sizeof(int)*cur_interval_num_vehicles, cudaMemcpyHostToDevice);

		//init current paths array to -1
		thrust::fill(current_paths_start_ptr, current_paths_start_ptr+numPathsPerTTInterval, -1);

		//Mark paths for path cost computation
		markPaths<<<ceil(cur_interval_num_vehicles/vehicle_threads_in_a_block), vehicle_threads_in_a_block, 0>>>(gpu_data, vpool_gpu, cur_interval_num_vehicles, to_simulate_time*kTTInterval, parameter_setting_on_gpu);

		//compact current_paths array
		thrust::device_ptr<int> new_end = thrust::remove(current_paths_start_ptr, current_paths_start_ptr + numPathsPerTTInterval, -1);

		//compute path cost
		computePathCosts<<<ceil((new_end-current_paths_start_ptr)/path_threads_in_a_block), path_threads_in_a_block, 0>>>(gpu_data, new_end-current_paths_start_ptr, parameter_setting_on_gpu);

		curandState* devStates;
		cudaMalloc ( &devStates, cur_interval_num_vehicles*sizeof( curandState ) );

		pathSelection<<<ceil(cur_interval_num_vehicles/vehicle_threads_in_a_block), vehicle_threads_in_a_block, 0>>>(gpu_data, vpool_gpu, cur_interval_num_vehicles, to_simulate_time*kTTInterval, parameter_setting_on_gpu, devStates);

		to_simulate_time++;
	}
	profile.end_profiling();
	profile.output();
	return true;
}

bool StartSimulation() {

	while(to_simulate_time < 1800)
	{
		//=========================================SUPPLY==========================================//
		SupplySimulationPreVehiclePassing<<<segment_blocks, segment_threads_in_a_block, 0, stream_gpu_supply>>>(gpu_data, to_simulate_time, kSegmentSize, parameter_setting_on_gpu, vpool_gpu);

		bool num_processed_nodes = true;

		SupplySimulationVehiclePassingFirst<<<node_blocks, node_threads_in_a_block, 0, stream_gpu_supply>>>(gpu_data, to_simulate_time, kNodeSize, parameter_setting_on_gpu, vpool_gpu);

		cudaMemcpy(&num_processed_nodes, &gpu_data->num_processed_blocks, sizeof(bool), cudaMemcpyDeviceToHost);

		while(num_processed_nodes){
			num_processed_nodes = false;
			cudaMemcpy(&gpu_data->num_processed_blocks, &num_processed_nodes, sizeof(bool), cudaMemcpyHostToDevice);
			SupplySimulationVehiclePassing<<<node_blocks, node_threads_in_a_block, 0, stream_gpu_supply>>>(gpu_data, to_simulate_time, kNodeSize, parameter_setting_on_gpu, vpool_gpu);
			cudaMemcpy(&num_processed_nodes, &gpu_data->num_processed_blocks, sizeof(bool), cudaMemcpyDeviceToHost);
		}

		to_simulate_time += simulation_time_step;
	}


	TimeTools profile;
	profile.start_profiling();

	while (to_simulate_time < simulation_end_time) {

		SupplySimulationPreVehiclePassing<<<segment_blocks, segment_threads_in_a_block, 0, stream_gpu_supply>>>(gpu_data, to_simulate_time, kSegmentSize, parameter_setting_on_gpu, vpool_gpu);

		//SupplySimulationVehiclePassingVNode<<<node_blocks, node_threads_in_a_block, 0, stream_gpu_supply>>>(gpu_data, to_simulate_time, kNodeSize, parameter_setting_on_gpu, vpool_gpu);

		bool num_processed_nodes = true;

		SupplySimulationVehiclePassingFirst<<<node_blocks, node_threads_in_a_block, 0, stream_gpu_supply>>>(gpu_data, to_simulate_time, kNodeSize, parameter_setting_on_gpu, vpool_gpu);

		cudaMemcpy(&num_processed_nodes, &gpu_data->num_processed_blocks, sizeof(bool), cudaMemcpyDeviceToHost);
		//int n = 1;
		while(num_processed_nodes){
			num_processed_nodes = false;
			cudaMemcpy(&gpu_data->num_processed_blocks, &num_processed_nodes, sizeof(bool), cudaMemcpyHostToDevice);
			SupplySimulationVehiclePassing<<<node_blocks, node_threads_in_a_block, 0, stream_gpu_supply>>>(gpu_data, to_simulate_time, kNodeSize, parameter_setting_on_gpu, vpool_gpu);
			cudaMemcpy(&num_processed_nodes, &gpu_data->num_processed_blocks, sizeof(bool), cudaMemcpyDeviceToHost);
			//cudaMemcpy(nodes_processed, gpu_data->seg_pool.processed, sizeof(bool)*kSegmentSize, cudaMemcpyDeviceToHost);
//					for(int i=0; i<kSegmentSize; i++){
//						if(nodes_processed[i]>0){
//							std::cout<<"time "<<to_simulate_time<<" iter "<<n<<" node "<<i<<"\n";
//						}
//					}
			//std::cout<<"Interval "<<n<<' '<<num_processed_nodes<<'\n';
			//n++;
		}
		//std::cout<<n<<'\n';

		//SupplySimulationAfterVehiclePassing<<<lane_blocks, lane_threads_in_a_block, 0, stream_gpu_supply>>>(gpu_data, to_simulate_time, kLaneSize, parameter_setting_on_gpu);
		to_simulate_time += simulation_time_step;
	}
	profile.end_profiling();
	profile.output();
	return true;
}

bool StartSimulationOptimizeWarp() {
	TimeTools profile;

	int num_unprocessed_nodes;
	int updated_count;

	while (to_simulate_time < 1800) {
		//std::cout<<to_simulate_time<<"\n";

		SupplySimulationPreVehiclePassing<<<segment_blocks, segment_threads_in_a_block, 0, stream_gpu_supply>>>(gpu_data, to_simulate_time, kSegmentSize, parameter_setting_on_gpu, vpool_gpu);

		num_unprocessed_nodes = kNodeSize;

		SupplySimulationVehiclePassingFirstOptimizeWarp<<<node_blocks, node_threads_in_a_block, 0, stream_gpu_supply>>>(gpu_data, to_simulate_time, kNodeSize, parameter_setting_on_gpu, vpool_gpu);
		cudaMemcpy(data_local->node_status, gpu_data->node_status, sizeof(int)*(num_unprocessed_nodes+1), cudaMemcpyDeviceToHost);

		//std::cout<<"After: "<<num_unprocessed_nodes<<"\n";

		while(data_local->node_status[num_unprocessed_nodes]>0){
			updated_count = 0;
			for(int i=0; i<num_unprocessed_nodes; i++){
				if(data_local->node_status[i]>=0){
					data_local->node_status[updated_count] = data_local->node_status[i];
					updated_count++;
				}
			}
			num_unprocessed_nodes = updated_count;
			data_local->node_status[num_unprocessed_nodes]=0;

			cudaMemcpy(gpu_data->node_status, data_local->node_status, sizeof(int)*(num_unprocessed_nodes+1), cudaMemcpyHostToDevice);

			SupplySimulationVehiclePassingOptimizeWarp<<<ceil(num_unprocessed_nodes/node_threads_in_a_block), node_threads_in_a_block, 0, stream_gpu_supply>>>(gpu_data, to_simulate_time, num_unprocessed_nodes, parameter_setting_on_gpu, vpool_gpu);

			cudaMemcpy(data_local->node_status, gpu_data->node_status, sizeof(int)*(num_unprocessed_nodes+1), cudaMemcpyDeviceToHost);

			//std::cout<<to_simulate_time<<" "<<num_unprocessed_nodes<<'\n';
//			std::cout<<to_simulate_time<<" ";
//			for(int i=0; i<num_unprocessed_nodes; i++){
//				std::cout<<data_local->node_status[i]<<" ";
//			}
//			std::cout<<"\n";
		}
		to_simulate_time += simulation_time_step;
	}

	profile.start_profiling();

	while (to_simulate_time < simulation_end_time) {

		SupplySimulationPreVehiclePassing<<<segment_blocks, segment_threads_in_a_block, 0, stream_gpu_supply>>>(gpu_data, to_simulate_time, kSegmentSize, parameter_setting_on_gpu, vpool_gpu);

		num_unprocessed_nodes = kNodeSize;

		SupplySimulationVehiclePassingFirstOptimizeWarp<<<node_blocks, node_threads_in_a_block, 0, stream_gpu_supply>>>(gpu_data, to_simulate_time, kNodeSize, parameter_setting_on_gpu, vpool_gpu);

		cudaMemcpy(data_local->node_status, gpu_data->node_status, sizeof(int)*(num_unprocessed_nodes+1), cudaMemcpyDeviceToHost);

		//std::cout<<"After: "<<num_unprocessed_nodes<<"\n";

		while(data_local->node_status[num_unprocessed_nodes]>0){
			updated_count = 0;
			for(int i=0; i<num_unprocessed_nodes; i++){
				if(data_local->node_status[i]>=0){
					data_local->node_status[updated_count] = data_local->node_status[i];
					updated_count++;
				}
			}
			num_unprocessed_nodes = updated_count;
			data_local->node_status[num_unprocessed_nodes]=0;

			cudaMemcpy(gpu_data->node_status, data_local->node_status, sizeof(int)*(num_unprocessed_nodes+1), cudaMemcpyHostToDevice);

			SupplySimulationVehiclePassingOptimizeWarp<<<ceil(num_unprocessed_nodes/node_threads_in_a_block), node_threads_in_a_block, 0, stream_gpu_supply>>>(gpu_data, to_simulate_time, num_unprocessed_nodes, parameter_setting_on_gpu, vpool_gpu);

			cudaMemcpy(data_local->node_status, gpu_data->node_status, sizeof(int)*(num_unprocessed_nodes+1), cudaMemcpyDeviceToHost);

			//std::cout<<to_simulate_time<<" "<<num_unprocessed_nodes<<'\n';
//			std::cout<<to_simulate_time<<" ";
//			for(int i=0; i<num_unprocessed_nodes; i++){
//				std::cout<<data_local->node_status[i]<<" ";
//			}
//			std::cout<<"\n";
		}
		to_simulate_time += simulation_time_step;
	}
	profile.end_profiling();
	profile.output();
	return true;
}

bool StartSimulationVP() {
	TimeTools profile;
	//profile.start_profiling();

	while (to_simulate_time < 1800) {

		SupplySimulationPreVehiclePassing<<<segment_blocks, segment_threads_in_a_block, 0, stream_gpu_supply>>>(gpu_data, to_simulate_time, kSegmentSize, parameter_setting_on_gpu, vpool_gpu);

		SupplySimulationVehiclePassingVNode<<<node_blocks, node_threads_in_a_block, 0, stream_gpu_supply>>>(gpu_data, to_simulate_time, kNodeSize, parameter_setting_on_gpu, vpool_gpu);

		to_simulate_time += simulation_time_step;
	}
	//profile.end_profiling();
	//profile.output();

	profile.start_profiling();

	while (to_simulate_time < simulation_end_time) {

		SupplySimulationPreVehiclePassing<<<segment_blocks, segment_threads_in_a_block, 0, stream_gpu_supply>>>(gpu_data, to_simulate_time, kSegmentSize, parameter_setting_on_gpu, vpool_gpu);

		SupplySimulationVehiclePassingVNode<<<node_blocks, node_threads_in_a_block, 0, stream_gpu_supply>>>(gpu_data, to_simulate_time, kNodeSize, parameter_setting_on_gpu, vpool_gpu);

		to_simulate_time += simulation_time_step;
	}
	profile.end_profiling();
	profile.output();
	return true;
}

bool StartSimulationSynch() {
	TimeTools profile;
	//profile.start_profiling();

	while (to_simulate_time < 1800) {

		SupplySimulationPreVehiclePassing<<<segment_blocks, segment_threads_in_a_block, 0, stream_gpu_supply>>>(gpu_data, to_simulate_time, kSegmentSize, parameter_setting_on_gpu, vpool_gpu);

		SupplySimulationVehiclePassingSynch<<<node_blocks, node_threads_in_a_block, 0, stream_gpu_supply>>>(gpu_data, to_simulate_time, kNodeSize, parameter_setting_on_gpu, vpool_gpu);

		to_simulate_time += simulation_time_step;
	}
	//profile.end_profiling();
	//profile.output();

	profile.start_profiling();

	while (to_simulate_time < simulation_end_time) {

		SupplySimulationPreVehiclePassing<<<segment_blocks, segment_threads_in_a_block, 0, stream_gpu_supply>>>(gpu_data, to_simulate_time, kSegmentSize, parameter_setting_on_gpu, vpool_gpu);

		SupplySimulationVehiclePassingSynch<<<node_blocks, node_threads_in_a_block, 0, stream_gpu_supply>>>(gpu_data, to_simulate_time, kNodeSize, parameter_setting_on_gpu, vpool_gpu);

		to_simulate_time += simulation_time_step;
	}
	profile.end_profiling();
	profile.output();
	return true;
}


/**
 * Minor Functions
 */
bool CopySimulatedResultsToCPU(int time_step) {
	int index = TimestepToArrayIndex(time_step);
	SimulationResults* one = new SimulationResults();

	cudaMemcpy(one->flow, gpu_data->seg_pool.flow, sizeof(float) * kSegmentSize, cudaMemcpyDeviceToHost);
	cudaMemcpy(one->density, gpu_data->seg_pool.density, sizeof(float) * kSegmentSize, cudaMemcpyDeviceToHost);
	cudaMemcpy(one->speed, gpu_data->seg_pool.speed, sizeof(float) * kSegmentSize, cudaMemcpyDeviceToHost);
	cudaMemcpy(one->queue_length, gpu_data->seg_pool.queue_length, sizeof(float) * kSegmentSize, cudaMemcpyDeviceToHost);
	cudaMemcpy(one->counts, gpu_data->seg_pool.veh_counts, sizeof(int) * kSegmentSize, cudaMemcpyDeviceToHost);
	simulation_results_pool[index] = one;

	return true;
}

bool CopyBufferSimulatedResultsToCPU(int time_step) {
	cudaMemcpyAsync(one_buffer, simulation_results_buffer_on_gpu, sizeof(SimulationResults) * kGPUToCPUSimulationResultsCopyBufferSize, cudaMemcpyDeviceToHost, stream_gpu_io);

	for (int i = 0; i < kGPUToCPUSimulationResultsCopyBufferSize; i++) {
		int time_index = time_step - (kGPUToCPUSimulationResultsCopyBufferSize - 1) + i;
		simulation_results_pool[time_index] = &one_buffer[i];
	}

	return true;
}

bool OutputSimulatedResults(int time_step) {
	//output every a minute
	//if(time_step % 60 != 0) return true;

	if (simulation_results_pool.find(time_step) == simulation_results_pool.end()) {
		std::cerr << "System Error, Try to output time " << time_step << ", while it is not ready!" << std::endl;
		return false;
	}

	int index = time_step;
	SimulationResults* one = simulation_results_pool[index];
	assert(one != NULL);

	if(time_step>=3599){
	for (int i = 0; i < kNodeSize; i++) {
		if(one->states[i]>0){
			std::cout<<"VIRTUAL_NODES:"<<i<<'\n';
		}
//		int lane_ID = i;
//		int lane_Index = lane_ID;
//		if(one->counts[lane_Index]>0)
//			std::cout << time_step << ":Segment:" << lane_ID << ":(" << one->counts[lane_Index] << ":" << one->flow[lane_Index] << ":" << one->density[lane_Index]
////				<< ":" << gpu_data->lane_pool.speed[i] << ":" << gpu_data->lane_pool.queue_length[i] << ":" << gpu_data->lane_pool.empty_space[i] << ")" << endl;
//				<< ":" << one->speed[lane_Index] << ":" << one->queue_length[lane_Index] << ")" << endl;
	}
	}
	return true;
}

bool OutputBufferedSimulatedResults(int time_step) {
	//std::cout << "OutputBufferedSimulatedResults AT time " << time_step << std::endl;

	for (int i = 0; i < kGPUToCPUSimulationResultsCopyBufferSize; i++) {
		OutputSimulatedResults(time_step + i);
	}

	return true;
}
