#ifndef GPU_OD_PATH_H_
#define GPU_OD_PATH_H_

#include "../../components_on_cpu/util/configurations_on_cpu.h"

class GPUODPath {
public:
	int od_id[kNumOD];
	int path_start_index[kNumOD];
	int num_paths[kNumOD];
};

#endif /* GPU_OD_PATH_H_ */
