/*
 * OD_Path_Mapping.h
 */

#ifndef OD_PATH_MAPPING_
#define OD_PATH_MAPPING_

#include "../util/shared_cpu_include.h"
#include "../util/configurations_on_cpu.h"

class ODPathMapping {

public:
	int od_id;
	std::vector<int> path_ids;
};

#endif /* OD_PATH_MAPPING_ */
