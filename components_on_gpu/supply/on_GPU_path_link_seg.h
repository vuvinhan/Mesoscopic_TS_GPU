#ifndef GPU_PATH_LINK_SEG_H_
#define GPU_PATH_LINK_SEG_H_

#include "../../components_on_cpu/util/configurations_on_cpu.h"

class GPUPathLinkSeg {
public:
	int path_id;
	int num_links;
	int num_segs;
	int path_links[kMaxRouteLengthLink];
	int path_segs[kMaxRouteLength];
};

#endif /* GPU_PATH_LINK_SEG_H_ */
