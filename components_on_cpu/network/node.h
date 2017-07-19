/*
 * Node.h
 *
 *  Created on: Jan 1, 2014
 *      Author: xuyan
 */

#ifndef NODE_H_
#define NODE_H_

#include "../util/shared_cpu_include.h"

//forward declaration
class Link;

class Node {
public:
	int node_id;
	int up_seg_start_index;
	int up_seg_end_index;
	bool vnode;
	bool enode;
};

#endif /* NODE_H_ */
