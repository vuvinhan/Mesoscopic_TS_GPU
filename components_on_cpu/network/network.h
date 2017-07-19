/*
 * Network.h
 *
 *  Created on: Jan 1, 2014
 *      Author: xuyan
 */

#ifndef NETWORK_H_
#define NETWORK_H_

#include "../util/shared_cpu_include.h"
#include "../util/configurations_on_cpu.h"
#include "lane.h"
#include "node.h"
#include "segment.h"

class Network {
public:
	int node_size;
	Node** all_nodes;

	int seg_size;
	Segment** all_segs;

	int lane_size;
	Lane** all_lanes;
public:
	static bool load_network(Network& network, const std::string network_file_path);
};

std::vector<std::string> &network_reading_split(const std::string &s, char delim, std::vector<std::string> &elems) {
	std::stringstream ss(s);
	std::string item;
	while (std::getline(ss, item, delim)) {
		elems.push_back(item);
	}
	return elems;
}

bool Network::load_network(Network& network, const std::string network_file_path) {

	//clear the data firstly
	//network.node_mapping.clear();
	//network.link_mapping.clear();

	std::string line;
	std::ifstream myfile(network_file_path.c_str());
	int buff_start_index = 0;

	if (myfile.is_open()) {
		while (getline(myfile, line)) {
			if (line.empty() || line.compare(0, 1, "#") == 0) {
				continue;
			}

			if (line.compare(0, 11, "PARAM_NODE:") == 0) {
				std::vector<std::string> temp_elems;
				network_reading_split(line, ':', temp_elems);

				assert(temp_elems.size() == 2);

				network.node_size = atoi(temp_elems[1].c_str());
				network.all_nodes = new Node*[network.node_size];

			} else if (line.compare(0, 5, "NODE:") == 0) {
				std::vector<std::string> temp_elems;
				network_reading_split(line, ':', temp_elems);

//				std::cout << "temp_elems.size():" << temp_elems.size() << std::endl;
				assert(temp_elems.size() == 4);

				Node* one_node = new Node();
				one_node->node_id = atoi(temp_elems[1].c_str());
				one_node->up_seg_start_index = atoi(temp_elems[2].c_str());
				one_node->up_seg_end_index = atoi(temp_elems[3].c_str());
				one_node->vnode = false;
				one_node->enode = false;

				network.all_nodes[one_node->node_id] = one_node;
				//network.node_mapping[one_node->node_id] = one_node;

			} else if(line.compare(0, 14, "VIRTUAL_NODES:") == 0){
				std::vector<std::string> temp_elems;
				network_reading_split(line, ':', temp_elems);

				assert(temp_elems.size() == 2);
				network.all_nodes[atoi(temp_elems[1].c_str())]->vnode = true;

			}else if(line.compare(0, 15, "NODEDOWNSTREAM:") == 0){
				std::vector<std::string> temp_elems;
				network_reading_split(line, ':', temp_elems);

				assert(temp_elems.size() == 2);
				network.all_nodes[atoi(temp_elems[1].c_str())]->enode = true;

			}else if (line.compare(0, 14, "PARAM_SEGMENT:") == 0) {
				std::vector<std::string> temp_elems;
				network_reading_split(line, ':', temp_elems);

				assert(temp_elems.size() == 2);

				network.seg_size = atoi(temp_elems[1].c_str());
				network.all_segs = new Segment*[network.seg_size];
			}

			else if (line.compare(0, 8, "SEGMENT:") == 0) {
				std::vector<std::string> temp_elems;
				network_reading_split(line, ':', temp_elems);

				assert(temp_elems.size() == 12);

				Segment* one_seg = new Segment();

				one_seg->seg_id = atoi(temp_elems[1].c_str());
				one_seg->lane_start_index = atoi(temp_elems[2].c_str());
				one_seg->num_lanes = atoi(temp_elems[3].c_str());


				one_seg->alpha = atof(temp_elems[4].c_str()); //for the ring buffer which contains both vehicles on lanes and in buffer
				one_seg->beta = atof(temp_elems[5].c_str());
				one_seg->min_density = atof(temp_elems[6].c_str());
				one_seg->max_density = atof(temp_elems[7].c_str());
				one_seg->MIN_speed = atof(temp_elems[8].c_str());
				one_seg->MAX_speed = atof(temp_elems[9].c_str());
				one_seg->capacity = atoi(temp_elems[10].c_str());

				one_seg->length = atof(temp_elems[11].c_str());

				network.all_segs[one_seg->seg_id] = one_seg;
			}
			else if (line.compare(0, 11, "PARAM_LANE:") == 0) {
				std::vector<std::string> temp_elems;
				network_reading_split(line, ':', temp_elems);

				assert(temp_elems.size() == 2);

				network.lane_size = atoi(temp_elems[1].c_str());
				network.all_lanes = new Lane*[network.lane_size];
			}

			else if (line.compare(0, 5, "LANE:") == 0) {
				std::vector<std::string> temp_elems;
				network_reading_split(line, ':', temp_elems);

				assert(temp_elems.size() == 5);

				Lane* one_lane = new Lane();

				one_lane->lane_id = atoi(temp_elems[1].c_str());
				one_lane->seg_id = atoi(temp_elems[2].c_str());
				one_lane->veh_start_index = atoi(temp_elems[3].c_str());
				one_lane->max_vehs = atoi(temp_elems[4].c_str());
				one_lane->buff_veh_start_index = buff_start_index;
				buff_start_index += kMaxSegmentInputCapacityPerTimeStep;

				network.all_lanes[one_lane->lane_id] = one_lane;
			}
		}
		myfile.close();
	} else {
		std::cout << "Unable to open network file:" << network_file_path << std::endl;
	}

	std::cout << "-------------------------------------" << std::endl;
	std::cout << "Network Loaded" << std::endl;
	std::cout << "Nodes:" << network.node_size << std::endl;
	std::cout << "Segments:"<<network.seg_size<<std::endl;
	std::cout << "Lanes:" << network.lane_size << std::endl;
	std::cout << "-------------------------------------" << std::endl;

	return true;
}

#endif /* NETWORK_H_ */
