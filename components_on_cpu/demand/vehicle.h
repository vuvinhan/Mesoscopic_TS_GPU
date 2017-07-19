/*
 * Vehicle.h
 *
 *  Created on: Jan 1, 2014
 *      Author: xuyan
 */

#ifndef CPUVEHICLE_H_
#define CPUVEHICLE_H_

#include "../util/shared_cpu_include.h"
#include "../../components_on_gpu/util/on_gpu_configuration.h"

class Vehicle {
public:
	int vehicle_id;
	int orig;
	int dest;
	int path_id;
	int entry_time;

public:
	static bool load_in_all_vehicles(std::vector<Vehicle*>& all_vehicles, const std::string demand_file_path);
};

bool Vehicle::load_in_all_vehicles(std::vector<Vehicle*>& all_vehicles, const std::string demand_file_path) {
	std::string line;
	std::ifstream myfile(demand_file_path.c_str());

	if (myfile.is_open()) {
		while (getline(myfile, line)) {
			if (line.empty() || line.compare(0, 1, "#") == 0) {
				continue;
			}

			std::vector<std::string> temp_elems;
			network_reading_split(line, ':', temp_elems);
			assert(temp_elems.size() == 5);

			Vehicle* one_vec = new Vehicle();
			one_vec->vehicle_id = atoi(temp_elems[0].c_str());
			one_vec->orig = atoi(temp_elems[1].c_str());
			one_vec->dest = atoi(temp_elems[2].c_str());
			one_vec->path_id = atoi(temp_elems[3].c_str());
			one_vec->entry_time = atoi(temp_elems[4].c_str());

			if(one_vec->entry_time<=kEndTimeSteps && one_vec->entry_time>=kStartTimeSteps)
				all_vehicles.push_back(one_vec);
		}
		myfile.close();
	} else {
		std::cout << "Unable to open OD Path file:" << demand_file_path << std::endl;
		return false;
	}

	std::cout << "How many vehicles are loaded?:" << all_vehicles.size() << std::endl;
	return true;
}

#endif /* CPUVEHICLE_H_ */
