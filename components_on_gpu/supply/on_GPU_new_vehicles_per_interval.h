/*
 * OnGPUNewVehiclesPerInterval.h
 *
 *  Created on: Dec 31, 2013
 *      Author: xuyan
 */

#ifndef ONGPUNEWVEHICLESPERINTERVAL_H_
#define ONGPUNEWVEHICLESPERINTERVAL_H_

#include "../../components_on_cpu/util/configurations_on_cpu.h"

class GPUNewVehiclesPerInterval {

public:
	int new_vehicle_size;
	int veh_ids[kMaxNumVehPerInterval];

};

#endif /* ONGPUNEWVEHICLESPERINTERVAL_H_ */
