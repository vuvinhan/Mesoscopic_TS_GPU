/*
 * NewVehiclesPerInterval.h
 *
 *  Created on: Dec 31, 2013
 *      Author: xuyan
 */

#ifndef NEWVEHICLESPERINTERVAL_H_
#define NEWVEHICLESPERINTERVAL_H_

#include "../util/configurations_on_cpu.h"

class NewVehiclesPerInterval {

public:
	int new_vehicle_size;
	int veh_ids[kMaxNumVehPerInterval];

};

#endif /* NEWVEHICLESPERINTERVAL_H_ */
