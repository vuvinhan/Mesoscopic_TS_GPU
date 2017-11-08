#!/bin/bash
for((i=1; i<=3; i++)); do
	for((j=1; j<=20; j++)); do
		./CPU "./data_inputs_$i/network.dat" "./data_inputs_$i/demand_$j.dat" "./data_inputs_$i/paths.dat" "./data_output_$i/speed_cpu_$j.dat" "./data_output_$i/density_cpu_$j.dat" "./data_output_$i/count_cpu_$j.dat" >> "./data_output_$i/CPU_runtime_$i.dat"
		./VP_1.5hrs_noRB "./data_inputs_$i/network.dat" "./data_inputs_$i/demand_$j.dat" "./data_inputs_$i/paths.dat" >> "./data_output_$i/VP_runtime_$i.dat"
		./Accurate_1.5hrs_noRB "./data_inputs_$i/network.dat" "./data_inputs_$i/demand_$j.dat" "./data_inputs_$i/paths.dat" >> "./data_output_$i/Accurate_runtime_$i.dat"
	done
done
