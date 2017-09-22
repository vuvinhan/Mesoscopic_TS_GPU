/*
 * shared_cpu_include.h
 *
 *  Created on: Jan 1, 2014
 *      Author: xuyan
 */

#ifndef SHARED_GPU_DATA_H_
#define SHARED_GPU_DATA_H_

//for small network
//const int kLaneSize = 220;
//const int kNodeSize = 121;

//for large network
const int kLaneSize = 9437;
const int kSegmentSize = 3386;
const int kNodeSize = 3186;
const int kLinkSize = 1049;

const int kStartTimeSteps = 23400;
const int kEndTimeSteps = 28800;

const int kUnitTimeStep = 1; //sec
const int kTotalTimeSteps = (kEndTimeSteps - kStartTimeSteps) / kUnitTimeStep;

const int kQueueLengthHistory = 4;

const int kMaxLaneDownstream = 2;
const int kMaxLaneUpstream = 2;
const int kMaxSegmentInputCapacityPerTimeStep = 5;

//Length Related
//const int kRoadLength = 1000; //meter
const int kVehicleLength = 5; //meter
//const int kMaxVehiclePerLane = kRoadLength / kVehicleLength;
//const int kVehicleMaxLoadingOneTime = 2;

//Speed Related
const float kAlpha = 1;
const float kBeta = 0.5;
const float kMaxDensity = 1.0; //vehicle on road
const float kMinDensity = 3.0 * kVehicleLength / 1000; //vehicle on road

const int kMaxSpeed = 30;
const int kMinSpeed = 1;

const int kMaxRouteLength = 767;
const int kGPUToCPUSimulationResultsCopyBufferSize = 1;//100;

//additional buffer space
const int kTotalVehicleSpace = 307683;
const int kTotalBufferVehicleSpace = 47185;

const int kNumPaths = 14476;
const int kNumTTInfo = 360;
const int kTTInterval = 60;
const int kNumOD = 4165;
const int kMaxRouteLengthLink = 144;
const float kFreeFlowSpeed = 42.4688; //m/s
const int kMaxNumVehPerInterval = 1312; //re-computer for different demand

float kMaxUtil = log(3.40282347e+38);
float kMaxFloat = 3.40282347e+38;
float kBetaT = -0.0108879;

#endif /* SHARED_GPU_DATA_H_ */
