/*
 * linkTravelTimes.h
 */

#ifndef LINK_TT_
#define LINK_TT_

#include "../util/shared_cpu_include.h"
#include "../util/configurations_on_cpu.h"

class LinkTravelTimes {

public:
	float travelTimes[kNumTTInfo+1]; // reserve 1 for free flow tt

public:
	static bool load_in_all_link_tt(LinkTravelTimes** ttTable, const std::string tt_file_path);
};

bool LinkTravelTimes::load_in_all_link_tt(LinkTravelTimes** ttTable, const std::string tt_file_path) {
// link ID starts from 1
	std::string line;
	std::ifstream myfile(tt_file_path.c_str());

	if (myfile.is_open()) {
		//int max_path_length=0;
		while (getline(myfile, line)) {
			if (line.empty() || line.compare(0, 1, "#") == 0) {
				continue;
			}

			std::vector<std::string> temp_elems;
			network_reading_split(line, ' ', temp_elems);
			int linkId = atoi(temp_elems[0].c_str())-1;

			LinkTravelTimes* linkTT = new LinkTravelTimes();
			//travelTimes[0] is reserved for free flow speed
			linkTT->travelTimes[0] = atof(temp_elems[2].c_str())/kFreeFlowSpeed;
			for(int i = 0; i<kNumTTInfo; i++){
				linkTT->travelTimes[i+1]=atof(temp_elems[i+3].c_str());
			}
			ttTable[linkId] = linkTT;
		}
		myfile.close();
	} else {
		std::cout << "Unable to open travel time file:" << tt_file_path << std::endl;
		return false;
	}
	std::cout << "Travel time table are loaded." << std::endl;
	return true;
}

#endif /* LINK_TT_ */
