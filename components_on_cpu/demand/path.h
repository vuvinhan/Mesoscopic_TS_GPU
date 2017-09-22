/*
 * path.h
 *
 *  Created on: Jan 1, 2014
 *      Author: xuyan
 */

#ifndef OD_PATH_H_
#define OD_PATH_H_

#include "../util/shared_cpu_include.h"
#include "../util/configurations_on_cpu.h"
#include "od_path_mapping.h"
#include "od_mapping.h"

class Path {

public:
	int path_id;
	std::vector<int> seg_ids;
	std::vector<int> link_ids;

public:
	static bool load_in_all_Paths(std::vector<Path*>& all_od_pair_paths,
			ODMapping** od_pairs,
			std::vector<ODPathMapping*>& ODPathMap, const std::string od_path_file_path, const std::string path_link_file_path);

	//static bool load_in_all_Path_links(std::vector<Path*>& all_od_pair_paths,
	//			const std::string path_link_file_path);
};

bool Path::load_in_all_Paths(std::vector<Path*>& all_paths,
		ODMapping** all_od_pairs,
		std::vector<ODPathMapping*>& ODPathMap, const std::string od_path_file_path, const std::string path_link_file_path) {
	//very important, ID starts from 0
//	int od_path_id = 0;

	std::string line;
	std::ifstream myfile(od_path_file_path.c_str());

	std::string line_l;
	std::ifstream myfile_l(path_link_file_path.c_str());

	int curOrig = -1;
	int curDest = -1;
	ODMapping* od_pair;
	ODPathMapping* one_mapping;
	int od_id = 0;

	if (myfile.is_open() && myfile_l.is_open()) {
		//int max_path_length=0;
		while (!myfile.eof() && !myfile_l.eof()) {
			getline(myfile, line);

			if (line.empty() || line.compare(0, 1, "#") == 0) {
				continue;
			}

			std::vector<std::string> temp_elems;
			network_reading_split(line, ':', temp_elems);
			assert(temp_elems.size() == 4);

			//load in path
			Path* one_path = new Path();
			one_path->path_id = atoi(temp_elems[0].c_str());

			//load in mapping
			if(curOrig != atoi(temp_elems[1].c_str()) || curDest != atoi(temp_elems[2].c_str())){
				//insert the current mapping path-OD
				if(curOrig != -1){
					ODPathMap.push_back(one_mapping);
				}

				curOrig = atoi(temp_elems[1].c_str());
				curDest = atoi(temp_elems[2].c_str());

				//insert a new OD pair
				od_pair = new ODMapping();
				od_pair->orig = curOrig;
				od_pair->dest = curDest;
				all_od_pairs[od_id] = od_pair;

				//create a new OD-path-mapping
				one_mapping = new ODPathMapping();
				one_mapping->od_id = od_id;
				one_mapping->path_ids.push_back(one_path->path_id);

				od_id++;

			}else{
				one_mapping->path_ids.push_back(one_path->path_id);
			}

			getline(myfile_l, line_l);
			std::vector<std::string> temp_elems3;
			network_reading_split(temp_elems[3], ',', temp_elems3);

			for (int ii = 0; ii < temp_elems3.size(); ii++) {
				int lg_id = atoi(temp_elems3[ii].c_str());
				one_path->seg_ids.push_back(lg_id);
			}


			std::vector<std::string> temp_elems_l;
			network_reading_split(line_l, ':', temp_elems_l);
			assert(temp_elems_l.size() == 2);

			//load in path
			int path_id = atoi(temp_elems_l[0].c_str());
			assert(path_id == one_path->path_id);

			std::vector<std::string> temp_elems1_l;
			network_reading_split(temp_elems_l[1], ',', temp_elems1_l);

			for (int ii = 0; ii < temp_elems1_l.size(); ii++) {
				int link_id = atoi(temp_elems1_l[ii].c_str());
				//std::cout<<link_id<<"\n";
				one_path->link_ids.push_back(link_id);
				//std::cout<<"abc 2"<<"\n";
			}

			all_paths.push_back(one_path);
		}
		ODPathMap.push_back(one_mapping);
		//std::cout<<"max path length: "<<max_path_length<<std::endl;
		myfile.close();
		myfile_l.close();
	} else {
		std::cout << "Unable to open OD Path file:" << od_path_file_path << std::endl;
		return false;
	}
	std::cout << "How many OD pair routes are loaded?:" << all_paths.size() << std::endl;
	return true;
}

//bool Path::load_in_all_Path_links(std::vector<Path*>& all_paths,
//		const std::string path_link_file_path) {
//
//	std::string line;
//	std::ifstream myfile(path_link_file_path.c_str());
//
//	if (myfile.is_open()) {
//		//int max_path_length=0;
//		while (getline(myfile, line)) {
//			if (line.empty() || line.compare(0, 1, "#") == 0) {
//				continue;
//			}
//
//			std::vector<std::string> temp_elems;
//			network_reading_split(line, ':', temp_elems);
//			assert(temp_elems.size() == 2);
//
//			//load in path
//			std::cout<<atoi(temp_elems[0].c_str())<<'\n';
//
//			int path_id = atoi(temp_elems[0].c_str());
//			std::cout<<all_paths[path_id]->path_id<<"\n";
//			std::cout<<"abc 1\n";
//
//			std::vector<std::string> temp_elems1;
//			network_reading_split(temp_elems[1], ',', temp_elems1);
//
//			for (int ii = 0; ii < temp_elems1.size(); ii++) {
//				int link_id = atoi(temp_elems1[ii].c_str());
//				//std::cout<<link_id<<"\n";
//				(all_paths[path_id]->link_ids).push_back(link_id);
//				//std::cout<<"abc 2"<<"\n";
//			}
//		}
//		//std::cout<<"max path length: "<<max_path_length<<std::endl;
//		myfile.close();
//	} else {
//		std::cout << "Unable to open OD Path Link file:" << path_link_file_path << std::endl;
//		return false;
//	}
//	std::cout << "How many path-link mapping are loaded?:" << all_paths.size() << std::endl;
//	return true;
//}

#endif /* OD_PATH_H_ */
