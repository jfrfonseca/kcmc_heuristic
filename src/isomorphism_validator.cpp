/*
 * KCMC Isomorphism Instance Validator
 * Validates whether instances from a file are isomorphic-compatible.
 */


// STDLib dependencies
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <algorithm>
#include <sstream>

// Dependencies from this package
#include "kcmc_instance.h"


void help(int argc, char* const argv[]) {
    std::cout << "RECEIVED LINE (" << argc << "): ";
    for (int i=0; i<argc; i++) {std::cout << argv[i] << " ";}
    std::cout << std::endl;
    std::cout << "Please, use the correct input for the KCMC isomorphism validator:" << std::endl << std::endl;
    std::cout << "./isomorphism_validator <instances_file_path>" << std::endl;
    exit(0);
}


int main(int argc, char* const argv[]) {
    if (argc != 2) {help(argc, argv);}

    // Buffers
    std::string line, normalized_line, group_key, profile, file_name;
    size_t pipe_index;
    int p, s, k, area_s, cov_v, com_r, total_rank, current_rank, rank_counter, current_element;
    long long seed;
    KCMC_Instance* instance = nullptr;
    std::vector<int> poi_rank, sensor_rank, sink_rank;
    std::unordered_map<std::string,std::unordered_map<std::string,std::vector<long long>>> profile_counter;
    bool has_possibly_isomorphic_instances = false;

    // for each line in the file
    std::ifstream instances_file(argv[1]);
    if (!instances_file.is_open()) {throw std::runtime_error("Unable to open input file: " + std::string(argv[1]));}
    while (std::getline(instances_file, line)) {

        // Copy just the part of the line between the prefix and the suddix, before the pipe
        pipe_index = line.find('|');
        if (pipe_index != std::string::npos) {normalized_line = line.substr(5, pipe_index-5);}

        // Replace semicolon with spaces
        std::replace(normalized_line.begin(), normalized_line.end(), ';', ' ');

        // Parse the string as a KCMC instance
        std::istringstream input_string_stream(normalized_line);
        input_string_stream >> p >> s >> k >> area_s >> cov_v >> com_r >> seed;
        instance = new KCMC_Instance(
            p, s, k,
            area_s, cov_v, com_r,
            seed
        );

        // Convert the instance to cx2 format and save it in a file
        file_name = "/data/cx2/kcmc_instance_" + std::to_string(p) + "_" + std::to_string(s) + "_" + std::to_string(k)
                  + "_" + std::to_string(area_s) + "_" + std::to_string(cov_v) + "_" + std::to_string(com_r)
                  + "_" + std::to_string(seed) + ".cx2";
        std::ofstream cx2_file(file_name);
        if (!cx2_file.is_open()) {throw std::runtime_error("Unable to open output file: " + file_name);}
        cx2_file << instance->to_cx2();
        cx2_file.close();

        // Get the rank of the instance
        total_rank = instance->rank(poi_rank, sensor_rank, sink_rank);

        // Count the number of elements of each rank, making the profile of the instance
        profile = std::to_string(total_rank) + "|";
        // For each possible rank in the instance
        for (current_rank=0; current_rank<poi_rank.size(); current_rank++) {
            // Count the number of elements with the current rank
            rank_counter = 0;
            for (current_element=0; current_element<poi_rank.size(); current_element++) {
                if (poi_rank[current_element] == current_rank) {
                    rank_counter++;
                }
            }
            // Add the number of current elements with the current rank to the profile
            if (rank_counter > 0) {
                profile +=  std::to_string(current_rank) + "[" + std::to_string(rank_counter) + "],";
            }
        }

        // Get the group key of the instance
        group_key = std::to_string(p) + " "
                  + std::to_string(s) + " "
                  + std::to_string(k) + " "
                  + std::to_string(area_s) + " "
                  + std::to_string(cov_v) + " "
                  + std::to_string(com_r);

        // If the group key does not exists in the profile counter
        if (profile_counter.find(group_key) == profile_counter.end()) {
            // Add it with an empty profile map
            profile_counter[group_key] = std::unordered_map<std::string,std::vector<long long>>();
        }
        // If the current profile does not exist in the profile map for the group key
        if (profile_counter[group_key].find(profile) == profile_counter[group_key].end()) {
            // Add the current profile with the current seed as the first element
            profile_counter[group_key][profile] = std::vector<long long>{seed};
        } else {
            has_possibly_isomorphic_instances = true;
            // Append the current seed to the profile vector
            profile_counter[group_key][profile].push_back(seed);
        }

        // std::cout << group_key << " " << profile << std::endl;
    }

    if (!has_possibly_isomorphic_instances) {
        std::cout << "No possibly isomorphic instances found." << std::endl;
        return 0;
    }
    // If we got here, we might have isomorphic instances!
    std::cout << "THERE MIGHT BE ISOMORPHIC INSTANCES!" << std::endl;

    // for each group in the profile coutner
    for (const auto& group_pair : profile_counter) {
        const std::string& group_key = group_pair.first;
        const auto& profile_map = group_pair.second;

        // For each profile in the group
        for (const auto& profile_pair : profile_map) {
            const std::string& profile = profile_pair.first;
            const std::vector<long long>& seeds = profile_pair.second;

            // If we have more than one seed for this profile, it indicates possible isomorphic instances
            if (seeds.size() > 1) {
                has_possibly_isomorphic_instances = true;
                std::cout << "Group: " << group_key << ", Seeds: ";
                for (const auto& seed : seeds) {
                    std::cout << seed << " ";
                }
                std::cout << std::endl;
            }
        }
    }

    return 0;
}
