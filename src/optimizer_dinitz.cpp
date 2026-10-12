

// STDLib dependencies
#include <sstream>    // ostringstream
#include <queue>      // queue
#include <iostream>   // cin, cout, endl
#include <iomanip>    // setfill, setw

// Dependencies from this package
#include "kcmc_instance.h"  // KCMC Instance class headers


/** DINITZ DKOV
 * Use the dinitz algorithm to get sensors enough for m-connectivity, and then add sensors to also achieve k-coverage.
 */
int KCMC_Instance::dinitz_dkov(const int k, const int m, std::unordered_set<int> &inactive_sensors, std::unordered_set<int> &solution) {

    // Local buffers
    int num_paths, p, missing_coverage, pre_k_cov_sensors;
    std::unordered_set<int> set_used_sensors, poi_covering_sensors, pois_missing_coverage;

    // We run the greedy M-connectivity normally
    solution.clear();
    num_paths = this->m_connectivity(m, inactive_sensors, solution);
    if (num_paths >= LARGE_NUMBER) {
        throw std::runtime_error("Failed to find " + std::to_string(m) + " paths for POI " + std::to_string((num_paths/LARGE_NUMBER)-1) + " (only found " + std::to_string(num_paths%LARGE_NUMBER) + ")");
    }

    // Run the ADD-KCOV procedure to ensure k-coverage
    pre_k_cov_sensors = ((int)(solution.size()));
    this->add_kcov(k, solution);
    return pre_k_cov_sensors;
}
