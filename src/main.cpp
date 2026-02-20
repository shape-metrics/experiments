
#include <filesystem>
#include <iostream>
#include <stddef.h>

#include <domus/config/config.hpp>
#include <domus/core/csv.hpp>
#include <domus/core/graph/file_loader.hpp>
#include <domus/core/graph/graph.hpp>
#include <domus/core/graph/graphs_algorithms.hpp>
#include <domus/core/utils.hpp>

#include "experiments/ogdf.hpp"
#include "experiments/shape_metrics.hpp"

using namespace std;
using namespace std::filesystem;

template <typename E> void add_graphs_to_skip(E& experiment_runner, path csv_stats_file_path) {
    CSVData csv_data = parse_csv(csv_stats_file_path);
    for (const auto& row : csv_data.rows)
        if (!row.empty())
            experiment_runner.add_graph_to_skip(row[0]);
}

bool want_to_re_initialize_csv(path csv_stats_file_path) {
    if (!filesystem::exists(csv_stats_file_path))
        return true;
    std::cout << "File " << csv_stats_file_path.string() << " already exists.\n"
              << "What do you want to do?\n"
              << "1. Overwrite the file\n"
              << "2. Append to the file\n"
              << "3. Abort\n"
              << "Please enter your choice (1/2/3): ";
    int choice;
    std::cin >> choice;
    switch (choice) {
    case 1:
        return true;
    case 2:
        return false;
    }
    std::cout << "Aborting.\n";
    exit(1);
}

int main() {
    Config config("config.txt");
    bool initialize_csv = want_to_re_initialize_csv(config.get("csv_stats_file_path"));
    string drawer_type = config.get("drawer_type");
    if (drawer_type == "OGDF") {
        OgdfExperiments experiment_runner(
            config.get("graphs_folder"),
            initialize_csv,
            config.get("csv_stats_file_path"),
            config.get("ogdf_svgs_folder_path"),
            config.get("svgs_folder_path"),
            config.get("drawing_results_folder_path")
        );
        if (!initialize_csv)
            add_graphs_to_skip(experiment_runner, config.get("csv_stats_file_path"));
        experiment_runner.run_experiments();
    } else if (drawer_type == "shape_metrics") {
        ShapeMetricsExperiments experiment_runner(
            config.get("graphs_folder"),
            initialize_csv,
            config.get("csv_stats_file_path"),
            config.get("svgs_folder_path"),
            config.get("drawing_results_folder_path")
        );
        if (!initialize_csv)
            add_graphs_to_skip(experiment_runner, config.get("csv_stats_file_path"));
        experiment_runner.run_experiments();
    } else {
        std::cout << "Unknown drawer type: " << drawer_type << "\n";
        return 1;
    }
    return 0;
}
