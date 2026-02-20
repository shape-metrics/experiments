
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

void missing_key_in_config_error(string key) {
    std::cout << "Missing key in config.txt\n";
    std::cout << "Could not find [" << key << "]\n";
    exit(1);
}

int main() {
    auto config = Config::create("config.txt");
    if (!config) {
        std::cout << "Could not initialize Config: " << config.error() << "\n";
        return 1;
    }

    const auto csv_file_path = config->get("csv_stats_file_path");
    if (!csv_file_path)
        missing_key_in_config_error("csv_stats_file_path");
    const auto drawer_type = config->get("drawer_type");
    if (!drawer_type)
        missing_key_in_config_error("drawer_type");
    const auto graphs_folder = config->get("graphs_folder");
    if (!graphs_folder)
        missing_key_in_config_error("graphs_folder");
    const auto svgs_folder_path = config->get("svgs_folder_path");
    if (!svgs_folder_path)
        missing_key_in_config_error("svgs_folder_path");
    const auto drawing_results_folder_path = config->get("drawing_results_folder_path");
    if (!drawing_results_folder_path)
        missing_key_in_config_error("drawing_results_folder_path");

    bool initialize_csv = want_to_re_initialize_csv(*csv_file_path);

    if (drawer_type == "OGDF") {
        const auto ogdf_svgs_folder_path = config->get("ogdf_svgs_folder_path");
        if (!ogdf_svgs_folder_path)
            missing_key_in_config_error("ogdf_svgs_folder_path");
        OgdfExperiments experiment_runner(
            *graphs_folder,
            initialize_csv,
            *csv_file_path,
            *ogdf_svgs_folder_path,
            *svgs_folder_path,
            *drawing_results_folder_path
        );
        if (!initialize_csv)
            add_graphs_to_skip(experiment_runner, *csv_file_path);
        experiment_runner.run_experiments();
    } else if (drawer_type == "shape_metrics") {
        ShapeMetricsExperiments experiment_runner(
            *graphs_folder,
            initialize_csv,
            *csv_file_path,
            *svgs_folder_path,
            *drawing_results_folder_path
        );
        if (!initialize_csv)
            add_graphs_to_skip(experiment_runner, *csv_file_path);
        experiment_runner.run_experiments();
    } else {
        std::cout << "Unknown drawer type: " << *drawer_type << "\n";
        return 1;
    }
    return 0;
}
