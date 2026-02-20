
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
    auto csv_data = parse_csv(csv_stats_file_path);
    if (!csv_data) {
        std::cout << "Could not parse csv file at " << csv_stats_file_path.string() << "\n";
        std::cout << "Error: " << csv_data.error() << "\n";
        exit(1);
    }
    for (const auto& row : csv_data->rows)
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

#define CONFIG_FILE_PATH "config.txt"
#define CSV_STATS_FILE_PATH "csv_stats_file_path"
#define DRAWER_TYPE "drawer_type"
#define GRAPHS_FOLDER "graphs_folder"
#define SVGS_FOLDER_PATH "svgs_folder_path"
#define DRAWING_RESULTS_FOLDER_PATH "drawing_results_folder_path"
#define OGDF_SVGS_FOLDER_PATH "ogdf_svgs_folder_path"

int main() {
    auto config = Config::create("config.txt");
    if (!config) {
        std::cout << "Could not initialize Config: " << config.error() << "\n";
        return 1;
    }

    const auto csv_file_path = config->get(CSV_STATS_FILE_PATH);
    if (!csv_file_path)
        missing_key_in_config_error(CSV_STATS_FILE_PATH);
    const auto drawer_type = config->get(DRAWER_TYPE);
    if (!drawer_type)
        missing_key_in_config_error(DRAWER_TYPE);
    const auto graphs_folder = config->get(GRAPHS_FOLDER);
    if (!graphs_folder)
        missing_key_in_config_error(GRAPHS_FOLDER);
    const auto svgs_folder_path = config->get(SVGS_FOLDER_PATH);
    if (!svgs_folder_path)
        missing_key_in_config_error(SVGS_FOLDER_PATH);
    const auto drawing_results_folder_path = config->get(DRAWING_RESULTS_FOLDER_PATH);
    if (!drawing_results_folder_path)
        missing_key_in_config_error(DRAWING_RESULTS_FOLDER_PATH);

    bool initialize_csv = want_to_re_initialize_csv(*csv_file_path);

    if (drawer_type == "OGDF") {
        const auto ogdf_svgs_folder_path = config->get(OGDF_SVGS_FOLDER_PATH);
        if (!ogdf_svgs_folder_path)
            missing_key_in_config_error(OGDF_SVGS_FOLDER_PATH);
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
