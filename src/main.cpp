
#include <filesystem>
#include <iostream>
#include <stddef.h>

#include <domus/core/config.hpp>
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
    auto csv_data = domus::parse_csv(csv_stats_file_path);
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
    return domus::Config::create("config.txt")
        .and_then([](unique_ptr<domus::Config> config) -> expected<int, string> {
            auto csv_path_opt = config->get(CSV_STATS_FILE_PATH);
            auto drawer_opt = config->get(DRAWER_TYPE);
            auto graphs_opt = config->get(GRAPHS_FOLDER);
            auto svgs_opt = config->get(SVGS_FOLDER_PATH);
            auto results_opt = config->get(DRAWING_RESULTS_FOLDER_PATH);

            if (!csv_path_opt || !drawer_opt || !graphs_opt || !svgs_opt || !results_opt)
                return unexpected("One or more required config keys are missing.");

            const string csv_path = *csv_path_opt;
            const string drawer = *drawer_opt;
            const string graphs = *graphs_opt;
            const string svgs = *svgs_opt;
            const string results = *results_opt;

            bool initialize_csv = want_to_re_initialize_csv(csv_path);

            if (drawer == "OGDF") {
                auto ogdf_svgs_opt = config->get(OGDF_SVGS_FOLDER_PATH);
                if (!ogdf_svgs_opt)
                    return unexpected("Missing OGDF_SVGS_FOLDER_PATH");

                const string ogdf_svgs = *ogdf_svgs_opt;

                OgdfExperiments runner(graphs, initialize_csv, csv_path, ogdf_svgs, svgs, results);

                if (!initialize_csv)
                    add_graphs_to_skip(runner, csv_path);

                runner.run_experiments();
                return 0;
            }

            if (drawer == "shape_metrics") {
                ShapeMetricsExperiments runner(graphs, initialize_csv, csv_path, svgs, results);

                if (!initialize_csv)
                    add_graphs_to_skip(runner, csv_path);

                runner.run_experiments();
                return 0;
            }
            return unexpected("Unknown drawer type: " + drawer);
        })
        .or_else([](const string& err) {
            std::cerr << "Fatal Error: " << err << "\n";
            return expected<int, string>(1);
        })
        .value();
}