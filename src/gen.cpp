#include "domus/core/graph/graph.hpp"
#include <filesystem>
#include <iostream>
#include <stddef.h>
#include <string>

#include <domus/core/config.hpp>
#include <domus/core/graph/file_loader.hpp>
#include <domus/core/graph/generators.hpp>
#include <domus/core/graph/graphs_algorithms.hpp>

using namespace std;
using namespace std::filesystem;
using namespace domus::graph;
using namespace domus::graph::generators;
using namespace domus::graph::loader;

void generate_graphs(
    path generated_graphs_folder,
    int min_number_of_nodes,
    int max_number_of_nodes,
    int graphs_generated_per_same_nodes_number,
    double min_density,
    double max_density
) {
    filesystem::create_directories(generated_graphs_folder);
    int number_of_generated_graphs = 0;
    for (int number_of_nodes = min_number_of_nodes; number_of_nodes <= max_number_of_nodes;
         ++number_of_nodes) {
        path sub_folder = generated_graphs_folder / std::to_string(number_of_nodes);
        filesystem::create_directories(sub_folder);
        for (int i = 1; i <= graphs_generated_per_same_nodes_number; ++i) {
            const double density = min_density + (max_density - min_density) * i /
                                                     graphs_generated_per_same_nodes_number;
            const int number_of_edges = static_cast<int>(density * number_of_nodes);
            std::cout << "\rGenerating graph with " << number_of_nodes << " nodes and "
                      << number_of_edges << " edges.     ";
            const Graph graph = generate_connected_random_graph_degree_max_4(
                static_cast<size_t>(number_of_nodes),
                static_cast<size_t>(number_of_edges)
            );
            const string graph_name = "graph_" + std::to_string(i) + "_n" +
                                      std::to_string(number_of_nodes) + "_m" +
                                      std::to_string(number_of_edges) + ".txt";
            path filename = sub_folder / graph_name;
            const auto saved = save_graph_to_file(graph, filename);
            if (!saved) {
                string error_msg = "Could not save graph to file " + filename.string() + "\n";
                error_msg += "Error: " + saved.error() + "\n";
                std::cout << error_msg;
                return;
            }
            ++number_of_generated_graphs;
        }
    }
    std::cout << "\nGenerated " << number_of_generated_graphs << " graphs.\n";
}

#define CONFIG_FILE_PATH "config.txt"
#define GENERATED_GRAPHS_FOLDER "generated_graphs_folder"
#define MIN_GRAPH_DENSITY "min_graph_density"
#define MAX_GRAPH_DENSITY "max_graph_density"
#define MIN_NUMBER_OF_NODES "min_number_of_nodes"
#define MAX_NUMBER_OF_NODES "max_number_of_nodes"
#define GRAPHS_GENERATED_PER_SAME_NODES_NUMBER "graphs_generated_per_same_nodes_number"

int main() {
    return domus::Config::create(CONFIG_FILE_PATH)
        .and_then([](unique_ptr<domus::Config> config) -> expected<int, string> {
            const auto generated_graphs_folder = config->get(GENERATED_GRAPHS_FOLDER);
            const auto min_density = config->get(MIN_GRAPH_DENSITY);
            const auto max_density = config->get(MAX_GRAPH_DENSITY);
            const auto min_number_of_nodes = config->get(MIN_NUMBER_OF_NODES);
            const auto max_number_of_nodes = config->get(MAX_NUMBER_OF_NODES);
            const auto graphs_generated_per_same_nodes_number =
                config->get(GRAPHS_GENERATED_PER_SAME_NODES_NUMBER);
            if (!generated_graphs_folder || !min_density || !max_density || !min_number_of_nodes ||
                !max_number_of_nodes || !graphs_generated_per_same_nodes_number)
                return unexpected("One or more required config keys are missing.");
            if (filesystem::exists(*generated_graphs_folder)) {
                std::cout << "Folder " << *generated_graphs_folder << " already exists.\n";
                std::cout << "Do you want to delete it? (y/n): ";
                char answer;
                std::cin >> answer;
                if (answer == 'y' || answer == 'Y') {
                    filesystem::remove_all(*generated_graphs_folder);
                } else {
                    std::cout << "Folder not deleted.\n";
                    return 0;
                }
            }
            generate_graphs(
                *generated_graphs_folder,
                std::stoi(*min_number_of_nodes),
                std::stoi(*max_number_of_nodes),
                std::stoi(*graphs_generated_per_same_nodes_number),
                std::stod(*min_density),
                std::stod(*max_density)
            );
            return 0;
        })
        .or_else([](const string error) {
            std::cerr << "Fatal Error: " << error << "\n";
            return expected<int, string>(1);
        })
        .value();
}