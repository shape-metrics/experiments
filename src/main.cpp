#include <atomic>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <thread>
#include <unordered_set>
#include <vector>

#include <domus/core/csv.hpp>
#include <domus/core/graph/file_loader.hpp>
#include <domus/core/graph/graph.hpp>
#include <domus/core/utils.hpp>
#include <domus/orthogonal/drawing.hpp>
#include <domus/orthogonal/drawing_stats.hpp>

#include "ogdf-drawer.hpp"

std::unordered_set<std::string> graphs_already_in_csv;

using namespace std::chrono;
using namespace std::filesystem;

void save_stats(
    std::ofstream& csv_file, const OrthogonalStats& stats, double ogdf_time, std::string graph_name
) {
    csv_file << graph_name << "," << stats.crossings << "," << stats.bends << "," << stats.area
             << "," << stats.total_edge_length << "," << stats.max_edge_length << ","
             << stats.max_bends_per_edge << "," << stats.edge_length_stddev << ","
             << stats.bends_stddev << "," << ogdf_time << std::endl;
}

void compute_stats_ogdf(
    std::string folder_path, std::ofstream& csv_file, path ogdf_svgs_folder, path grid_svg_folder
) {
    auto txt_files = collect_txt_files(folder_path);
    std::atomic<int> number_of_processed_graphs{0};
    std::mutex input_output_lock;
    std::atomic<int> index{0};
    unsigned num_threads = std::thread::hardware_concurrency();
    std::vector<std::thread> threads;
    for (unsigned i = 0; i < num_threads; ++i) {
        threads.emplace_back([&]() {
            while (true) {
                int current = index.fetch_add(1, std::memory_order_relaxed);
                if (current >= txt_files.size())
                    break;
                std::string entry_path = txt_files[current];
                std::string graph_filename = path(entry_path).stem().string();
                int current_number =
                    number_of_processed_graphs.fetch_add(1, std::memory_order_relaxed);
                if (graphs_already_in_csv.contains(graph_filename))
                    continue;
                UndirectedGraph graph = load_graph_from_txt_file(entry_path);
                {
                    std::lock_guard<std::mutex> lock(input_output_lock);
                    std::cout << "Processing graph #" << current_number << " - " << graph_filename
                              << std::endl;
                }
                auto [drawing, time, svg_string] = make_orthogonal_drawing_ogdf(graph);
                std::string ogdf_svg_filename = ogdf_svgs_folder / (graph_filename + ".svg");
                std::ofstream svgFile(ogdf_svg_filename);
                svgFile << svg_string;
                OrthogonalStats stats = compute_all_orthogonal_stats(drawing);
                path svg_output_path_prova = path(grid_svg_folder) / (graph_filename + ".svg");
                make_svg(drawing.augmented_graph, drawing.attributes, svg_output_path_prova);
                {
                    std::lock_guard<std::mutex> lock(input_output_lock);
                    save_stats(csv_file, stats, time, graph_filename);
                }
            }
        });
    }
    for (auto& t : threads)
        if (t.joinable())
            t.join();
    std::cout << "All graphs processed.\nThreads used: " << num_threads
              << "\nTotal graphs: " << number_of_processed_graphs.load() << std::endl;
}

void initialize_csv_file(std::ofstream& result_file) {
    if (!result_file.is_open())
        throw std::runtime_error("Error: Could not open result file");
    result_file << "graph_name,crossings,bends,area,total_edge_length,"
                << "max_edge_length,max_bends_per_edge,edge_length_stddev,"
                << "bends_stddev,time" << std::endl;
}

void run_stats(std::string folder_path) {
    std::cout << "Running stats ogdf..." << std::endl;
    std::string csv_filename = "test_results.csv";
    std::ofstream csv_file;
    if (std::filesystem::exists(csv_filename)) {
        std::cout << "File " << csv_filename << " already exists.\n"
                  << "What do you want to do?\n"
                  << "1. Overwrite the file\n"
                  << "2. Append to the file\n"
                  << "3. Abort\n"
                  << "Please enter your choice (1/2/3): ";
        int choice;
        std::cin >> choice;
        if (choice == 1) {
            std::filesystem::remove(csv_filename);
            csv_file.open(csv_filename);
            initialize_csv_file(csv_file);
        } else if (choice == 2) {
            CSVData csv_data = parse_csv(csv_filename);
            for (const auto& row : csv_data.rows)
                if (row.size() > 0)
                    graphs_already_in_csv.insert(row[0]);
            csv_file.open(csv_filename, std::ios_base::app);
        } else {
            std::cout << "Aborting." << std::endl;
            return;
        }
    } else {
        csv_file.open(csv_filename);
        initialize_csv_file(csv_file);
    }
    path ogdf_svgs_folder("output-svgs-ogdf");
    if (!std::filesystem::exists(ogdf_svgs_folder))
        if (!std::filesystem::create_directories(ogdf_svgs_folder)) {
            std::cerr << "Error: Could not create directory " << ogdf_svgs_folder << std::endl;
            return;
        }
    path grid_svg_folder("output-svgs-grid");
    if (!std::filesystem::exists(grid_svg_folder))
        if (!std::filesystem::create_directories(grid_svg_folder)) {
            std::cerr << "Error: Could not create directory " << grid_svg_folder << std::endl;
            return;
        }
    compute_stats_ogdf(folder_path, csv_file, ogdf_svgs_folder, grid_svg_folder);
    std::cout << std::endl;
    csv_file.close();
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "Insert graphs folder path" << std::endl;
        return 1;
    }
    std::string folder_path = argv[1];
    run_stats(folder_path);
    return 0;
}
