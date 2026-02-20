#ifndef MY_EXPERIMENTS_HPP
#define MY_EXPERIMENTS_HPP

#include <atomic>
#include <expected>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#include <domus/core/graph/file_loader.hpp>
#include <domus/core/graph/graph.hpp>
#include <domus/core/graph/graphs_algorithms.hpp>
#include <domus/core/utils.hpp>

template <typename DrawingT> class Experiments {
  private:
    std::string graphs_folder_path;
    std::unordered_set<std::string> graphs_to_skip;
    std::mutex input_output_lock_m;
    std::filesystem::path output_svgs_folder_m;
    std::filesystem::path drawing_results_folder_m;

  protected:
    void create_folder(std::filesystem::path folder) {
        if (!std::filesystem::exists(folder))
            if (!std::filesystem::create_directories(folder)) {
                std::string error = "Error: Could not create directory " + folder.string() + "\n";
                throw std::runtime_error(error);
            }
    }

  protected:
    Experiments(
        std::filesystem::path graphs_folder_path,
        std::filesystem::path output_svgs_folder,
        std::filesystem::path drawing_results_folder
    )
        : graphs_folder_path(graphs_folder_path), output_svgs_folder_m(output_svgs_folder),
          drawing_results_folder_m(drawing_results_folder) {};
    std::filesystem::path get_svg_folder_path() { return output_svgs_folder_m; }
    std::filesystem::path get_drawing_results_folder_path() { return drawing_results_folder_m; }
    std::mutex& get_lock() { return input_output_lock_m; }
    virtual std::expected<std::pair<DrawingT, double>, std::string>
    compute_drawing(const UndirectedGraph& graph, std::string graph_name) = 0;
    virtual std::expected<void, std::string>
    save_stats(const DrawingT& drawing, double time, std::string graph_name) = 0;
    virtual std::expected<void, std::string>
    save_svg(const DrawingT& drawing, std::string graph_name) = 0;
    virtual std::expected<void, std::string>
    save_drawing(const DrawingT& drawing, std::string graph_name) = 0;
    virtual void initialize_csv_file() = 0;

  public:
    void add_graph_to_skip(std::string graph_name) { graphs_to_skip.insert(graph_name); }
    void run_experiments() {
        int total_fails = 0;
        std::vector<std::string> txt_files = collect_txt_files(graphs_folder_path);
        size_t current_number{0};
        std::atomic<size_t> index{0};
        unsigned num_threads = std::thread::hardware_concurrency();
        std::vector<std::thread> threads;
        for (unsigned i = 0; i < num_threads; ++i) {
            threads.emplace_back([&]() {
                while (true) {
                    size_t current = index.fetch_add(1, std::memory_order_relaxed);
                    if (current >= txt_files.size())
                        break;
                    std::string entry_path = txt_files[current];
                    std::string graph_filename = std::filesystem::path(entry_path).stem().string();
                    if (graphs_to_skip.contains(graph_filename))
                        continue;
                    auto graph = load_graph_from_txt_file(entry_path);
                    if (!graph) {
                        std::string error_msg =
                            "Could not load graph from file " + entry_path + "\n";
                        error_msg += "Error: " + graph.error() + "\n";
                        std::lock_guard<std::mutex> lock(get_lock());
                        total_fails++;
                        std::cerr << error_msg;
                        continue;
                    }
                    if (!is_graph_connected(*graph)) {
                        std::lock_guard<std::mutex> lock(get_lock());
                        total_fails++;
                        std::cerr << "Graph " << graph_filename << " is not connected, skipping.\n";
                        continue;
                    }
                    {
                        std::lock_guard<std::mutex> lock(get_lock());
                        std::cout << "Processing graph #" << current_number++ << " - "
                                  << graph_filename << "\n";
                    }
                    auto result = compute_drawing(*graph, graph_filename);
                    if (!result) {
                        std::string error_msg =
                            "Could not compute drawing for graph " + graph_filename + "\n";
                        error_msg += "Error: " + result.error() + "\n";
                        std::lock_guard<std::mutex> lock(get_lock());
                        total_fails++;
                        std::cerr << error_msg;
                        continue;
                    }
                    DrawingT drawing = result->first;
                    double time = result->second;
                    auto saved_stats = save_stats(drawing, time, graph_filename);
                    if (!saved_stats) {
                        std::string error_msg =
                            "Could not save stats for graph " + graph_filename + "\n";
                        error_msg += "Error: " + saved_stats.error() + "\n";
                        std::cerr << error_msg;
                    }
                    auto saved_svg = save_svg(drawing, graph_filename);
                    if (!saved_svg) {
                        std::string error_msg =
                            "Could not save svg for graph " + graph_filename + "\n";
                        error_msg += "Error: " + saved_svg.error() + "\n";
                        std::cerr << error_msg;
                    }
                    auto saved_drawing = save_drawing(drawing, graph_filename);
                    if (!saved_drawing) {
                        std::string error_msg =
                            "Could not save drawing for graph " + graph_filename + "\n";
                        error_msg += "Error: " + saved_drawing.error() + "\n";
                        std::cerr << error_msg;
                    }
                }
            });
        }
        for (auto& t : threads)
            if (t.joinable())
                t.join();
        std::cout << "All stats computed.\n"
                  << "Total graphs: " << current_number << "\n"
                  << "Total fails: " << total_fails << "\n";
    }
};

#endif