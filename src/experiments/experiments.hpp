#ifndef MY_EXPERIMENTS_HPP
#define MY_EXPERIMENTS_HPP

#include <atomic>
#include <expected>
#include <filesystem>
#include <print>
#include <string>
#include <string_view>
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
    std::filesystem::path graphs_folder_path;
    std::unordered_set<std::string> graphs_to_skip;
    std::filesystem::path output_svgs_folder_m;
    std::filesystem::path drawing_results_folder_m;

  protected:
    std::expected<void, std::string> create_folder(std::filesystem::path folder) {
        if (!std::filesystem::exists(folder))
            if (!std::filesystem::create_directories(folder))
                return std::unexpected("Could not create directory " + folder.string());
        return {};
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
    virtual std::expected<std::pair<DrawingT, double>, std::string>
    compute_drawing(const UndirectedGraph& graph, std::string_view graph_name) = 0;
    virtual void save_stats(const DrawingT& drawing, double time, std::string_view graph_name) = 0;
    virtual std::expected<void, std::string>
    save_svg(const DrawingT& drawing, std::string_view graph_name) = 0;
    virtual std::expected<void, std::string>
    save_drawing(const DrawingT& drawing, std::string_view graph_name) = 0;
    virtual std::expected<void, std::string> initialize_csv_file() = 0;
    int process_graph(const UndirectedGraph& graph, std::string_view graph_filename) {
        auto result = compute_drawing(graph, graph_filename);
        if (!result) {
            std::print(
                "Could not compute drawing for graph {}\n"
                "Error: {}\n",
                graph_filename,
                result.error()
            );
            return 1;
        }
        DrawingT drawing = result->first;
        double time = result->second;
        save_stats(drawing, time, graph_filename);
        auto saved_svg = save_svg(drawing, graph_filename);
        if (!saved_svg) {
            std::print(
                "Could not save svg for graph {}\n"
                "Error: {}",
                graph_filename,
                saved_svg.error()
            );
            return 1;
        }
        auto saved_drawing = save_drawing(drawing, graph_filename);
        if (!saved_drawing) {
            std::print(
                "Could not save drawing for graph {}\n"
                "Error: {}\n",
                graph_filename,
                saved_drawing.error()
            );
            return 1;
        }
        return 0;
    }

  public:
    void add_graph_to_skip(std::string graph_name) { graphs_to_skip.insert(graph_name); }
    void run_experiments() {
        collect_txt_files(graphs_folder_path)
            .and_then(
                [this](std::vector<std::string> txt_files) -> std::expected<void, std::string> {
                    std::atomic<int> total_fails{0};
                    std::atomic<size_t> index{0};
                    std::atomic<size_t> total_graphs{0};
                    unsigned num_threads = std::thread::hardware_concurrency();
                    std::vector<std::thread> threads;
                    for (unsigned i = 0; i < num_threads; ++i) {
                        threads.emplace_back([&]() {
                            while (true) {
                                size_t current = index.fetch_add(1, std::memory_order_relaxed);
                                if (current >= txt_files.size())
                                    break;
                                std::string entry_path = txt_files[current];
                                std::string graph_filename =
                                    std::filesystem::path(entry_path).stem().string();
                                if (graphs_to_skip.contains(graph_filename))
                                    continue;
                                auto graph = load_graph_from_txt_file(entry_path);
                                if (!graph) {
                                    std::print(
                                        "Could not load graph from file {}\n"
                                        "Error: {}\n",
                                        entry_path,
                                        graph.error()
                                    );
                                    total_fails.fetch_add(1, std::memory_order_relaxed);
                                    continue;
                                }
                                if (!is_graph_connected(*graph)) {
                                    std::print("Graph {} is not connected.\n", graph_filename);
                                    total_fails.fetch_add(1, std::memory_order_relaxed);
                                    continue;
                                }
                                std::print("Processing graph# {} - {}\n", current, graph_filename);
                                int failed = process_graph(*graph, graph_filename);
                                if (failed)
                                    total_fails.fetch_add(1, std::memory_order_relaxed);
                                else
                                    total_graphs.fetch_add(1, std::memory_order_relaxed);
                            }
                        });
                    }
                    for (auto& t : threads)
                        if (t.joinable())
                            t.join();
                    std::print(
                        "All stats computed\n"
                        "Total graphs: {}\n"
                        "Total fails: {}\n",
                        total_graphs.load(),
                        total_fails.load()
                    );
                    return {};
                }
            )
            .or_else([](const std::string err) -> std::expected<void, std::string> {
                std::print("Fatal Error: {}\n", err);
                return {};
            })
            .value();
    }
};

#endif