#ifndef MY_SHAPE_METRICS_EXPERIMENTS_HPP
#define MY_SHAPE_METRICS_EXPERIMENTS_HPP

#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <utility>

#include <domus/orthogonal/drawing_builder.hpp>

#include "experiments.hpp"

class ShapeMetricsExperiments : public Experiments<domus::orthogonal::ShapeMetricsDrawing> {
    std::ofstream csv_stats_file_m;

  public:
    ShapeMetricsExperiments(
        std::filesystem::path graphs_folder_path,
        bool need_to_initialize_csv,
        std::filesystem::path csv_stats_file_path,
        std::filesystem::path output_svgs_folder,
        std::filesystem::path drawing_results_folder
    );
    std::expected<std::pair<domus::orthogonal::ShapeMetricsDrawing, double>, std::string>
    compute_drawing(const domus::graph::Graph& graph, std::string_view graph_name) override;
    void save_stats(
        const domus::orthogonal::ShapeMetricsDrawing& drawing,
        double time,
        std::string_view graph_name
    ) override;
    std::expected<void, std::string> save_svg(
        const domus::orthogonal::ShapeMetricsDrawing& drawing, std::string_view graph_name
    ) override;
    std::expected<void, std::string> save_drawing(
        const domus::orthogonal::ShapeMetricsDrawing& drawing, std::string_view graph_name
    ) override;
    std::expected<void, std::string> initialize_csv_file() override;
};

#endif