#ifndef MY_SHAPE_METRICS_EXPERIMENTS_HPP
#define MY_SHAPE_METRICS_EXPERIMENTS_HPP

#include <filesystem>
#include <fstream>
#include <string>
#include <utility>

#include <domus/orthogonal/drawing_builder.hpp>

#include "experiments.hpp"

class ShapeMetricsExperiments : public Experiments<ShapeMetricsDrawing> {
    std::ofstream csv_stats_file_m;

  public:
    ShapeMetricsExperiments(
        std::filesystem::path graphs_folder_path,
        bool need_to_initialize_csv,
        std::filesystem::path csv_stats_file_path,
        std::filesystem::path output_svgs_folder,
        std::filesystem::path drawing_results_folder
    );
    std::pair<ShapeMetricsDrawing, double>
    compute_drawing(const UndirectedGraph& graph, std::string graph_name) override;
    void
    save_stats(const ShapeMetricsDrawing& drawing, double time, std::string graph_name) override;
    void save_svg(const ShapeMetricsDrawing& drawing, std::string graph_name) override;
    void save_drawing(const ShapeMetricsDrawing& drawing, std::string graph_name) override;
    void initialize_csv_file() override;
};

#endif