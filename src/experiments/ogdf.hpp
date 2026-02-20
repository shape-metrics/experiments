#ifndef MY_OGDF_EXPERIMENTS_HPP
#define MY_OGDF_EXPERIMENTS_HPP

#include <filesystem>
#include <fstream>
#include <string>
#include <utility>

#include <domus/orthogonal/drawing.hpp>

#include "experiments.hpp"

class OgdfExperiments : public Experiments<OrthogonalDrawing> {
  private:
    std::filesystem::path output_ogdf_svgs_folder_m;
    std::ofstream csv_stats_file_m;

  public:
    OgdfExperiments(
        std::filesystem::path graphs_folder_path,
        bool need_to_initialize_csv,
        std::filesystem::path csv_stats_file,
        std::filesystem::path output_ogdf_svgs_folder,
        std::filesystem::path output_grid_svgs_folder,
        std::filesystem::path drawing_results_folder
    );
    std::pair<OrthogonalDrawing, double>
    compute_drawing(const UndirectedGraph& graph, std::string graph_name) override;
    void save_stats(const OrthogonalDrawing& drawing, double time, std::string graph_name) override;
    void save_svg(const OrthogonalDrawing& drawing, std::string graph_name) override;
    void save_drawing(const OrthogonalDrawing& drawing, std::string graph_name) override;
    void initialize_csv_file() override;
};

#endif