#include "ogdf.hpp"

#include <iostream>
#include <stdexcept>

#include <domus/orthogonal/drawing_stats.hpp>

#include "ogdf-drawer/drawer.hpp"

using namespace std;
using namespace std::filesystem;

OgdfExperiments::OgdfExperiments(
    path graphs_folder_path,
    bool need_to_initialize_csv,
    path csv_stats_file,
    path output_ogdf_svgs_folder,
    path output_grid_svgs_folder,
    path drawing_results_folder
)
    : Experiments(graphs_folder_path, output_grid_svgs_folder, drawing_results_folder),
      output_ogdf_svgs_folder_m(output_ogdf_svgs_folder) {
    if (need_to_initialize_csv) {
        csv_stats_file_m.open(csv_stats_file);
        initialize_csv_file();
        filesystem::remove_all(output_ogdf_svgs_folder);
        filesystem::remove_all(output_grid_svgs_folder);
        filesystem::remove_all(drawing_results_folder);
    } else
        csv_stats_file_m.open(csv_stats_file, std::ios_base::app);
    create_folder(output_ogdf_svgs_folder);
    create_folder(output_grid_svgs_folder);
    create_folder(drawing_results_folder);
}

pair<OrthogonalDrawing, double>
OgdfExperiments::compute_drawing(const UndirectedGraph& graph, string graph_name) {
    auto [drawing, time, svg_string, crossings] = make_orthogonal_drawing_ogdf(graph);
    string ogdf_svg_filename = output_ogdf_svgs_folder_m / (graph_name + ".svg");
    ofstream svgFile(ogdf_svg_filename);
    svgFile << svg_string;
    svgFile.close();
    return make_pair(drawing, time);
}

void OgdfExperiments::save_stats(const OrthogonalDrawing& drawing, double time, string graph_name) {
    lock_guard lock(get_lock());
    const OrthogonalStats stats = compute_all_orthogonal_stats(drawing);
    csv_stats_file_m << graph_name << "," << stats.crossings << "," << stats.bends << ","
                     << stats.area << "," << stats.total_edge_length << "," << stats.max_edge_length
                     << "," << stats.max_bends_per_edge << "," << stats.edge_length_stddev << ","
                     << stats.bends_stddev << "," << time << "\n";
}

void OgdfExperiments::initialize_csv_file() {
    if (!csv_stats_file_m.is_open())
        throw runtime_error("Error: Could not open result file");
    csv_stats_file_m << "graph_name,crossings,bends,area,total_edge_length,max_edge_length,"
                     << "max_bends_per_edge,edge_length_stddev,bends_stddev,time\n";
}

void OgdfExperiments::save_svg(const OrthogonalDrawing& drawing, string graph_name) {
    path svg_output_path = get_svg_folder_path() / (graph_name + ".svg");
    make_svg(drawing.augmented_graph, drawing.attributes, svg_output_path);
}

void OgdfExperiments::save_drawing(const OrthogonalDrawing& drawing, string graph_name) {
    path drawing_results_path = get_drawing_results_folder_path() / (graph_name + ".json");
    save_orthogonal_drawing_to_file(drawing, drawing_results_path.string());
}