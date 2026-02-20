#include "shape_metrics.hpp"

#include <chrono>
#include <iostream>

#include <domus/orthogonal/drawing_stats.hpp>

using namespace std;
using namespace std::filesystem;
using namespace std::chrono;

ShapeMetricsExperiments::ShapeMetricsExperiments(
    path graphs_folder_path,
    bool need_to_initialize_csv,
    path csv_stats_file_path,
    path output_svgs_folder,
    path drawing_results_folder
)
    : Experiments(graphs_folder_path, output_svgs_folder, drawing_results_folder) {
    if (need_to_initialize_csv) {
        csv_stats_file_m.open(csv_stats_file_path);
        initialize_csv_file();
        filesystem::remove_all(output_svgs_folder);
        filesystem::remove_all(drawing_results_folder);
    } else
        csv_stats_file_m.open(csv_stats_file_path, std::ios_base::app);
    create_folder(output_svgs_folder);
    create_folder(drawing_results_folder);
}

pair<ShapeMetricsDrawing, double>
ShapeMetricsExperiments::compute_drawing(const UndirectedGraph& graph, string graph_name) {
    auto start = high_resolution_clock::now();
    auto result = make_orthogonal_drawing(graph);
    auto end = high_resolution_clock::now();
    auto duration = end - start;
    return make_pair(result, duration.count());
}

void ShapeMetricsExperiments::save_stats(
    const ShapeMetricsDrawing& drawing, double time, string graph_name
) {
    lock_guard lock(get_lock());
    const OrthogonalStats stats = compute_all_orthogonal_stats(drawing.drawing);
    csv_stats_file_m << graph_name << "," << stats.crossings << "," << stats.bends << ","
                     << stats.area << "," << stats.total_edge_length << "," << stats.max_edge_length
                     << "," << stats.max_bends_per_edge << "," << stats.edge_length_stddev << ","
                     << stats.bends_stddev << "," << time << "," << drawing.initial_number_of_cycles
                     << "," << drawing.number_of_added_cycles << ","
                     << drawing.number_of_useless_bends << "\n";
}

void ShapeMetricsExperiments::initialize_csv_file() {
    if (!csv_stats_file_m.is_open())
        throw runtime_error("Error: Could not open result file");
    csv_stats_file_m << "graph_name,crossings,bends,area,total_edge_length,max_edge_length,"
                     << "max_bends_per_edge,edge_length_stddev,bends_stddev,time,"
                     << "initial_number_cycles,number_added_cycles,number_useless_bends\n";
}

void ShapeMetricsExperiments::save_svg(const ShapeMetricsDrawing& drawing, string graph_name) {
    path svg_output_path = get_svg_folder_path() / (graph_name + ".svg");
    make_svg(drawing.drawing.augmented_graph, drawing.drawing.attributes, svg_output_path);
}

void ShapeMetricsExperiments::save_drawing(const ShapeMetricsDrawing& drawing, string graph_name) {
    path drawing_results_path = get_drawing_results_folder_path() / (graph_name + ".json");
    save_shape_metrics_drawing_to_file(drawing, drawing_results_path.string());
}