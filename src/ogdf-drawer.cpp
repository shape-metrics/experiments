#include "ogdf-drawer.hpp"

#include <ogdf/basic/Graph.h>
#include <ogdf/basic/GraphAttributes.h>
#include <ogdf/basic/GraphList.h>
#include <ogdf/basic/LayoutStatistics.h>
#include <ogdf/fileformats/GraphIO.h>
#include <ogdf/orthogonal/OrthoLayout.h>
#include <ogdf/planarity/EmbedderMinDepthMaxFaceLayers.h>
#include <ogdf/planarity/PlanarSubgraphFast.h>
#include <ogdf/planarity/PlanarizationLayout.h>
#include <ogdf/planarity/RemoveReinsertType.h>
#include <ogdf/planarity/SubgraphPlanarizer.h>
#include <ogdf/planarity/VariableEmbeddingInserter.h>

#include <cassert>
#include <chrono>
#include <cmath>
#include <domus/core/utils.hpp>
#include <domus/orthogonal/equivalence_classes.hpp>
#include <domus/orthogonal/shape/shape.hpp>
#include <functional>
#include <map>
#include <string>

constexpr double grid = 100.0;

int snap_coordinate(double v) {
    return static_cast<int>(std::round(v * grid));
}

Shape compute_shape(
    const UndirectedGraph& augmented_graph,
    const std::unordered_map<int, std::pair<int, int>>& id_to_ogdf_positions) {
    Shape shape;
    for (int from_id : augmented_graph.get_nodes_ids()) {
        for (int to_id : augmented_graph.get_neighbors_of_node(from_id)) {
            if (from_id > to_id)
                continue;
            int x_from = id_to_ogdf_positions.at(from_id).first;
            int y_from = id_to_ogdf_positions.at(from_id).second;
            int x_to = id_to_ogdf_positions.at(to_id).first;
            int y_to = id_to_ogdf_positions.at(to_id).second;

            int x_offset = std::abs(x_from - x_to);
            int y_offset = std::abs(y_from - y_to);
            if (x_offset > y_offset) {  // edge is horizontal
                if (x_from < x_to) {
                    shape.set_direction(from_id, to_id, Direction::RIGHT);
                    shape.set_direction(to_id, from_id, Direction::LEFT);
                } else {
                    shape.set_direction(from_id, to_id, Direction::LEFT);
                    shape.set_direction(to_id, from_id, Direction::RIGHT);
                }
            } else {  // edge is vertical
                if (y_from < y_to) {
                    shape.set_direction(from_id, to_id, Direction::UP);
                    shape.set_direction(to_id, from_id, Direction::DOWN);
                } else {
                    shape.set_direction(from_id, to_id, Direction::DOWN);
                    shape.set_direction(to_id, from_id, Direction::UP);
                }
            }
        }
    }
    return shape;
}

std::tuple<UndirectedGraph, std::unordered_map<int, std::pair<int, int>>>
compute_augmented_graph_and_ogdf_positions(
    const UndirectedGraph& graph,
    const ogdf::GraphAttributes& GA,
    const ogdf::Graph& G,
    std::unordered_map<int, int>& ogdf_index_to_nodeid) {
    UndirectedGraph augmented_graph;
    std::unordered_map<int, std::pair<int, int>> id_to_ogdf_positions;
    for (const int node_id : graph.get_nodes_ids())
        augmented_graph.add_node(node_id);
    for (ogdf::node v : G.nodes) {
        const int x = snap_coordinate(GA.x(v));
        const int y = snap_coordinate(GA.y(v));
        id_to_ogdf_positions[ogdf_index_to_nodeid[v->index()]] = {x, y};
    }
    for (ogdf::edge e : G.edges) {
        if (GA.bends(e).size() <= 2) {  // handling edges without bends
            const int from_id = ogdf_index_to_nodeid[e->source()->index()];
            const int to_id = ogdf_index_to_nodeid[e->target()->index()];
            augmented_graph.add_edge(from_id, to_id);
        } else {  // handling edges with bends
            int from_id = ogdf_index_to_nodeid[e->source()->index()];
            const int to_id = ogdf_index_to_nodeid[e->target()->index()];
            std::vector<ogdf::DPoint> bend_vec;
            for (auto& elem : GA.bends(e))
                bend_vec.push_back(elem);
            for (int j = 1; j < bend_vec.size() - 1; ++j) {
                const int node_id = augmented_graph.add_node();
                augmented_graph.add_edge(from_id, node_id);
                from_id = node_id;
                const int x = snap_coordinate(bend_vec[j].m_x);
                const int y = snap_coordinate(bend_vec[j].m_y);
                id_to_ogdf_positions[node_id] = {x, y};
            }
            augmented_graph.add_edge(from_id, to_id);
        }
    }
    return std::make_tuple(augmented_graph, id_to_ogdf_positions);
}

std::unordered_map<int, int> compute_grid_positions(
    const DirectedGraph& ordering_of_classes,
    const EquivalenceClasses& equivalence_classes,
    const std::unordered_map<int, std::pair<int, int>>& id_to_ogdf_positions,
    std::function<int(int, const std::unordered_map<int, std::pair<int, int>>&)>
        get_position) {
    std::unordered_map<int, int> node_id_to_position;
    int current_position = 0;  // position at this point are 0, 1, 2, ...
    constexpr int THRESHOLD = 4500;
    std::unordered_map<int, int> in_degree;
    for (int class_id : ordering_of_classes.get_nodes_ids()) {
        for (int neighbor_class_id :
             ordering_of_classes.get_out_neighbors_of_node(class_id)) {
            if (!in_degree.contains(neighbor_class_id))
                in_degree[neighbor_class_id] = 0;
            in_degree[neighbor_class_id]++;
        }
    }
    std::unordered_set<int> queue;
    for (int class_id : ordering_of_classes.get_nodes_ids())
        if (in_degree[class_id] == 0)
            queue.insert(class_id);
    while (!queue.empty()) {
        std::unordered_map<int, int> avg_coordinate_of_class;
        for (int class_id : queue) {
            int position_sum = 0;
            for (int node_id : equivalence_classes.get_elems_of_class(class_id))
                position_sum += get_position(node_id, id_to_ogdf_positions);
            avg_coordinate_of_class[class_id] =
                position_sum /
                equivalence_classes.get_elems_of_class(class_id).size();
        }
        std::vector<int> classes(queue.begin(), queue.end());
        std::sort(
            classes.begin(),
            classes.end(),
            [&avg_coordinate_of_class](int a, int b) {
                return avg_coordinate_of_class[a] < avg_coordinate_of_class[b];
            });
        for (int i = 0; i < classes.size(); ++i) {
            if (avg_coordinate_of_class[classes[i]] -
                    avg_coordinate_of_class[classes[0]] <
                THRESHOLD) {
                queue.erase(classes[i]);
                for (int node_id :
                     equivalence_classes.get_elems_of_class(classes[i]))
                    node_id_to_position[node_id] = current_position;
                for (int class_id :
                     ordering_of_classes.get_out_neighbors_of_node(
                         classes[i])) {
                    if (--in_degree[class_id] == 0)
                        queue.insert(class_id);
                }
            }
        }
        ++current_position;
    }
    return node_id_to_position;
}

void make_shifts_overlapping_edges(
    UndirectedGraph& augmented_graph,
    GraphAttributes& attributes,
    Shape& shape,
    const std::unordered_map<int, std::pair<int, int>>& id_to_ogdf_positions) {
    std::map<int, std::vector<int>> x_coor_to_ids;
    std::map<int, std::vector<int>> y_coor_to_ids;
    for (int node_id : augmented_graph.get_nodes_ids()) {
        int x = attributes.get_position_x(node_id);
        int y = attributes.get_position_y(node_id);
        x_coor_to_ids[x].push_back(node_id);
        y_coor_to_ids[y].push_back(node_id);
    }
    // TODOOOOO
}

GraphAttributes compute_graph_attributes(
    const UndirectedGraph& graph,
    Shape& shape,
    UndirectedGraph& augmented_graph,
    std::unordered_map<int, std::pair<int, int>>& id_to_ogdf_positions) {
    auto [equivalence_x, equivalence_y] =
        build_equivalence_classes(shape, augmented_graph);
    auto
        [ordering_x,
         ordering_y,
         ordering_x_edge_to_graph_edge,
         ordering_y_edge_to_graph_edge] =
            equivalence_classes_to_ordering(
                equivalence_x, equivalence_y, augmented_graph, shape);
    auto node_id_to_position_x = compute_grid_positions(
        ordering_x,
        equivalence_x,
        id_to_ogdf_positions,
        [](int id,
           const std::unordered_map<int, std::pair<int, int>>&
               id_to_ogdf_positions) {
            return id_to_ogdf_positions.at(id).first;
        });
    auto node_id_to_position_y = compute_grid_positions(
        ordering_y,
        equivalence_y,
        id_to_ogdf_positions,
        [](int id,
           const std::unordered_map<int, std::pair<int, int>>&
               id_to_ogdf_positions) {
            return id_to_ogdf_positions.at(id).second;
        });
    GraphAttributes attributes;
    attributes.add_attribute(Attribute::NODES_POSITION);
    attributes.add_attribute(Attribute::NODES_COLOR);
    for (int node_id : augmented_graph.get_nodes_ids()) {
        attributes.set_position(
            node_id,
            100 * node_id_to_position_x[node_id],
            100 * node_id_to_position_y[node_id]);
        if (graph.has_node(node_id))
            attributes.set_node_color(node_id, Color::BLACK);
        else
            attributes.set_node_color(node_id, Color::RED);
    }
    make_shifts_overlapping_edges(
        augmented_graph, attributes, shape, id_to_ogdf_positions);
    return attributes;
}

OrthogonalDrawing convert_ogdf_result(
    const ogdf::GraphAttributes& GA,
    const ogdf::Graph& G,
    const UndirectedGraph& graph,
    std::unordered_map<int, int>& ogdf_index_to_nodeid) {
    auto [augmented_graph, id_to_ogdf_positions] =
        compute_augmented_graph_and_ogdf_positions(
            graph, GA, G, ogdf_index_to_nodeid);
    Shape shape = compute_shape(augmented_graph, id_to_ogdf_positions);
    GraphAttributes attributes = compute_graph_attributes(
        graph, shape, augmented_graph, id_to_ogdf_positions);
    return {
        std::move(augmented_graph), std::move(attributes), std::move(shape)};
}

struct OgdfDrawingOutput {
    ogdf::Graph G;
    ogdf::GraphAttributes GA;
    double time;
    std::unordered_map<int, int> ogdf_index_to_nodeid;
};

OgdfDrawingOutput compute_drawing_with_ogdf(const UndirectedGraph& graph);

std::pair<OrthogonalDrawing, double> make_orthogonal_drawing_ogdf(
    const UndirectedGraph& graph,
    const std::string& svg_output_filename) {
    auto [G, GA, elapsed, ogdf_index_to_nodeid] =
        compute_drawing_with_ogdf(graph);
    ogdf::GraphIO::write(GA, svg_output_filename, ogdf::GraphIO::drawSVG);
    OrthogonalDrawing result =
        convert_ogdf_result(GA, G, graph, ogdf_index_to_nodeid);
    return std::make_pair(std::move(result), elapsed);
}

OgdfDrawingOutput compute_drawing_with_ogdf(const UndirectedGraph& graph) {
    ogdf::Graph G;
    ogdf::GraphAttributes GA(
        G,
        ogdf::GraphAttributes::nodeGraphics | ogdf::GraphAttributes::nodeType |
            ogdf::GraphAttributes::edgeGraphics |
            ogdf::GraphAttributes::edgeType | ogdf::GraphAttributes::nodeLabel |
            ogdf::GraphAttributes::nodeStyle |
            ogdf::GraphAttributes::nodeTemplate);
    std::unordered_map<int, ogdf::node> nodeid_to_ogdf_node;
    std::unordered_map<int, int> ogdf_index_to_nodeid;
    for (const int node_id : graph.get_nodes_ids()) {
        nodeid_to_ogdf_node[node_id] = G.newNode(node_id);
        ogdf_index_to_nodeid[nodeid_to_ogdf_node[node_id]->index()] = node_id;
    }
    for (int from_id : graph.get_nodes_ids()) {
        for (int to_id : graph.get_neighbors_of_node(from_id)) {
            if (from_id > to_id)
                continue;
            G.newEdge(nodeid_to_ogdf_node[from_id], nodeid_to_ogdf_node[to_id]);
        }
    }
    for (ogdf::node v : G.nodes)
        GA.label(v) = std::to_string(v->index());

    auto start = std::chrono::high_resolution_clock::now();

    ogdf::PlanarizationLayout pl;
    ogdf::SubgraphPlanarizer* crossMin = new ogdf::SubgraphPlanarizer;
    ogdf::PlanarSubgraphFast<int>* ps = new ogdf::PlanarSubgraphFast<int>;
    ps->runs(100);
    ogdf::VariableEmbeddingInserter* ves = new ogdf::VariableEmbeddingInserter;
    ves->removeReinsert(ogdf::RemoveReinsertType::All);

    crossMin->setSubgraph(ps);
    crossMin->setInserter(ves);
    pl.setCrossMin(crossMin);

    ogdf::EmbedderMinDepthMaxFaceLayers* emb =
        new ogdf::EmbedderMinDepthMaxFaceLayers;
    pl.setEmbedder(emb);

    ogdf::OrthoLayout* ol = new ogdf::OrthoLayout;
    ol->separation(100.0);
    ol->cOverhang(0.0);
    pl.setPlanarLayouter(ol);

    ogdf::setSeed(0);
    pl.call(GA);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    return {G, GA, elapsed.count(), ogdf_index_to_nodeid};
}