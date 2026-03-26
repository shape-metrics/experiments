#include "drawer.hpp"

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
#include <functional>
#include <optional>
#include <sstream>
#include <unordered_set>
#include <utility>

#include <domus/core/utils.hpp>
#include <domus/orthogonal/equivalence_classes.hpp>
#include <domus/orthogonal/shape/shape.hpp>

using namespace std;
using namespace std::chrono;

using namespace domus::graph;
using namespace domus::orthogonal;
using namespace domus::orthogonal::shape;

constexpr double grid = 100.0;

int snap_coordinate(double v) { return static_cast<int>(round(v * grid)); }

Shape compute_shape(
    const Graph& graph, const unordered_map<size_t, pair<int, int>>& id_to_ogdf_positions
) {
    Shape shape;
    for (size_t from_id : graph.get_node_ids()) {
        for (auto [edge_id, to_id] : graph.get_edges(from_id)) {
            if (from_id > to_id)
                continue;
            int x_from = id_to_ogdf_positions.at(from_id).first;
            int y_from = id_to_ogdf_positions.at(from_id).second;
            int x_to = id_to_ogdf_positions.at(to_id).first;
            int y_to = id_to_ogdf_positions.at(to_id).second;

            int x_offset = abs(x_from - x_to);
            int y_offset = abs(y_from - y_to);
            if (x_offset > y_offset) { // edge is horizontal
                if (x_from < x_to)
                    shape.set_direction(graph, edge_id, from_id, to_id, Direction::RIGHT);
                else
                    shape.set_direction(graph, edge_id, from_id, to_id, Direction::LEFT);
            } else { // edge is vertical
                if (y_from < y_to)
                    shape.set_direction(graph, edge_id, from_id, to_id, Direction::UP);
                else
                    shape.set_direction(graph, edge_id, from_id, to_id, Direction::DOWN);
            }
        }
    }
    return shape;
}

tuple<Graph, unordered_map<size_t, pair<int, int>>> compute_augmented_graph_and_ogdf_positions(
    const Graph& graph,
    const ogdf::GraphAttributes& GA,
    const ogdf::Graph& G,
    unordered_map<int, size_t>& ogdf_index_to_nodeid
) {
    Graph augmented_graph;
    unordered_map<size_t, pair<int, int>> id_to_ogdf_positions;
    for (const size_t node_id : graph.get_node_ids())
        augmented_graph.add_node();
    for (ogdf::node v : G.nodes) {
        const int x = snap_coordinate(GA.x(v));
        const int y = snap_coordinate(GA.y(v));
        id_to_ogdf_positions[ogdf_index_to_nodeid[v->index()]] = {x, y};
    }
    for (ogdf::edge e : G.edges) {
        if (GA.bends(e).size() <= 2) { // handling edges without bends
            const int from_id = ogdf_index_to_nodeid[e->source()->index()];
            const int to_id = ogdf_index_to_nodeid[e->target()->index()];
            augmented_graph.add_edge(from_id, to_id);
        } else { // handling edges with bends
            int from_id = ogdf_index_to_nodeid[e->source()->index()];
            const int to_id = ogdf_index_to_nodeid[e->target()->index()];
            vector<ogdf::DPoint> bend_vec;
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
    return make_tuple(augmented_graph, id_to_ogdf_positions);
}

unordered_map<int, int> compute_grid_positions(
    const Graph& ordering_of_classes,
    const EquivalenceClasses& equivalence_classes,
    const unordered_map<size_t, pair<int, int>>& id_to_ogdf_positions,
    function<int(int, const unordered_map<size_t, pair<int, int>>&)> get_position
) {
    unordered_map<int, int> node_id_to_position;
    int current_position = 0; // position at this point are 0, 1, 2, ...
    constexpr int THRESHOLD = 5500;
    unordered_map<int, int> in_degree;
    for (size_t class_id : ordering_of_classes.get_node_ids()) {
        for (size_t neighbor_class_id : ordering_of_classes.get_out_neighbors(class_id)) {
            if (!in_degree.contains(neighbor_class_id))
                in_degree[neighbor_class_id] = 0;
            in_degree[neighbor_class_id]++;
        }
    }
    unordered_set<int> queue;
    for (int class_id : ordering_of_classes.get_node_ids())
        if (in_degree[class_id] == 0)
            queue.insert(class_id);
    while (!queue.empty()) {
        unordered_map<int, int> avg_coordinate_of_class;
        for (int class_id : queue) {
            int position_sum = 0;
            equivalence_classes.for_each_elem_of_class(class_id, [&](size_t node_id) {
                position_sum += get_position(node_id, id_to_ogdf_positions);
            });
            avg_coordinate_of_class[class_id] =
                position_sum / equivalence_classes.number_of_elems_in_class(class_id);
        }
        vector<int> classes(queue.begin(), queue.end());
        sort(classes.begin(), classes.end(), [&avg_coordinate_of_class](int a, int b) {
            return avg_coordinate_of_class[a] < avg_coordinate_of_class[b];
        });
        for (int i = 0; i < classes.size(); ++i) {
            if (avg_coordinate_of_class[classes[i]] - avg_coordinate_of_class[classes[0]] <
                THRESHOLD) {
                queue.erase(classes[i]);

                equivalence_classes.for_each_elem_of_class(classes[i], [&](size_t node_id) {
                    node_id_to_position[node_id] = current_position;
                });

                for (size_t class_id : ordering_of_classes.get_out_neighbors(classes[i]))
                    if (--in_degree[class_id] == 0)
                        queue.insert(class_id);
            }
        }
        ++current_position;
    }
    return node_id_to_position;
}

void make_shifts_overlapping_edges(
    Graph& augmented_graph,
    Attributes& attributes,
    Shape& shape,
    const unordered_map<size_t, pair<int, int>>& id_to_ogdf_positions
) {
    std::vector<size_t> nodes;
    for (size_t node_id : augmented_graph.get_node_ids())
        nodes.push_back(node_id);
    for (size_t node_id : nodes) {
        if (attributes.get_node_color(node_id) == domus::Color::RED)
            continue;
        std::unordered_map<Direction, std::vector<std::pair<int, int>>> direction_to_ids;
        for (auto [edge_id, neighbor_id] : augmented_graph.get_edges(node_id))
            direction_to_ids[shape.get_direction(augmented_graph, edge_id, node_id, neighbor_id)]
                .push_back({edge_id, neighbor_id});
        for (auto& [direction, neighbors_ids] : direction_to_ids) {
            if (neighbors_ids.size() <= 1)
                continue;
            sort(
                neighbors_ids.begin(),
                neighbors_ids.end(),
                [&id_to_ogdf_positions, direction](auto a, auto b) {
                    if (is_horizontal(direction))
                        return id_to_ogdf_positions.at(a.second).second <
                               id_to_ogdf_positions.at(b.second).second;
                    else
                        return id_to_ogdf_positions.at(a.second).first <
                               id_to_ogdf_positions.at(b.second).first;
                }
            );
            std::optional<int> black_node_index;
            for (int i = 0; i < neighbors_ids.size(); ++i)
                if (attributes.get_node_color(neighbors_ids[i].second) == domus::Color::BLACK) {
                    black_node_index = i;
                    break;
                }
            int black_index = black_node_index.value_or(0);
            for (int i = 0; i < neighbors_ids.size(); ++i) {
                if (i == black_index)
                    continue;
                size_t neighbor_id = neighbors_ids[i].second;
                size_t edge_id = neighbors_ids[i].first;
                int shift = 5 * (i - black_index);
                if (attributes.get_node_color(neighbor_id) == domus::Color::BLACK) {
                    int prev_id = node_id;
                    int current_id = neighbor_id;
                    size_t current_edge_id = edge_id;
                    assert(augmented_graph.get_degree_of_node(current_id) <= 2);
                    while (attributes.get_node_color(current_id) == domus::Color::BLACK &&
                           shape.get_direction(
                               augmented_graph,
                               current_edge_id,
                               prev_id,
                               current_id
                           ) == direction) {
                        if (is_horizontal(direction)) {
                            attributes.change_position_y(
                                current_id,
                                attributes.get_position_y(current_id) + shift
                            );
                        } else {
                            attributes.change_position_x(
                                current_id,
                                attributes.get_position_x(current_id) + shift
                            );
                        }
                        if (augmented_graph.get_degree_of_node(current_id) == 1)
                            break;
                        for (auto [next_edge_id, next_id] : augmented_graph.get_edges(current_id)) {
                            if (next_id != prev_id) {
                                prev_id = current_id;
                                current_id = next_id;
                                current_edge_id = next_edge_id;
                                break;
                            }
                        }
                    }
                    continue;
                }
                assert([&]() {
                    auto edge = augmented_graph.get_edge(edge_id);
                    size_t from_id = edge.from_id;
                    size_t to_id = edge.to_id;
                    return (from_id == node_id && to_id == neighbor_id) ||
                           (from_id == neighbor_id && to_id == node_id);
                }());
                shape.remove_direction(edge_id);
                const Subdivision s = augmented_graph.subdivide_edge(edge_id);
                const size_t added_node_id = s.in_between_id;
                assert(augmented_graph.are_neighbors(added_node_id, node_id));
                assert(augmented_graph.are_neighbors(added_node_id, neighbor_id));
                attributes.set_node_color(added_node_id, domus::Color::GREEN);

                const size_t edge_to_neighbor =
                    (s.to_id == neighbor_id) ? s.edge_between_to_id : s.edge_from_between_id;
                const size_t edge_to_node =
                    (s.from_id == node_id) ? s.edge_from_between_id : s.edge_between_to_id;

                shape.set_direction(
                    augmented_graph,
                    edge_to_neighbor,
                    added_node_id,
                    neighbor_id,
                    direction
                );

                if (is_horizontal(direction)) {
                    int added_x = attributes.get_position_x(node_id);
                    int added_y = attributes.get_position_y(node_id) + shift;
                    attributes.set_position(added_node_id, added_x, added_y);
                    if (shift < 0)
                        shape.set_direction(
                            augmented_graph,
                            edge_to_node,
                            node_id,
                            added_node_id,
                            Direction::DOWN
                        );
                    else
                        shape.set_direction(
                            augmented_graph,
                            edge_to_node,
                            node_id,
                            added_node_id,
                            Direction::UP
                        );
                    attributes.change_position_y(neighbor_id, added_y);
                } else {
                    int added_x = attributes.get_position_x(node_id) + shift;
                    int added_y = attributes.get_position_y(node_id);
                    attributes.set_position(added_node_id, added_x, added_y);
                    if (shift < 0)
                        shape.set_direction(
                            augmented_graph,
                            edge_to_node,
                            node_id,
                            added_node_id,
                            Direction::LEFT
                        );
                    else
                        shape.set_direction(
                            augmented_graph,
                            edge_to_node,
                            node_id,
                            added_node_id,
                            Direction::RIGHT
                        );
                    attributes.change_position_x(neighbor_id, added_x);
                }
            }
        }
    }
}

void flip_y_values(const Graph& augmented_graph, Attributes& attributes) {
    int max_y = numeric_limits<int>::min();
    for (size_t node_id : augmented_graph.get_node_ids())
        max_y = max(max_y, attributes.get_position_y(node_id));
    for (size_t node_id : augmented_graph.get_node_ids())
        attributes.change_position_y(node_id, max_y - attributes.get_position_y(node_id));
}

Attributes compute_graph_attributes(
    const Graph& graph,
    Shape& shape,
    Graph& augmented_graph,
    unordered_map<size_t, pair<int, int>>& id_to_ogdf_positions
) {
    auto [equivalence_x, equivalence_y] = EquivalenceClasses::build(shape, augmented_graph);
    auto ordering = Ordering::build(equivalence_x, equivalence_y, augmented_graph, shape);
    auto node_id_to_position_x = compute_grid_positions(
        ordering.get_ordering_x(),
        equivalence_x,
        id_to_ogdf_positions,
        [](int id, const unordered_map<size_t, pair<int, int>>& id_to_ogdf_positions) {
            return id_to_ogdf_positions.at(id).first;
        }
    );
    auto node_id_to_position_y = compute_grid_positions(
        ordering.get_ordering_y(),
        equivalence_y,
        id_to_ogdf_positions,
        [](int id, const unordered_map<size_t, pair<int, int>>& id_to_ogdf_positions) {
            return id_to_ogdf_positions.at(id).second;
        }
    );
    Attributes attributes;
    attributes.add_attribute(Attribute::NODES_POSITION);
    attributes.add_attribute(Attribute::NODES_COLOR);
    for (int node_id : augmented_graph.get_node_ids()) {
        attributes.set_position(
            node_id,
            100 * node_id_to_position_x[node_id],
            100 * node_id_to_position_y[node_id]
        );
        if (graph.has_node(node_id))
            attributes.set_node_color(node_id, domus::Color::BLACK);
        else
            attributes.set_node_color(node_id, domus::Color::RED);
    }
    make_shifts_overlapping_edges(augmented_graph, attributes, shape, id_to_ogdf_positions);
    flip_y_values(augmented_graph, attributes);
    return attributes;
}

OrthogonalDrawing convert_ogdf_result(
    const ogdf::GraphAttributes& GA,
    const ogdf::Graph& G,
    const Graph& graph,
    unordered_map<int, size_t>& ogdf_index_to_nodeid
) {
    auto [augmented_graph, id_to_ogdf_positions] =
        compute_augmented_graph_and_ogdf_positions(graph, GA, G, ogdf_index_to_nodeid);
    Shape shape = compute_shape(augmented_graph, id_to_ogdf_positions);
    Attributes attributes =
        compute_graph_attributes(graph, shape, augmented_graph, id_to_ogdf_positions);
    return {std::move(augmented_graph), std::move(attributes), std::move(shape)};
}

tuple<OrthogonalDrawing, double, string, int> make_orthogonal_drawing_ogdf(const Graph& graph) {
    ogdf::Graph G;
    ogdf::GraphAttributes GA(
        G,
        ogdf::GraphAttributes::nodeGraphics | ogdf::GraphAttributes::nodeType |
            ogdf::GraphAttributes::edgeGraphics | ogdf::GraphAttributes::edgeType |
            ogdf::GraphAttributes::nodeLabel | ogdf::GraphAttributes::nodeStyle |
            ogdf::GraphAttributes::nodeTemplate
    );
    unordered_map<int, ogdf::node> nodeid_to_ogdf_node;
    unordered_map<int, size_t> ogdf_index_to_nodeid;
    for (const size_t node_id : graph.get_node_ids()) {
        nodeid_to_ogdf_node[node_id] = G.newNode(node_id);
        ogdf_index_to_nodeid[nodeid_to_ogdf_node[node_id]->index()] = node_id;
    }
    for (size_t from_id : graph.get_node_ids())
        for (size_t to_id : graph.get_out_neighbors(from_id))
            G.newEdge(nodeid_to_ogdf_node[from_id], nodeid_to_ogdf_node[to_id]);
    for (ogdf::node v : G.nodes)
        GA.label(v) = to_string(v->index());

    auto start = high_resolution_clock::now();

    ogdf::PlanarizationLayout pl;
    ogdf::SubgraphPlanarizer* crossMin = new ogdf::SubgraphPlanarizer;
    ogdf::PlanarSubgraphFast<int>* ps = new ogdf::PlanarSubgraphFast<int>;
    ps->runs(100);
    ogdf::VariableEmbeddingInserter* ves = new ogdf::VariableEmbeddingInserter;
    ves->removeReinsert(ogdf::RemoveReinsertType::All);

    crossMin->setSubgraph(ps);
    crossMin->setInserter(ves);
    pl.setCrossMin(crossMin);

    ogdf::EmbedderMinDepthMaxFaceLayers* emb = new ogdf::EmbedderMinDepthMaxFaceLayers;
    pl.setEmbedder(emb);

    ogdf::OrthoLayout* ol = new ogdf::OrthoLayout;
    ol->separation(100.0);
    ol->cOverhang(0.1);
    pl.setPlanarLayouter(ol);

    // ogdf::setSeed(0);
    pl.call(GA);

    auto end = high_resolution_clock::now();
    chrono::duration<double> elapsed = end - start;

    ogdf::LayoutStatistics stats;

    int crossings = 0;
    for (auto& elem : stats.numberOfCrossings(GA))
        crossings += elem;
    crossings /= 2;

    stringstream ss;
    ogdf::GraphIO::drawSVG(GA, ss);
    OrthogonalDrawing result = convert_ogdf_result(GA, G, graph, ogdf_index_to_nodeid);
    return make_tuple(std::move(result), elapsed.count(), ss.str(), crossings);
}