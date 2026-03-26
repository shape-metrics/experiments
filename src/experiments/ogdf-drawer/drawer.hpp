#ifndef OGDF_DRAWER_H
#define OGDF_DRAWER_H

#include <domus/core/graph/graph.hpp>
#include <domus/orthogonal/drawing.hpp>

#include <string>
#include <tuple>

std::tuple<domus::orthogonal::OrthogonalDrawing, double, std::string, int>
make_orthogonal_drawing_ogdf(const domus::graph::Graph& graph);

#endif
