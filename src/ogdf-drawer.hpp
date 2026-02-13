#ifndef OGDF_DRAWER_H
#define OGDF_DRAWER_H

#include <domus/core/graph/graph.hpp>
#include <domus/orthogonal/drawing.hpp>
#include <string>
#include <tuple>

std::tuple<OrthogonalDrawing, double, std::string>
make_orthogonal_drawing_ogdf(const UndirectedGraph& graph);

#endif
