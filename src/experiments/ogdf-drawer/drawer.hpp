#ifndef OGDF_DRAWER_H
#define OGDF_DRAWER_H

#include <domus/orthogonal/drawing.hpp>
#include <string>
#include <tuple>

class UndirectedGraph;

std::tuple<OrthogonalDrawing, double, std::string, int>
make_orthogonal_drawing_ogdf(const UndirectedGraph& graph);

#endif
