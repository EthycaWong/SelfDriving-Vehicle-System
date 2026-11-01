#include "road_network.h"

RoadNetwork::RoadNetwork(int columns, int rows, float spacing)
    : columns_(columns), rows_(rows), spacing_(spacing) {}

Vector2 RoadNetwork::node(int column, int row) const {
    return {static_cast<float>(column) * spacing_, static_cast<float>(row) * spacing_};
}
