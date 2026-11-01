#pragma once

#include <raylib.h>

// A square grid of two-way roads. Each direction has two lanes and a shoulder, and traffic drives
// on the right. World axes: +X east, +Y up, +Z south. Facing +X, the driver's right is +Z.
class RoadNetwork {
public:
    static constexpr float kLaneWidth = 3.5f;
    static constexpr float kShoulderWidth = 2.0f;
    static constexpr float kSidewalkWidth = 3.0f;
    static constexpr float kHalfRoad = 2 * kLaneWidth + kShoulderWidth;
    static constexpr float kHalfStreet = kHalfRoad + kSidewalkWidth;

    RoadNetwork(int columns, int rows, float spacing);

    int columns() const { return columns_; }
    int rows() const { return rows_; }
    float spacing() const { return spacing_; }

    Vector2 node(int column, int row) const;
    float width() const { return (columns_ - 1) * spacing_; }
    float depth() const { return (rows_ - 1) * spacing_; }

    // Distance from the road center to the middle of a lane. Lane 0 is next to the center line.
    static float laneOffset(int lane) { return (static_cast<float>(lane) + 0.5f) * kLaneWidth; }
    static float shoulderOffset() { return 2 * kLaneWidth + kShoulderWidth / 2; }

private:
    int columns_;
    int rows_;
    float spacing_;
};
