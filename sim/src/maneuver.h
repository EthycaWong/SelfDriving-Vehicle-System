#pragma once

#include <random>
#include <vector>

#include "bus.h"
#include "messages.h"
#include "road_network.h"

// Turns the Planner's maneuver choices into paths. While following a lane, the path is the lane
// plus a sideways offset that blends smoothly from one value to another, so lane changes, passing
// and pulling over are all the same mechanism. Turns and U-turns use fixed curves.
class ManeuverModule {
public:
    ManeuverModule(MessageBus& bus, const RoadNetwork& roads, unsigned seed);

    // Puts the car on the road from one grid node to a neighbouring node.
    void start(int fromColumn, int fromRow, int toColumn, int toRow, int lane);
    void tick();

    // Picks the road the car is on after manual driving, so it can carry on from there.
    void relocate(Vec2 position, float heading);
    // With random routing off, the car only turns where the road ends or when asked.
    void setRandomRoute(bool enabled) { randomRoute_ = enabled; }

    const ManeuverStatus& status() const { return status_; }

private:
    struct Edge {
        int fromColumn = 0, fromRow = 0, toColumn = 1, toRow = 0;
    };
    enum class NextMove { Straight, Left, Right };

    RoadFrame frameOf(const Edge& edge) const;
    bool validEdge(const Edge& edge) const;
    Edge edgeAfter(const Edge& edge, NextMove move) const;
    void chooseNextMove();

    float offsetAt(float along) const;
    void blendTo(float lateral, float distance);
    void handleRequest(const ManeuverRequest& request);
    void finish();
    void abortManeuver();

    void updateLaneFollowing();
    void beginTurn();
    void updateFixedPath();
    void beginUTurn();
    void beginReverse();
    void publish();

    MessageBus& bus_;
    const RoadNetwork& roads_;
    std::mt19937 rng_;
    VehicleState vehicle_{};
    ManeuverStatus status_{};
    Edge edge_{};
    RoadFrame frame_{};
    NextMove next_ = NextMove::Straight;
    int lane_ = 1;
    bool randomRoute_ = true;

    // Sideways offset blend, in road coordinates.
    float blendStart_ = 0, blendEnd_ = 0, offsetStart_ = 0, offsetEnd_ = 0;
    float homeOffset_ = 0;  // Where to return if a maneuver is aborted.
    float passAlong_ = 0;   // Along-road position to clear before returning.
    float passOffset_ = 0;
    int phase_ = 0;

    std::vector<std::vector<PathPoint>> segments_;  // Fixed paths, for example the 3-point turn.
    std::vector<bool> segmentReverse_;
    size_t segment_ = 0;
    Edge afterFixed_{};
    int laneAfterFixed_ = 1;
    float reverseFrom_ = 0;

    ManeuverPath path_{};
};
