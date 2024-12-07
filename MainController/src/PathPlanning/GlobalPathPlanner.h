#ifndef GLOBALPATHPLANNER_H
#define GLOBALPATHPLANNER_H

#include <Arduino.h>
#include "../CommonMessageDefinitions/Message.h"
#include <vector>
#include <queue>
#include <functional>
#include <mutex> // Include mutex for thread safety

// Structure representing a node in the grid
struct GridNode
{
    int x, y, z;
    float cost;
    float heuristic;
    float totalCost;
    GridNode *parent;

    GridNode(int _x, int _y, int _z) : x(_x), y(_y), z(_z), cost(0), heuristic(0), totalCost(0), parent(nullptr) {}
};

// Comparator for the priority queue (min-heap based on totalCost)
struct CompareGridNode
{
    bool operator()(const GridNode *a, const GridNode *b) const
    {
        return a->totalCost > b->totalCost;
    }
};

class GlobalPathPlanner
{
public:
    GlobalPathPlanner();
    void init();
    void setGlobalPath(const Path &path);
    Path getGlobalPath() const;
    void updateOccupancyGrid(const OccupancyGrid &grid);
    bool computePath(const Waypoint &start, const Waypoint &goal, Path &computedPath);
    // Additional methods as needed

private:
    Path globalPath;
    OccupancyGrid currentOccupancyGrid;
    mutable std::mutex plannerMutex; // Mutex for thread safety

    // A* related functions
    bool isOccupied(int x, int y, int z) const;
    std::vector<Waypoint> findPath(const Waypoint &start, const Waypoint &goal);
    float calculateHeuristic(int x, int y, int z, const Waypoint &goal) const;
    bool isValid(int x, int y, int z) const;
};

#endif // GLOBALPATHPLANNER_H
