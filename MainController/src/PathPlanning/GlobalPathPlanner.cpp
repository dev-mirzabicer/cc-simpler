#include "GlobalPathPlanner.h"
#include <algorithm>
#include <cmath>

// Constructor
GlobalPathPlanner::GlobalPathPlanner()
{
    // Initialize occupancy grid with free space
    // Initially empty; will be updated from remote computer
}

// Initialize Global Path Planner
void GlobalPathPlanner::init()
{
    // Any additional initialization
}

// Set global path received from Remote Computer
void GlobalPathPlanner::setGlobalPath(const Path &path)
{
    std::lock_guard<std::mutex> lock(plannerMutex);
    globalPath = path;
}

// Get current global path
Path GlobalPathPlanner::getGlobalPath() const
{
    std::lock_guard<std::mutex> lock(plannerMutex);
    return globalPath;
}

// Update occupancy grid based on received data
void GlobalPathPlanner::updateOccupancyGrid(const OccupancyGrid &grid)
{
    std::lock_guard<std::mutex> lock(plannerMutex);
    currentOccupancyGrid = grid;
}

// Check if a grid cell is occupied
bool GlobalPathPlanner::isOccupied(int x, int y, int z) const
{
    std::lock_guard<std::mutex> lock(plannerMutex);
    if (x < 0 || x >= currentOccupancyGrid.sizeX ||
        y < 0 || y >= currentOccupancyGrid.sizeY ||
        z < 0 || z >= currentOccupancyGrid.sizeZ)
    {
        return true; // Out of bounds treated as occupied
    }
    size_t index = z * currentOccupancyGrid.sizeY * currentOccupancyGrid.sizeX + y * currentOccupancyGrid.sizeX + x;
    if (index >= currentOccupancyGrid.gridData.size())
    {
        return true; // Invalid index treated as occupied
    }
    return (currentOccupancyGrid.gridData[index] == 1);
}

// Calculate heuristic (Euclidean distance)
float GlobalPathPlanner::calculateHeuristic(int x, int y, int z, const Waypoint &goal) const
{
    float dx = (x * currentOccupancyGrid.resolution) - goal.x;
    float dy = (y * currentOccupancyGrid.resolution) - goal.y;
    float dz = (z * currentOccupancyGrid.resolution) - goal.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

// Check if the cell is within the grid and not occupied
bool GlobalPathPlanner::isValid(int x, int y, int z) const
{
    return (x >= 0 && x < currentOccupancyGrid.sizeX &&
            y >= 0 && y < currentOccupancyGrid.sizeY &&
            z >= 0 && z < currentOccupancyGrid.sizeZ &&
            !isOccupied(x, y, z));
}

// Find path using A* algorithm with priority queue
std::vector<Waypoint> GlobalPathPlanner::findPath(const Waypoint &start, const Waypoint &goal)
{
    std::vector<Waypoint> path;

    // Convert start and goal positions to grid coordinates
    int startX = round(start.x / currentOccupancyGrid.resolution);
    int startY = round(start.y / currentOccupancyGrid.resolution);
    int startZ = round(start.z / currentOccupancyGrid.resolution);

    int goalX = round(goal.x / currentOccupancyGrid.resolution);
    int goalY = round(goal.y / currentOccupancyGrid.resolution);
    int goalZ = round(goal.z / currentOccupancyGrid.resolution);

    // Initialize open and closed lists
    std::priority_queue<GridNode *, std::vector<GridNode *>, CompareGridNode> openList;
    std::vector<std::vector<std::vector<bool>>> closedList(
        currentOccupancyGrid.sizeX,
        std::vector<std::vector<bool>>(
            currentOccupancyGrid.sizeY,
            std::vector<bool>(currentOccupancyGrid.sizeZ, false)));

    GridNode *startNode = new GridNode(startX, startY, startZ);
    startNode->cost = 0.0f;
    startNode->heuristic = calculateHeuristic(startX, startY, startZ, goal);
    startNode->totalCost = startNode->heuristic;

    openList.push(startNode);

    while (!openList.empty())
    {
        GridNode *current = openList.top();
        openList.pop();

        // If already closed, skip
        if (closedList[current->x][current->y][current->z])
        {
            delete current;
            continue;
        }

        // Mark as closed
        closedList[current->x][current->y][current->z] = true;

        // Check if goal is reached
        if (current->x == goalX && current->y == goalY && current->z == goalZ)
        {
            // Reconstruct path
            GridNode *node = current;
            while (node != nullptr)
            {
                Waypoint wp;
                wp.x = node->x * currentOccupancyGrid.resolution;
                wp.y = node->y * currentOccupancyGrid.resolution;
                wp.z = node->z * currentOccupancyGrid.resolution;
                wp.yaw = 0.0f; // Placeholder; can be set based on path orientation
                path.push_back(wp);
                node = node->parent;
            }
            // Reverse path to start from beginning
            std::reverse(path.begin(), path.end());

            // Clean up remaining nodes in open list
            while (!openList.empty())
            {
                GridNode *remaining = openList.top();
                openList.pop();
                delete remaining;
            }

            // Clean up current node
            delete current;
            break;
        }

        // Generate neighbors (6-connected grid)
        const int directions[6][3] = {
            {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};

        for (int i = 0; i < 6; ++i)
        {
            int neighborX = current->x + directions[i][0];
            int neighborY = current->y + directions[i][1];
            int neighborZ = current->z + directions[i][2];

            if (!isValid(neighborX, neighborY, neighborZ))
                continue;

            if (closedList[neighborX][neighborY][neighborZ])
                continue;

            // Calculate cost
            float newCost = current->cost + currentOccupancyGrid.resolution;

            // Create neighbor node
            GridNode *neighbor = new GridNode(neighborX, neighborY, neighborZ);
            neighbor->cost = newCost;
            neighbor->heuristic = calculateHeuristic(neighborX, neighborY, neighborZ, goal);
            neighbor->totalCost = neighbor->cost + neighbor->heuristic;
            neighbor->parent = current;

            openList.push(neighbor);
        }

        // Delete current node to free memory
        delete current;
    }

    return path;
}

// Compute path using A* and update global path
bool GlobalPathPlanner::computePath(const Waypoint &start, const Waypoint &goal, Path &computedPath)
{
    std::lock_guard<std::mutex> lock(plannerMutex);
    computedPath.waypoints = findPath(start, goal);
    return !computedPath.waypoints.empty();
}
