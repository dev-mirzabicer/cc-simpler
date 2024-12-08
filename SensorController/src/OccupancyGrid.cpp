#include "OccupancyGrid.h"
#include <cstring>
#include <Arduino.h> // For Serial logging

/**
 * @brief Constructor for OccupancyGrid.
 * Initializes all cells to free (0) and occupancy counts to 0.
 */
OccupancyGrid::OccupancyGrid()
    : originX(0.0f), originY(0.0f), originZ(0.0f)
{
    memset(grid, 0, sizeof(grid));
    memset(occupancyCount, 0, sizeof(occupancyCount));
}

/**
 * @brief Set the origin of the grid in world coordinates.
 */
void OccupancyGrid::setOrigin(float ox, float oy, float oz)
{
    originX = ox;
    originY = oy;
    originZ = oz;
}

/**
 * @brief Convert world coordinates to grid indices.
 */
bool OccupancyGrid::worldToGrid(float wx, float wy, float wz, int *gx, int *gy, int *gz) const
{
    int xIndex = static_cast<int>((wx - originX) / CELL_SIZE);
    int yIndex = static_cast<int>((wy - originY) / CELL_SIZE);
    int zIndex = static_cast<int>((wz - originZ) / CELL_SIZE);

    if (xIndex < 0 || xIndex >= GRID_SIZE_X ||
        yIndex < 0 || yIndex >= GRID_SIZE_Y ||
        zIndex < 0 || zIndex >= GRID_SIZE_Z)
    {
        return false;
    }

    *gx = xIndex;
    *gy = yIndex;
    *gz = zIndex;
    return true;
}

/**
 * @brief Mark a cell as occupied (1). If it was previously free (0), record the change.
 *
 * Also, increment occupancy count.
 */
void OccupancyGrid::markCellOccupied(int gx, int gy, int gz, std::vector<ChangedCell> &changes)
{
    if (gx < 0 || gx >= GRID_SIZE_X ||
        gy < 0 || gy >= GRID_SIZE_Y ||
        gz < 0 || gz >= GRID_SIZE_Z)
    {
        // Out of bounds, do nothing
        return;
    }

    if (occupancyCount[gx][gy][gz] < 255)
    {
        occupancyCount[gx][gy][gz]++;
    }

    if (occupancyCount[gx][gy][gz] == 1)
    {
        // Cell was previously free, now occupied
        grid[gx][gy][gz] = 1;
        ChangedCell c{gx, gy, gz};
        changes.push_back(c);
    }
}

/**
 * @brief Mark a cell as free (0). If it was previously occupied (1), record the change.
 *
 * Also, decrement occupancy count.
 */
void OccupancyGrid::markCellFree(int gx, int gy, int gz, std::vector<ChangedCell> &changes)
{
    if (gx < 0 || gx >= GRID_SIZE_X ||
        gy < 0 || gy >= GRID_SIZE_Y ||
        gz < 0 || gz >= GRID_SIZE_Z)
    {
        // Out of bounds, do nothing
        return;
    }

    if (occupancyCount[gx][gy][gz] > 0)
    {
        occupancyCount[gx][gy][gz]--;
    }

    if (occupancyCount[gx][gy][gz] == 0 && grid[gx][gy][gz] == 1)
    {
        // Cell was previously occupied, now free
        grid[gx][gy][gz] = 0;
        ChangedCell c{gx, gy, gz};
        changes.push_back(c);
    }
}

/**
 * @brief Decay occupancy counts for all cells and free cells with zero count.
 *
 * @param decayRate Number of counts to decrement per decay cycle.
 * @param changes Vector to append changed cells.
 */
void OccupancyGrid::decayOccupancy(int decayRate, std::vector<ChangedCell> &changes)
{
    for (int x = 0; x < GRID_SIZE_X; x++)
    {
        for (int y = 0; y < GRID_SIZE_Y; y++)
        {
            for (int z = 0; z < GRID_SIZE_Z; z++)
            {
                if (occupancyCount[x][y][z] > 0)
                {
                    occupancyCount[x][y][z] -= decayRate;
                    if (occupancyCount[x][y][z] < 0)
                        occupancyCount[x][y][z] = 0;

                    if (occupancyCount[x][y][z] == 0 && grid[x][y][z] == 1)
                    {
                        grid[x][y][z] = 0;
                        ChangedCell c{x, y, z};
                        changes.push_back(c);
                    }
                }
            }
        }
    }
}

/**
 * @brief Get the occupancy value of a specific cell.
 */
uint8_t OccupancyGrid::getCell(int gx, int gy, int gz) const
{
    if (gx < 0 || gx >= GRID_SIZE_X ||
        gy < 0 || gy >= GRID_SIZE_Y ||
        gz < 0 || gz >= GRID_SIZE_Z)
    {
        return 0; // Treat out-of-bounds as free
    }
    return grid[gx][gy][gz];
}

/**
 * @brief Retrieve the internal grid data.
 */
const uint8_t *OccupancyGrid::getData() const
{
    return &grid[0][0][0];
}

/**
 * @brief Get the origin of the grid in world coordinates.
 */
void OccupancyGrid::getOrigin(float *ox, float *oy, float *oz) const
{
    *ox = originX;
    *oy = originY;
    *oz = originZ;
}
