#ifndef OCCUPANCYGRID_H
#define OCCUPANCYGRID_H

#include <cstdint>
#include <vector>

/**
 * @brief Structure to represent a changed cell in the occupancy grid.
 */
struct ChangedCell
{
    int x;
    int y;
    int z;
};

/**
 * @brief The OccupancyGrid class manages a 3D grid representing the environment.
 *
 * It allows marking cells as occupied and free, tracks changes for efficient communication,
 * and implements cell decay to handle dynamic environments.
 */
class OccupancyGrid
{
public:
    static const int GRID_SIZE_X = 100;
    static const int GRID_SIZE_Y = 100;
    static const int GRID_SIZE_Z = 10;
    static constexpr float CELL_SIZE = 1.0f; // meters

    /**
     * @brief Constructor for OccupancyGrid.
     * Initializes all cells to free (0) and occupancy counts to 0.
     */
    OccupancyGrid();

    /**
     * @brief Set the origin of the grid in world coordinates.
     *
     * @param ox Origin X coordinate in meters.
     * @param oy Origin Y coordinate in meters.
     * @param oz Origin Z coordinate in meters.
     */
    void setOrigin(float ox, float oy, float oz);

    /**
     * @brief Convert world coordinates to grid indices.
     *
     * @param wx World X coordinate in meters.
     * @param wy World Y coordinate in meters.
     * @param wz World Z coordinate in meters.
     * @param gx Pointer to store grid X index.
     * @param gy Pointer to store grid Y index.
     * @param gz Pointer to store grid Z index.
     * @return bool true if conversion is within grid bounds, false otherwise.
     */
    bool worldToGrid(float wx, float wy, float wz, int *gx, int *gy, int *gz) const;

    /**
     * @brief Mark a cell as occupied (1). If it was previously free (0), record the change.
     *
     * Also, increment occupancy count.
     *
     * @param gx Grid X index.
     * @param gy Grid Y index.
     * @param gz Grid Z index.
     * @param changes Vector to append changed cells.
     */
    void markCellOccupied(int gx, int gy, int gz, std::vector<ChangedCell> &changes);

    /**
     * @brief Mark a cell as free (0). If it was previously occupied (1), record the change.
     *
     * Also, decrement occupancy count.
     *
     * @param gx Grid X index.
     * @param gy Grid Y index.
     * @param gz Grid Z index.
     * @param changes Vector to append changed cells.
     */
    void markCellFree(int gx, int gy, int gz, std::vector<ChangedCell> &changes);

    /**
     * @brief Decay occupancy counts for all cells and free cells with zero count.
     *
     * @param decayRate Number of counts to decrement per decay cycle.
     * @param changes Vector to append changed cells.
     */
    void decayOccupancy(int decayRate, std::vector<ChangedCell> &changes);

    /**
     * @brief Get the occupancy value of a specific cell.
     *
     * @param gx Grid X index.
     * @param gy Grid Y index.
     * @param gz Grid Z index.
     * @return uint8_t 1 if occupied, 0 if free.
     */
    uint8_t getCell(int gx, int gy, int gz) const;

    /**
     * @brief Retrieve the internal grid data.
     *
     * @return const uint8_t* Pointer to the grid data.
     */
    const uint8_t *getData() const;

    /**
     * @brief Get the origin of the grid in world coordinates.
     *
     * @param ox Pointer to store Origin X coordinate.
     * @param oy Pointer to store Origin Y coordinate.
     * @param oz Pointer to store Origin Z coordinate.
     */
    void getOrigin(float *ox, float *oy, float *oz) const;

private:
    uint8_t grid[GRID_SIZE_X][GRID_SIZE_Y][GRID_SIZE_Z];
    uint8_t occupancyCount[GRID_SIZE_X][GRID_SIZE_Y][GRID_SIZE_Z];
    float originX;
    float originY;
    float originZ;
};

#endif // OCCUPANCYGRID_H
