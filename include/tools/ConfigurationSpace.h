#pragma once

#include <vector>
#include <Eigen/Core>

#include "tools/Serializer.h"

namespace amp {

/// @brief N-Dimensional configuration space/collision checker. You can derive this to implement different
/// collision checkers or C-space representations for different problems
class ConfigurationSpace {
    public: 
        ConfigurationSpace(const Eigen::VectorXd& lower_bounds, const Eigen::VectorXd& upper_bounds)
            : m_lower_bounds(lower_bounds)
            , m_upper_bounds(upper_bounds) 
        {
            ASSERT(lower_bounds.size() == upper_bounds.size(), "Upper and lower bounds do not have the same size")
        }

        /******* User Implemented Methods ********/

        /// @brief Check if a state in C-space is colliding
        /// @param cspace_state N-dimensional C-space state
        /// @return `true` if the the state is in collision, `false` if it is not
        virtual bool inCollision(const Eigen::VectorXd& cspace_state) const = 0;

        /*****************************************/

        /// @brief Get the lower bounds for each configuration dimension
        /// @return Lower bounds
        inline const Eigen::VectorXd& lowerBounds() const {return m_lower_bounds;}

        /// @brief Get the upper bounds for each configuration dimension
        /// @return Lower bounds
        inline const Eigen::VectorXd& upperBounds() const {return m_upper_bounds;}

        /// @brief Get the dimension of the configuration space
        /// @return Dimension
        inline std::size_t dimension() const {return m_lower_bounds.size();}

    protected:
        Eigen::VectorXd m_lower_bounds;
        Eigen::VectorXd m_upper_bounds;

};

/// @brief User implemented abstract class that accesses the continuous C-Space (bounded)
class ConfigurationSpace2D {
    public:
        /// @brief Constructor
        /// @param x0_min Lower bound on first configuration dimension
        /// @param x0_max Upper bound on first configuration dimension
        /// @param x1_min Lower bound on second configuration dimension
        /// @param x1_max Upper bound on second configuration dimension
        ConfigurationSpace2D(double x0_min, double x0_max, double x1_min, double x1_max)
            : m_x0_bounds(x0_min, x0_max)
            , m_x1_bounds(x1_min, x1_max)
            {}

            /******* User Implemented Methods ********/

            /// @brief Check if a point in C-space is colliding.
            /// NOTE: For grid-discretization of C-space, this method should simply find the cell that (x0, x1) is within, then return the collision value of that cell (see below).
            /// For other implementations, you can simply implement a collision checker here.
            /// @param x0 Value of the first configuration space variable
            /// @param x1 Value of the second configuration space variable
            /// @return `true` if the the point is in collision, `false` if it is not
            virtual bool inCollision(double x0, double x1) const = 0;

            /*****************************************/

        /// @brief Get bounds for the first config dimension
        /// @return Bounds
        inline const std::pair<double, double>& x0Bounds() const {
            return m_x0_bounds;
        }

        /// @brief Get bounds for the second config dimension
        /// @return Bounds
        inline const std::pair<double, double>& x1Bounds() const {
            return m_x1_bounds;
        }

        /// @brief Virtual dtor
        virtual ~ConfigurationSpace2D() {}
    protected:
        std::pair<double, double> m_x0_bounds;
        std::pair<double, double> m_x1_bounds;
};

/* Some of the tools below might help you define your C-Space */

template <typename T = bool>
class DenseArray2D {
    public:
        /// @brief Constructor that initializes data to T{}
        /// @param x0_cells Number of cells along the first configuration dimension
        /// @param x1_cells Number of cells along the second configuration dimension
        DenseArray2D(std::size_t x0_cells, std::size_t x1_cells);

        /// @brief Constructor that initializes data to T{}
        /// @param x0_cells Number of cells along the first configuration dimension
        /// @param x1_cells Number of cells along the second configuration dimension
        /// @param default_element Use custom default element to fill array with
        DenseArray2D(std::size_t x0_cells, std::size_t x1_cells, T default_element);

        /// @brief Get the # of cells along x0 and number of cells along x1
        /// @return 
        inline std::pair<std::size_t, std::size_t> size() const;

        /// @brief Edit/Access an element with bounds checking
        /// @param i x0 index
        /// @param j x1 index
        /// @return Reference to the element (for use with vector<bool>)
        inline typename std::vector<T>::reference operator()(std::size_t i, std::size_t j);

        /// @brief Read an element with bounds checking
        /// @param i x0 index
        /// @param j x1 index
        /// @return Const reference to the element
        inline typename std::vector<T>::const_reference operator()(std::size_t i, std::size_t j) const;

        /// @brief Read the wrapped data
        /// @return Wrapped data
        inline const std::vector<bool>& data() const;

        /// @brief Virtual dtor
        virtual ~DenseArray2D() {}

    private:
        inline std::size_t getWrappedIndex(std::size_t i, std::size_t j) const;

    private:
        std::vector<T> m_data;
        const std::size_t m_x0_cells, m_x1_cells;
};

/// @brief Abstract type that can be used with the Visualizer
/// NOTE: This class is still abstract becase the `inCollision` method has not been overridden. You will need to override `inCollision` to return the boolean collision
/// value in the correct DenseArray2D<bool> cell.
class GridCSpace2D : public ConfigurationSpace2D, public DenseArray2D<bool> {
    public:
        GridCSpace2D(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
            : ConfigurationSpace2D(x0_min, x0_max, x1_min, x1_max)
            , DenseArray2D<bool>(x0_cells, x1_cells)
            {}

        /******* User Implemented Methods ********/
        
        /// @brief Given a point in continuous space that is between the bounds, determine what cell (i, j) that the point is in
        /// @param x0 Value of the first configuration space variable
        /// @param x1 Value of the second configuration space variable
        /// @return A pair (i, j) of indices that correspond to the cell that (x0, x1) is in
        virtual std::pair<std::size_t, std::size_t> getCellFromPoint(double x0, double x1) const = 0;

        /*****************************************/

        /// @brief I have overridded this method for you. This method uses the `getCellFromPoint` to determine which cell (x0, x1) is in,
        /// then return the boolean value inside that cell
        /// @param x0 Value of the first configuration space variable
        /// @param x1 Value of the second configuration space variable
        /// @return A pair (i, j) of indices that correspond to the cell that (x0, x1) is in
        virtual bool inCollision(double x0, double x1) const override {
            auto[i, j] = getCellFromPoint(x0, x1);
            return operator()(i, j);
        }

        /// @brief Virtual dtor
        virtual ~GridCSpace2D() {}
};

// this is all claude AI:
/// @brief N-Dimensional grid-based configuration space (derives from ConfigurationSpace)
class MyGridCSpaceND : public ConfigurationSpace {
    public:
        MyGridCSpaceND(const Eigen::VectorXd& lower_bounds, 
                       const Eigen::VectorXd& upper_bounds,
                       const std::vector<std::size_t>& cells_per_dim)
            : ConfigurationSpace(lower_bounds, upper_bounds)
            , m_cells_per_dim(cells_per_dim)
            , m_data(computeTotalCells(), false) // flat array of collision values
        {
            ASSERT(cells_per_dim.size() == dimension(), 
                   "cells_per_dim must match configuration space dimension");
        }

        /// @brief Convert continuous point to cell indices (generalizes your 2D approach)
        /// @param point N-dimensional point in continuous C-space
        /// @return Vector of cell indices, one per dimension
        std::vector<std::size_t> getCellFromPoint(const Eigen::VectorXd& point) const {
            ASSERT(point.size() == (int)dimension(), "Point dimension must match C-space dimension");
            
            std::vector<std::size_t> cell_indices(dimension());
            
            for (std::size_t d = 0; d < dimension(); ++d) {
                double min_bound = lowerBounds()(d);
                double max_bound = upperBounds()(d);
                std::size_t num_cells = m_cells_per_dim[d];
                
                // Normalize to [0, 1]
                double u = (point(d) - min_bound) / (max_bound - min_bound);
                
                // Scale to cell index
                std::size_t cell_idx = static_cast<std::size_t>(std::floor(u * num_cells));
                
                // Clamp to valid range [0, num_cells - 1]
                cell_idx = std::min(cell_idx, num_cells - 1);
                
                cell_indices[d] = cell_idx;
            }
            
            return cell_indices;
        }

        /// @brief Set collision value for a cell
        /// @param indices Cell indices (one per dimension)
        /// @param collision true for collision (1), false for free (0)
        void setCell(const std::vector<std::size_t>& indices, bool collision) {
            ASSERT(indices.size() == dimension(), "Index vector size must match dimension");
            std::size_t flat_idx = indicesToFlat(indices);
            m_data[flat_idx] = collision;
        }

        /// @brief Get collision value for a cell
        /// @param indices Cell indices (one per dimension)
        /// @return true if collision, false if free
        bool getCell(const std::vector<std::size_t>& indices) const {
            ASSERT(indices.size() == dimension(), "Index vector size must match dimension");
            std::size_t flat_idx = indicesToFlat(indices);
            return m_data[flat_idx];
        }

        /// @brief Override inCollision to query the grid
        bool inCollision(const Eigen::VectorXd& cspace_state) const override {
            auto cell_indices = getCellFromPoint(cspace_state);
            return getCell(cell_indices);
        }

    private:
        std::vector<std::size_t> m_cells_per_dim;
        std::vector<bool> m_data; // Flat storage: 1 = collision, 0 = free

        /// @brief Convert multi-dimensional indices to flat array index
        /// Uses row-major ordering (last dimension varies fastest)
        std::size_t indicesToFlat(const std::vector<std::size_t>& indices) const {
            std::size_t flat_idx = 0;
            std::size_t stride = 1;
            for (int d = (int)dimension() - 1; d >= 0; --d) {
                flat_idx += indices[d] * stride;
                stride *= m_cells_per_dim[d];
            }
            return flat_idx;
        }

        std::size_t computeTotalCells() const {
            std::size_t total = 1;
            for (std::size_t cells : m_cells_per_dim) {
                total *= cells;
            }
            return total;
        }
};


}

#include "public/ConfigurationSpace_impl.h"
