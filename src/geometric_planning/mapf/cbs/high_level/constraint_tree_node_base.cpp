/*
 * Graphically Recursive Simultaneous Task Allocation, Planning,
 * Scheduling, and Execution
 *
 * Copyright (C) 2020-2022
 *
 * Author: Andrew Messing
 * Author: Glen Neville
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#include "grstapse/geometric_planning/mapf/cbs/high_level/constraint_tree_node_base.hpp"

// Global
#include <iostream>

// Local
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/geometric_planning/grid/grid_map.hpp"
#include "grstapse/geometric_planning/mapf/cbs/high_level/edge_conflict.hpp"
#include "grstapse/geometric_planning/mapf/cbs/high_level/vertex_conflict.hpp"
#include "grstapse/geometric_planning/mapf/cbs/low_level/temporal_grid_cell_node.hpp"

namespace grstapse
{
    unsigned int ConstraintTreeNodeBase::s_next_id = 0;

    ConstraintTreeNodeBase::ConstraintTreeNodeBase(unsigned int num_robots,
                                                   ConstraintTreeNodeCostType cost,
                                                   const std::shared_ptr<const ConstraintTreeNodeBase>& parent)
        : SearchNodeBase<ConstraintTreeNodeBase>(s_next_id++, parent)
        , m_num_robots(num_robots)
        , m_cost_type(cost)
    {}

    unsigned int ConstraintTreeNodeBase::cost() const
    {
        switch(m_cost_type)
        {
            case ConstraintTreeNodeCostType::e_makespan:
                return makespan();
            case ConstraintTreeNodeCostType::e_sum_of_costs:
                return sumOfCosts();
            default:
                throw createLogicError("Unknown cost type");
        }
    }

    unsigned int ConstraintTreeNodeBase::makespan() const
    {
        unsigned int rv = 0;
        for(unsigned int robot = 0; robot < m_num_robots; ++robot)
        {
            const auto& low_level_solution = lowLevelSolution(robot);
            rv                             = std::max(rv, static_cast<unsigned int>(low_level_solution.size()));
        }
        return rv;
    }

    unsigned int ConstraintTreeNodeBase::sumOfCosts() const
    {
        unsigned int rv = 0;
        for(unsigned int robot = 0; robot < m_num_robots; ++robot)
        {
            const auto& low_level_solution = lowLevelSolution(robot);
            rv += static_cast<unsigned int>(low_level_solution.size());
        }
        return rv;
    }

    std::unique_ptr<const ConflictBase> ConstraintTreeNodeBase::getFirstConflict() const
    {
        unsigned int max_time = makespan();

        for(unsigned int t = 0; t < max_time; ++t)
        {
            // Check vertex collisions
            for(unsigned int robot_i = 0; robot_i < m_num_robots; ++robot_i)
            {
                std::shared_ptr<const GridCell> state_i = getStateOrLast(robot_i, t);

                // If robot i has finished its path
                //                if(state_i == nullptr)
                //                {
                //                    continue;
                //                }
                for(unsigned int robot_j = robot_i + 1; robot_j < m_num_robots; ++robot_j)
                {
                    std::shared_ptr<const GridCell> state_j = getStateOrLast(robot_j, t);

                    // If robot j has finished its path
                    //                    if(state_j == nullptr)
                    //                    {
                    //                        continue;
                    //                    }

                    // If robot i and j are at the same GridCell at the same time
                    if(*state_i == *state_j)
                    {
                        return std::move(
                            std::make_unique<const VertexConflict>(std::array<unsigned int, 2>{robot_i, robot_j},
                                                                   t,
                                                                   state_i->x(),
                                                                   state_i->y()));
                    }
                }
            }

            // Check edge collisions
            if(t < max_time - 1)
            {
                for(unsigned int robot_i = 0; robot_i < m_num_robots; ++robot_i)
                {
                    std::shared_ptr<const GridCell> state_ia = getState(robot_i, t);
                    std::shared_ptr<const GridCell> state_ib = getState(robot_i, t + 1);

                    // If robot i has finished its path
                    if(state_ia == nullptr || state_ib == nullptr)
                    {
                        continue;
                    }
                    for(unsigned int robot_j = robot_i + 1; robot_j < m_num_robots; ++robot_j)
                    {
                        std::shared_ptr<const GridCell> state_ja = getState(robot_j, t);
                        std::shared_ptr<const GridCell> state_jb = getState(robot_j, t + 1);

                        // If robot j has finished its path
                        if(state_ja == nullptr || state_jb == nullptr)
                        {
                            continue;
                        }

                        // If robot i and j swap vertices (would be on the same edge in different directions)
                        if(*state_ia == *state_jb && *state_ib == *state_ja)
                        {
                            return std::move(
                                std::make_unique<const EdgeConflict>(std::array<unsigned int, 2>{robot_i, robot_j},
                                                                     t,
                                                                     state_ia->x(),
                                                                     state_ia->y(),
                                                                     state_ib->x(),
                                                                     state_ib->y()));
                        }
                    }
                }
            }
        }

        return nullptr;
    }

    unsigned int ConstraintTreeNodeBase::priority() const
    {
        return cost();
    }
    void ConstraintTreeNodeBase::display(const std::shared_ptr<const GridMap>& map) const
    {
        unsigned int max_time       = makespan();
        unsigned int width          = map->width();
        unsigned int height         = map->height();
        unsigned int max_text_width = std::max<unsigned int>(7, width);

        for(unsigned int t = 0; t < max_time; ++t)
        {
            std::cout << "t: " << t;
            unsigned int padding;
            if(t < 10)
            {
                padding = max_text_width - 4;
            }
            else if(t < 100)
            {
                padding = max_text_width - 5;
            }
            else if(t < 1000)
            {
                padding = max_text_width - 6;
            }
            else
            {
                padding = max_text_width - 7;
            }

            for(unsigned int i = 0; i < padding; ++i)
            {
                std::cout << " ";
            }
        }
        std::cout << std::endl;

        for(unsigned int y = 0; y < height; ++y)
        {
            for(unsigned int t = 0; t < max_time; ++t)
            {
                for(unsigned int x = 0; x < width; ++x)
                {
                    if(map->isObstacle(x, y))
                    {
                        std::cout << "\033[7;31m"
                                  << "X"
                                  << "\033[27;0m";
                        continue;
                    }

                    unsigned int num_r = 0;
                    unsigned int robot;
                    for(unsigned int r = 0; r < m_num_robots; ++r)
                    {
                        const auto& low_level_solution = lowLevelSolution(r);
                        if(t >= low_level_solution.size())
                        {
                            if(low_level_solution.back()->x() == x && low_level_solution.back()->y() == y)
                            {
                                ++num_r;
                                robot = r;
                            }
                            continue;
                        }

                        if(low_level_solution[t]->x() == x && low_level_solution[t]->y() == y)
                        {
                            ++num_r;
                            robot = r;
                        }
                    }

                    if(num_r > 1)
                    {
                        std::cout << "\033[1;31m" << num_r << "\033[0m";
                    }
                    else if(num_r == 1)
                    {
                        std::cout << "\033[1;32m" << robot << "\033[0m";
                    }
                    else
                    {
                        std::cout << "0";
                    }
                }
                if(width < max_text_width)
                {
                    for(unsigned int i = 0, end = max_text_width - width; i < end; ++i)
                    {
                        std::cout << " ";
                    }
                }
            }
            std::cout << std::endl;
        }
    }

    std::shared_ptr<const GridCell> ConstraintTreeNodeBase::getStateOrLast(unsigned int robot, unsigned int time) const
    {
        assert(robot < m_num_robots);
        if(time < lowLevelSolution(robot).size())
        {
            return std::dynamic_pointer_cast<const GridCell>(lowLevelSolution(robot)[time]);
        }
        return std::dynamic_pointer_cast<const GridCell>(lowLevelSolution(robot).back());
    }

    std::shared_ptr<const GridCell> ConstraintTreeNodeBase::getState(unsigned int robot, unsigned int time) const
    {
        assert(robot < m_num_robots);
        if(time < lowLevelSolution(robot).size())
        {
            return std::dynamic_pointer_cast<const GridCell>(lowLevelSolution(robot)[time]);
        }
        return nullptr;
    }
    std::shared_ptr<const GridCell> ConstraintTreeNodeBase::getLastState(unsigned int robot) const
    {
        assert(robot < m_num_robots);
        return std::dynamic_pointer_cast<const GridCell>(lowLevelSolution(robot).back());
    }
}  // namespace grstapse