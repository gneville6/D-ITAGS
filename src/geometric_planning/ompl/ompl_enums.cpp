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
#include "grstapse/geometric_planning/ompl/ompl_enums.hpp"

// External
#include <fmt/format.h>
#include <magic_enum/magic_enum.hpp>
#include <ompl/base/goals/GoalSpace.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
// Local
#include "grstapse/common/utilities/constants.hpp"
#include "grstapse/common/utilities/custom_json_conversions.hpp"
#include "grstapse/common/utilities/error.hpp"

namespace grstapse
{
    std::shared_ptr<ompl::base::State> createState(OmplStateSpaceType state_type)
    {
        switch(state_type)
        {
            case OmplStateSpaceType::e_se2:
            {
                auto s           = std::make_shared<ompl::base::SE2StateSpace::StateType>();
                s->components    = new ompl::base::State*[2];
                s->components[0] = new ompl::base::RealVectorStateSpace::StateType();
                s->components[0]->as<ompl::base::RealVectorStateSpace::StateType>()->values    = new double[2];
                s->components[0]->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = 0;
                s->components[0]->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = 0;
                s->components[1] = new ompl::base::SO2StateSpace::StateType();
                s->components[1]->as<ompl::base::SO2StateSpace::StateType>()->setIdentity();
                return s;
            }
            case OmplStateSpaceType::e_se3:
            {
                auto s           = std::make_shared<ompl::base::SE3StateSpace::StateType>();
                s->components    = new ompl::base::State*[2];
                s->components[0] = new ompl::base::RealVectorStateSpace::StateType();
                s->components[0]->as<ompl::base::RealVectorStateSpace::StateType>()->values    = new double[3];
                s->components[0]->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = 0;
                s->components[0]->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = 0;
                s->components[0]->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = 0;
                s->components[1] = new ompl::base::SO3StateSpace::StateType();
                s->components[1]->as<ompl::base::SO3StateSpace::StateType>()->setIdentity();
                return s;
            }
            default:
            {
                throw createLogicError(fmt::format("Unknown state type"));
            }
        }
    }

    std::shared_ptr<ompl::base::State> loadState(const nlohmann::json& j)
    {
        if(!j.is_object())
        {
            throw createLogicError("j should be an object to be deserialized into a geometric state");
        }

        OmplStateSpaceType state_type = j.at(constants::k_state_type).get<OmplStateSpaceType>();

        switch(state_type)
        {
            case OmplStateSpaceType::e_se2:
            {
                return j.get<std::shared_ptr<ompl::base::SE2StateSpace::StateType>>();
            }
            case OmplStateSpaceType::e_se3:
            {
                return j.get<std::shared_ptr<ompl::base::SE3StateSpace::StateType>>();
            }
            default:
            {
                throw createLogicError(fmt::format("Unknown state type: {0:s}", j.at("state_type").get<std::string>()));
            }
        }
    }

    bool equalStates(const std::shared_ptr<ompl::base::State>& lhs,
                     const std::shared_ptr<ompl::base::State>& rhs,
                     const ompl::base::StateSpacePtr& state_space)
    {
        return ompl::base::ScopedState(state_space, lhs.get()) == ompl::base::ScopedState(state_space, rhs.get());
    }

    ompl::base::StateSpacePtr loadSpace(const nlohmann::json& j)
    {
        if(!j.is_object())
        {
            throw createLogicError("j should be an object to be deserialized into a geometric state");
        }

        OmplStateSpaceType state_type = j.at(constants::k_state_type).get<OmplStateSpaceType>();

        switch(state_type)
        {
            case OmplStateSpaceType::e_se2:
            {
                return j.get<std::shared_ptr<ompl::base::SE2StateSpace>>();
            }
            case OmplStateSpaceType::e_se3:
            {
                return j.get<std::shared_ptr<ompl::base::SE3StateSpace>>();
            }
            default:
            {
                throw createLogicError(
                    fmt::format("Unknown state space type: {0:s}", j.at(constants::k_state_type).get<std::string>()));
            }
        }
    }

    ompl::base::StateSpacePtr cloneStateSpace(const ompl::base::StateSpacePtr& state_space)
    {
        switch(state_space->getType())
        {
            case ompl::base::StateSpaceType::STATE_SPACE_SE2:
            {
                auto se2_state_space = std::dynamic_pointer_cast<ompl::base::SE2StateSpace>(state_space);
                const ompl::base::RealVectorBounds& bounds = se2_state_space->getBounds();

                auto rv = std::make_shared<ompl::base::SE2StateSpace>();
                rv->setBounds(bounds);
                return rv;
            }
            case ompl::base::StateSpaceType::STATE_SPACE_SE3:
            {
                auto se3_state_space = std::dynamic_pointer_cast<ompl::base::SE3StateSpace>(state_space);
                const ompl::base::RealVectorBounds& bounds = se3_state_space->getBounds();

                auto rv = std::make_shared<ompl::base::SE3StateSpace>();
                rv->setBounds(bounds);
                return rv;
            }
            default:
            {
                throw createLogicError("Not implemented");
            }
        }
    }

    bool equalStateSpaces(const ompl::base::StateSpacePtr& lhs, const ompl::base::StateSpacePtr& rhs)
    {
        if(lhs->getType() != rhs->getType())
        {
            return false;
        }

        switch(lhs->getType())
        {
            case ompl::base::StateSpaceType::STATE_SPACE_SE2:
            {
                auto lhs_se2_state_space = std::dynamic_pointer_cast<ompl::base::SE2StateSpace>(lhs);
                auto rhs_se2_state_space = std::dynamic_pointer_cast<ompl::base::SE2StateSpace>(rhs);

                const ompl::base::RealVectorBounds& lhs_bounds = lhs_se2_state_space->getBounds();
                const ompl::base::RealVectorBounds& rhs_bounds = rhs_se2_state_space->getBounds();

                for(unsigned int i = 0; i < 2; ++i)
                {
                    if(lhs_bounds.low[i] != rhs_bounds.low[i] || lhs_bounds.high[i] != rhs_bounds.high[i])
                    {
                        return false;
                    }
                }
                return true;
            }
            case ompl::base::StateSpaceType::STATE_SPACE_SE3:
            {
                auto lhs_se3_state_space = std::dynamic_pointer_cast<ompl::base::SE3StateSpace>(lhs);
                auto rhs_se3_state_space = std::dynamic_pointer_cast<ompl::base::SE3StateSpace>(rhs);

                const ompl::base::RealVectorBounds& lhs_bounds = lhs_se3_state_space->getBounds();
                const ompl::base::RealVectorBounds& rhs_bounds = rhs_se3_state_space->getBounds();

                for(unsigned int i = 0; i < 3; ++i)
                {
                    if(lhs_bounds.low[i] != rhs_bounds.low[i] || lhs_bounds.high[i] != rhs_bounds.high[i])
                    {
                        return false;
                    }
                }
                return true;
            }
            default:
            {
                throw createLogicError("Not implemented");
            }
        }
    }

    ompl::base::GoalPtr loadGoal(const nlohmann::json& j, const ompl::base::SpaceInformationPtr& space_information)
    {
        if(!j.is_object())
        {
            throw createLogicError("j should be an object to be deserialized into a geometric goal");
        }

        OmplGoalType goal_type = j.at(constants::k_goal_type).get<OmplGoalType>();

        switch(goal_type)
        {
            case OmplGoalType::e_state:
            {
                auto goal_state                          = std::make_shared<ompl::base::GoalState>(space_information);
                std::shared_ptr<ompl::base::State> state = loadState(j);
                goal_state->setState(state.get());
                return goal_state;
            }
            case OmplGoalType::e_set_of_states:
            {
                auto goal_states               = std::make_shared<ompl::base::GoalStates>(space_information);
                const nlohmann::json& states_j = j.at(constants::k_states);
                for(const nlohmann::json& state_j: states_j)
                {
                    std::shared_ptr<ompl::base::State> state = loadState(state_j);
                    goal_states->addState(state.get());
                }

                return goal_states;
            }
            case OmplGoalType::e_space:
            {
                auto goal_space                 = std::make_shared<ompl::base::GoalSpace>(space_information);
                ompl::base::StateSpacePtr space = loadSpace(j);
                goal_space->setSpace(space);
                return goal_space;
            }
            default:
            {
                throw createLogicError(
                    fmt::format("Unknown goal type: {0:s}", j.at(constants::k_goal_type).get<std::string>()));
            }
        }
    }

    bool equalGoals(const ompl::base::GoalPtr& lhs,
                    const ompl::base::GoalPtr& rhs,
                    const ompl::base::StateSpacePtr& state_space)
    {
        if(lhs->getType() != rhs->getType())
        {
            return false;
        }

        switch(lhs->getType())
        {
            // ompl::base::GoalState
            case ompl::base::GoalType::GOAL_STATE:
            {
                return equalGoals(std::dynamic_pointer_cast<ompl::base::GoalState>(lhs),
                                  std::dynamic_pointer_cast<ompl::base::GoalState>(rhs),
                                  state_space);
            }
            // ompl::base::GoalStates
            case ompl::base::GoalType::GOAL_STATES:
            {
                return equalGoals(std::dynamic_pointer_cast<ompl::base::GoalStates>(lhs),
                                  std::dynamic_pointer_cast<ompl::base::GoalStates>(rhs),
                                  state_space);
            }
            // ompl::base::GoalSpace
            case ompl::base::GoalType::GOAL_SAMPLEABLE_REGION:
            {
                return equalGoals(std::dynamic_pointer_cast<ompl::base::GoalSpace>(lhs),
                                  std::dynamic_pointer_cast<ompl::base::GoalSpace>(rhs),
                                  state_space);
            }
            default:
            {
                throw createLogicError("Not Implemented");
            }
        }
    }

    bool equalGoals(const std::shared_ptr<ompl::base::GoalState>& lhs,
                    const std::shared_ptr<ompl::base::GoalState>& rhs,
                    const ompl::base::StateSpacePtr& state_space)
    {
        if(lhs == nullptr || rhs == nullptr)
        {
            return false;
        }

        return state_space->equalStates(lhs->getState(), rhs->getState());
    }

    bool equalGoals(const std::shared_ptr<ompl::base::GoalStates>& lhs,
                    const std::shared_ptr<ompl::base::GoalStates>& rhs,
                    const ompl::base::StateSpacePtr& state_space)
    {
        if(lhs == nullptr || rhs == nullptr)
        {
            return false;
        }

        // Check if there are the same number of states in the goal
        if(lhs->getStateCount() != rhs->getStateCount())
        {
            return false;
        }

        // TODO(Andrew): Is there a faster way than brute force
        for(unsigned int i = 0, num_states = lhs->getStateCount(); i < num_states; ++i)
        {
            bool found = false;
            for(unsigned int j = 0; j < num_states; ++j)
            {
                if(state_space->equalStates(lhs->getState(i), rhs->getState(j)))
                {
                    found = true;
                    break;
                }
            }
            if(!found)
            {
                return false;
            }
        }
        return true;
    }

    bool equalGoals(const std::shared_ptr<ompl::base::GoalSpace>& lhs,
                    const std::shared_ptr<ompl::base::GoalSpace>& rhs,
                    const ompl::base::StateSpacePtr& state_space)
    {
        if(lhs == nullptr || rhs == nullptr)
        {
            return false;
        }

        std::shared_ptr<ompl::base::StateSpace> lhs_state_space = lhs->getSpace();
        std::shared_ptr<ompl::base::StateSpace> rhs_state_space = rhs->getSpace();

        return equalStateSpaces(lhs_state_space, rhs_state_space);
    }

}  // namespace grstapse