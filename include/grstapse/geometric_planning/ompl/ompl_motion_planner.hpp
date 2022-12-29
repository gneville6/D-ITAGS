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
#pragma once

// Global
#include <concepts>
#include <memory>
#include <mutex>
// External
#include <nlohmann/json.hpp>
#include <ompl/geometric/SimpleSetup.h>
// Local
#include "grstapse/geometric_planning/motion_planner_base.hpp"

#include "ompl_environment.hpp"
#include "ompl_motion_planning_query_result.hpp"

// External Forward Declarations
namespace ompl {
    namespace base {
        class SpaceInformation;

        class PlannerStatus;

        class Planner;

        class GoalState;

        class GoalStates;

        class GoalSpace;
    }  // namespace base
    namespace geometric {
        class PathGeometric;
    }  // namespace geometric
}  // namespace ompl

namespace grstapse {
    // Forward Declarations
    class OmplMotionPlannerParameters;

    class OmplEnvironment;

    class ConfigurationBase;

    //! An enumeration of motion planner algorithms
    enum class OmplMotionPlannerType : uint8_t {
        e_unknown = 0,
        e_prm,
        e_prm_star,
        e_lazy_prm,
        e_lazy_prm_star,
        e_rrt,
        e_rrt_star,
        e_parallel_rrt,
        e_rrt_connect,
        e_lazy_rrt
    };

    NLOHMANN_JSON_SERIALIZE_ENUM(OmplMotionPlannerType,
                                 {
                                     { OmplMotionPlannerType::e_unknown, "unknown" },
                                     { OmplMotionPlannerType::e_prm, "prm" },
                                     { OmplMotionPlannerType::e_prm_star, "prm_star" },
                                     { OmplMotionPlannerType::e_lazy_prm, "lazy_prm" },
                                     { OmplMotionPlannerType::e_lazy_prm_star, "lazy_prm_star" },
                                     { OmplMotionPlannerType::e_rrt, "rrt" },
                                     { OmplMotionPlannerType::e_rrt_star, "rrt_star" },
                                     { OmplMotionPlannerType::e_parallel_rrt, "parallel_rrt" },
                                     { OmplMotionPlannerType::e_rrt_connect, "rrt_connect" },
                                     { OmplMotionPlannerType::e_lazy_rrt, "lazy_rrt" }
                                 });

    /**!
     *  Conducts motion planning by wrapping several classes from the Open Motion Planning Library
     *
     *  \cite I. Șucan, M. Moll, and L. Kavraki, "The Open Motion Planning Library",
     *        IEEE Robotics & Automation Magazine, 19(4):72–82, December 2012. https://ompl.kavrakilab.org
     */
    class OmplMotionPlanner : public MotionPlannerBase {

    public:
        //! \Constructor
        OmplMotionPlanner(OmplMotionPlannerType ompl_motion_planner_type,
                          const std::shared_ptr<const OmplMotionPlannerParameters> &parameters,
                          const std::shared_ptr<OmplEnvironment> &environment);

        //! \returns A pointer to the space information
        [[nodiscard]] const std::shared_ptr<ompl::base::SpaceInformation> &spaceInformation() const;

        //! \returns The type of motion planning algorithm used
        [[nodiscard]] inline OmplMotionPlannerType omplMotionPlannerType() const;

    protected:
        //! Computes a motion plan using an OMPL motion planner
        [[nodiscard]] std::shared_ptr<const MotionPlanningQueryResultBase> computeMotionPlan(
                const std::shared_ptr<const Species> &species,
                const std::shared_ptr<const ConfigurationBase> &initial_configuration,
                const std::shared_ptr<const ConfigurationBase> &goal_configuration) final override;

        OmplMotionPlannerType m_ompl_motion_planner_type;
        std::unique_ptr<ompl::geometric::SimpleSetup> m_simple_setup;
    };

    // Inline Functions
    OmplMotionPlannerType OmplMotionPlanner::omplMotionPlannerType() const {
        return m_ompl_motion_planner_type;
    }
}  // namespace grstapse