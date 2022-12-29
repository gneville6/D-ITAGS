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
#include "grstapse/common/utilities/custom_json_conversions.hpp"

// External
#include <magic_enum/magic_enum.hpp>
// Local
#include "grstapse/common/utilities/error.hpp"
#include "grstapse/common/utilities/json_field_validator.hpp"
#include "grstapse/geometric_planning/ompl/ompl_enums.hpp"

namespace nlohmann
{
    ompl::base::RealVectorBounds adl_serializer<ompl::base::RealVectorBounds>::from_json(const json& j)
    {
        grstapse::validate(j,
                           {{grstapse::constants::k_low, nlohmann::json::value_t::array},
                            {grstapse::constants::k_high, nlohmann::json::value_t::array}});

        const nlohmann::json& low  = j.at(grstapse::constants::k_low);
        const nlohmann::json& high = j.at(grstapse::constants::k_high);
        assert(low.size() == high.size());

        ompl::base::RealVectorBounds b(low.size());
        low.get_to(b.low);
        high.get_to(b.high);

        return b;
    }

    void adl_serializer<ompl::base::RealVectorBounds>::to_json(json& j, const ompl::base::RealVectorBounds& b)
    {
        j = {{grstapse::constants::k_low, b.low}, {grstapse::constants::k_high, b.high}};
    }

    void adl_serializer<ompl::base::SE2StateSpace::StateType>::from_json(const json& j,
                                                                         ompl::base::SE2StateSpace::StateType& s)
    {
        grstapse::validate(j,
                           {
                               {grstapse::constants::k_x, nlohmann::json::value_t::number_float},
                               {grstapse::constants::k_y, nlohmann::json::value_t::number_float},
                               {grstapse::constants::k_yaw, nlohmann::json::value_t::number_float},
                           });

        // Setup state because OMPL doesn't...
        s.components    = new ompl::base::State*[2];
        s.components[0] = new ompl::base::RealVectorStateSpace::StateType();
        s.components[0]->as<ompl::base::RealVectorStateSpace::StateType>()->values = new double[2];
        s.components[1] = new ompl::base::SO2StateSpace::StateType();

        s.setX(j.at(grstapse::constants::k_x).get<float>());
        s.setY(j.at(grstapse::constants::k_y).get<float>());
        s.setYaw(j.at(grstapse::constants::k_yaw).get<float>());
    }

    void adl_serializer<ompl::base::SE2StateSpace::StateType>::to_json(json& j,
                                                                       const ompl::base::SE2StateSpace::StateType& s)
    {
        j = {{grstapse::constants::k_state_type, grstapse::OmplStateSpaceType::e_se2},
             {grstapse::constants::k_x, s.getX()},
             {grstapse::constants::k_y, s.getY()},
             {grstapse::constants::k_yaw, s.getYaw()}};
    }

    void adl_serializer<ompl::base::SE2StateSpace>::from_json(const json& j, ompl::base::SE2StateSpace& s)
    {
        grstapse::validate(j, {{grstapse::constants::k_bounds, nlohmann::json::value_t::object}});
        s.setBounds(j.at(grstapse::constants::k_bounds).get<ompl::base::RealVectorBounds>());
    }

    void adl_serializer<ompl::base::SE2StateSpace>::to_json(json& j, const ompl::base::SE2StateSpace& s)
    {
        j = {{grstapse::constants::k_state_type, grstapse::OmplStateSpaceType::e_se2},
             {grstapse::constants::k_bounds, s.getBounds()}};
    }

    void adl_serializer<ompl::base::SE3StateSpace::StateType>::from_json(const json& j,
                                                                         ompl::base::SE3StateSpace::StateType& s)
    {
        grstapse::validate(j,
                           {
                               {grstapse::constants::k_x, nlohmann::json::value_t::number_float},
                               {grstapse::constants::k_y, nlohmann::json::value_t::number_float},
                               {grstapse::constants::k_z, nlohmann::json::value_t::number_float},
                               {grstapse::constants::k_rotation, nlohmann::json::value_t::object},
                           });

        // Setup state because OMPL doesn't...
        s.components    = new ompl::base::State*[2];
        s.components[0] = new ompl::base::RealVectorStateSpace::StateType();
        s.components[0]->as<ompl::base::RealVectorStateSpace::StateType>()->values = new double[3];
        s.components[1] = new ompl::base::SO3StateSpace::StateType();

        s.setX(j.at(grstapse::constants::k_x).get<float>());
        s.setY(j.at(grstapse::constants::k_y).get<float>());
        s.setZ(j.at(grstapse::constants::k_z).get<float>());
        auto rotation =
            j.at(grstapse::constants::k_rotation).get<std::shared_ptr<ompl::base::SO3StateSpace::StateType>>();
        s.rotation().x = rotation->x;
        s.rotation().y = rotation->y;
        s.rotation().z = rotation->z;
        s.rotation().w = rotation->w;
    }

    void adl_serializer<ompl::base::SE3StateSpace::StateType>::to_json(json& j,
                                                                       const ompl::base::SE3StateSpace::StateType& s)
    {
        j = {{grstapse::constants::k_state_type, grstapse::OmplStateSpaceType::e_se3},
             {grstapse::constants::k_x, s.getX()},
             {grstapse::constants::k_y, s.getY()},
             {grstapse::constants::k_z, s.getZ()},
             {grstapse::constants::k_rotation, s.rotation()}};
    }

    void adl_serializer<ompl::base::SE3StateSpace>::from_json(const json& j, ompl::base::SE3StateSpace& s)
    {
        grstapse::validate(j, {{grstapse::constants::k_bounds, nlohmann::json::value_t::object}});
        s.setBounds(j.at(grstapse::constants::k_bounds).get<ompl::base::RealVectorBounds>());
    }

    void adl_serializer<ompl::base::SE3StateSpace>::to_json(json& j, const ompl::base::SE3StateSpace& s)
    {
        j = {{grstapse::constants::k_state_type, grstapse::OmplStateSpaceType::e_se3},
             {grstapse::constants::k_bounds, s.getBounds()}};
    }

    void adl_serializer<ompl::base::SO3StateSpace::StateType>::from_json(const json& j,
                                                                         ompl::base::SO3StateSpace::StateType& s)
    {
        grstapse::validate(j,
                           {
                               {grstapse::constants::k_qx, nlohmann::json::value_t::number_float},
                               {grstapse::constants::k_qy, nlohmann::json::value_t::number_float},
                               {grstapse::constants::k_qz, nlohmann::json::value_t::number_float},
                               {grstapse::constants::k_qw, nlohmann::json::value_t::number_float},
                           });

        j.at(grstapse::constants::k_qx).get_to(s.x);
        j.at(grstapse::constants::k_qy).get_to(s.y);
        j.at(grstapse::constants::k_qz).get_to(s.z);
        j.at(grstapse::constants::k_qw).get_to(s.w);
    }

    void adl_serializer<ompl::base::SO3StateSpace::StateType>::to_json(json& j,
                                                                       const ompl::base::SO3StateSpace::StateType& s)
    {
        j = {{grstapse::constants::k_state_type, magic_enum::enum_name(grstapse::OmplStateSpaceType::e_so3)},
             {grstapse::constants::k_qx, s.x},
             {grstapse::constants::k_qy, s.y},
             {grstapse::constants::k_qz, s.z},
             {grstapse::constants::k_qw, s.w}};
    }

    void adl_serializer<ompl::geometric::PathGeometric>::to_json(json& j, const ompl::geometric::PathGeometric& p)
    {
        j                            = nlohmann::json::array();
        const std::size_t num_states = p.getStateCount();
        if(num_states == 0)
        {
            return;
        }

        grstapse::OmplStateSpaceType state_type = grstapse::OmplStateSpaceType::e_unknown;
        {
            const ompl::base::State* state = p.getState(0);
            nlohmann::json j_state;
            if(const auto* se2_state = dynamic_cast<const ompl::base::SE2StateSpace::StateType*>(state))
            {
                adl_serializer<ompl::base::SE2StateSpace::StateType>::to_json(j_state, *se2_state);
                state_type = grstapse::OmplStateSpaceType::e_se2;
            }
            else if(const auto* se3_state = dynamic_cast<const ompl::base::SE3StateSpace::StateType*>(state))
            {
                adl_serializer<ompl::base::SE3StateSpace::StateType>::to_json(j_state, *se3_state);
                state_type = grstapse::OmplStateSpaceType::e_se3;
            }
            else if(const auto* so3_state = dynamic_cast<const ompl::base::SO3StateSpace::StateType*>(state))
            {
                adl_serializer<ompl::base::SO3StateSpace::StateType>::to_json(j_state, *so3_state);
                state_type = grstapse::OmplStateSpaceType::e_so3;
            }
            else
            {
                throw grstapse::createLogicError("Unknown state type");
            }
            j.push_back(j_state);
        }
        if(state_type == grstapse::OmplStateSpaceType::e_unknown)
        {
            throw grstapse::createLogicError("Unknown state type");
        }

        for(unsigned int i = 1; i < num_states; ++i)
        {
            const ompl::base::State* state = p.getState(i);
            nlohmann::json j_state;
            switch(state_type)
            {
                case grstapse::OmplStateSpaceType::e_se2:
                {
                    adl_serializer<ompl::base::SE2StateSpace::StateType>::to_json(
                        j_state,
                        *dynamic_cast<const ompl::base::SE2StateSpace::StateType*>(state));
                    break;
                }
                case grstapse::OmplStateSpaceType::e_se3:
                {
                    adl_serializer<ompl::base::SE3StateSpace::StateType>::to_json(
                        j_state,
                        *dynamic_cast<const ompl::base::SE3StateSpace::StateType*>(state));
                    break;
                }
                case grstapse::OmplStateSpaceType::e_so3:
                {
                    adl_serializer<ompl::base::SO3StateSpace::StateType>::to_json(
                        j_state,
                        *dynamic_cast<const ompl::base::SO3StateSpace::StateType*>(state));
                    break;
                }
            }
            j.push_back(j_state);
        }
    }
}  // namespace nlohmann