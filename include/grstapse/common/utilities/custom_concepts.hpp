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

namespace grstapse
{
    /**!
     * Declaration of the concept "Hashable", which is satisfied by any type 'T'
     * such that for values 'a' of type 'T', the expression std::hash<T>{}(a)
     * compiles and its result is convertible to std::size_t
     *
     * \tparam T
     */
    template <typename T>
    concept Hashable = requires(T a)
    {
        {
            std::hash<T>{}(a)
            } -> std::convertible_to<std::size_t>;
    };

    namespace __detail
    {
        template <typename _Tp, typename _Up>
        concept __weakly_lt_cmp_with = requires(std::__detail::__cref<_Tp> __t, std::__detail::__cref<_Up> __u)
        {
            {
                __t < __u
                } -> std::__detail::__boolean_testable;
            {
                __u < __t
                } -> std::__detail::__boolean_testable;
        };
    }  // namespace __detail

    template <typename _Tp>
    concept LessThanComparable = __detail::__weakly_lt_cmp_with<_Tp, _Tp>;

    namespace __detail
    {
        template <typename _Tp, typename _Up>
        concept __weakly_gt_cmp_with = requires(std::__detail::__cref<_Tp> __t, std::__detail::__cref<_Up> __u)
        {
            {
                __t > __u
                } -> std::__detail::__boolean_testable;
            {
                __u > __t
                } -> std::__detail::__boolean_testable;
        };
    }  // namespace __detail

    template <typename _Tp>
    concept GreaterThanComparable = __detail::__weakly_gt_cmp_with<_Tp, _Tp>;

    namespace __detail
    {
        template <typename _Tp, typename _Up>
        concept __weakly_lte_cmp_with = requires(std::__detail::__cref<_Tp> __t, std::__detail::__cref<_Up> __u)
        {
            {
                __t <= __u
                } -> std::__detail::__boolean_testable;
            {
                __u <= __t
                } -> std::__detail::__boolean_testable;
        };
    }  // namespace __detail

    template <typename _Tp>
    concept LessThanEqualComparable = __detail::__weakly_lte_cmp_with<_Tp, _Tp>;

    namespace __detail
    {
        template <typename _Tp, typename _Up>
        concept __weakly_gte_cmp_with = requires(std::__detail::__cref<_Tp> __t, std::__detail::__cref<_Up> __u)
        {
            {
                __t >= __u
                } -> std::__detail::__boolean_testable;
            {
                __u >= __t
                } -> std::__detail::__boolean_testable;
        };
    }  // namespace __detail

    template <typename _Tp>
    concept GreaterThanEqualComparable = __detail::__weakly_gte_cmp_with<_Tp, _Tp>;

    namespace __detail
    {
        template <typename _Tp, typename _Up>
        concept __weakly_cmp_with = requires(std::__detail::__cref<_Tp> __t, std::__detail::__cref<_Up> __u)
        {
            {
                __t < __u
                } -> std::__detail::__boolean_testable;
            {
                __t > __u
                } -> std::__detail::__boolean_testable;
            {
                __t <= __u
                } -> std::__detail::__boolean_testable;
            {
                __t >= __u
                } -> std::__detail::__boolean_testable;
            {
                __t == __u
                } -> std::__detail::__boolean_testable;
            {
                __t != __u
                } -> std::__detail::__boolean_testable;
            {
                __u < __t
                } -> std::__detail::__boolean_testable;
            {
                __u > __t
                } -> std::__detail::__boolean_testable;
            {
                __u <= __t
                } -> std::__detail::__boolean_testable;
            {
                __u >= __t
                } -> std::__detail::__boolean_testable;
            {
                __u == __t
                } -> std::__detail::__boolean_testable;
            {
                __u != __t
                } -> std::__detail::__boolean_testable;
        };
    }  // namespace __detail

    template <typename _Tp>
    concept Comparable = __detail::__weakly_cmp_with<_Tp, _Tp>;
}  // namespace grstapse