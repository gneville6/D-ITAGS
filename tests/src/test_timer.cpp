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

// Global
#include <chrono>
#include <thread>
// External
#include <gtest/gtest.h>
// Project
#include <grstapse/common/utilities/timer.hpp>

namespace grstapse::unittests
{
    TEST(Timer, Simple)
    {
        Timer timer;

        timer.start();
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        timer.stop();
        ASSERT_NEAR(timer.get(), 0.25f, 1e-3f);

        timer.start();
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        timer.stop();
        ASSERT_NEAR(timer.get(), 0.5f, 1e-3f);

        timer.reset();
        ASSERT_NEAR(timer.get(), 0.0f, 1e-3f);

        timer.start();
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        timer.stop();
        ASSERT_NEAR(timer.get(), 0.25f, 1e-3f);
    }

    TEST(Timer, Loop)
    {
        Timer timer;
        Timer timer2;
        timer2.start();
        for(unsigned int i = 0; i < 100; ++i)
        {
            timer.start();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            timer.stop();
        }
        timer2.stop();
        ASSERT_NEAR(timer.get(), timer2.get(), 1e-3);
    }
}  // namespace grstapse::unittests