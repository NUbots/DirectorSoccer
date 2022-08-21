/*
 * This file is part of NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2022 NUbots <nubots@nubots.net>
 */

#include <catch.hpp>
#include <nuclear>
#include <nuclear>
#include "utility/module_test_utils/ModuleTester.hpp"


using utility::module_test::ModuleTester;

TEST_CASE("Testing that tests fail if the timeout finishes", "[shared][utility][module_test_utils][ModuleTester][TimeoutModule][!shouldfail]") {

    class DummyReactor : public NUClear::Reactor {
    public:
        explicit DummyReactor(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {};
    };

    static constexpr int NUM_THREADS = 1;

    ModuleTester<DummyReactor, 1, std::chrono::nanoseconds> tester(NUM_THREADS);

    tester.run();
}

