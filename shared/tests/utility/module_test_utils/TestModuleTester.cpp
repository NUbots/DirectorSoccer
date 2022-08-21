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

#include "utility/module_test_utils/ModuleTester.hpp"

using utility::module_test::ModuleTester;

namespace {
    class DummyReactor : public NUClear::Reactor {
    public:
        explicit DummyReactor(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

            // Reaction to stop test after 1 second delay
            on<Every<1, std::chrono::seconds>>().then([this]() {
                if (is_first) {
                    is_first = false;
                    return;
                }
                powerplant.shutdown();
            });
        }
        bool is_first = true;
    };
}  // namespace

TEST_CASE("Testing that tests don't timeout if they complete in time",
          "[shared][utility][module_test_utils][ModuleTester][TimeoutModule]") {

    static constexpr int NUM_THREADS = 1;

    // Give the test 5 seconds before it times out
    ModuleTester<DummyReactor, 5, std::chrono::seconds> tester(NUM_THREADS);

    tester.run();
}

TEST_CASE("Testing that tests fail if the timeout finishes",
          "[shared][utility][module_test_utils][ModuleTester][TimeoutModule][!shouldfail]") {

    static constexpr int NUM_THREADS = 1;

    // Set the timeout to be 1 nanosecond - should fail
    ModuleTester<DummyReactor, 1, std::chrono::nanoseconds> tester(NUM_THREADS);

    tester.run();
}

namespace {
    // It's a global, but DO NOT USE IT IN OTHER TEST CASES
    std::vector<std::string> install_log;
}  // namespace

TEST_CASE("Modules install in the right order", "[shared][utility][module_test_utils][ModuleTester]") {
    // When we have modules which are dependencies of the test, we need to install them before calling tester.run().
    // This test makes sure that these dependencies are installed properly and in the right order

    class FirstToInstall : public NUClear::Reactor {
    public:
        explicit FirstToInstall(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
            on<Startup>().then([] { install_log.push_back("FirstToInstall"); });
        }
    };

    class SecondToInstall : public NUClear::Reactor {
    public:
        explicit SecondToInstall(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
            on<Startup>().then([this] {
                install_log.push_back("SecondToInstall");
                powerplant.shutdown();
            });
        }
    };

    static constexpr int NUM_THREADS = 1;

    ModuleTester<DummyReactor> tester(NUM_THREADS);

    tester.install<FirstToInstall>("FirstToInstall");
    tester.install<SecondToInstall>("SecondToInstall");

    tester.run();

    REQUIRE(install_log.at(0) == "FirstToInstall");
    REQUIRE(install_log.at(1) == "SecondToInstall");
}