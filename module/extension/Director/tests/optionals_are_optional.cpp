#include <catch.hpp>
#include <nuclear>

#include "Director.hpp"
#include "TestBase.hpp"
#include "util/diff_string.hpp"

// Anonymous namespace to avoid name collisions
namespace {

    template <int i>
    struct SimpleTask {
        SimpleTask(const std::string& msg_) : msg(msg_) {}
        std::string msg;
    };
    struct ComplexTask {
        ComplexTask(const std::string& msg_) : msg(msg_) {}
        std::string msg;
    };
    struct BlockerTask {};

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment)
            : TestBase<TestReactor>(std::move(environment)) {

            on<Provide<SimpleTask<0>>>().then([this](const SimpleTask<0>& t) {  //
                events.push_back(t.msg);
            });
            on<Provide<SimpleTask<1>>>().then([this](const SimpleTask<1>& t) {  //
                events.push_back(t.msg);
            });

            on<Provide<ComplexTask>>().then([this](const ComplexTask& t) {
                events.push_back("emitting tasks from complex " + t.msg);
                // One required, one optional
                emit<Task>(std::make_unique<SimpleTask<0>>("from complex " + t.msg), 1, true);
                emit<Task>(std::make_unique<SimpleTask<1>>("from complex " + t.msg), 1, false);
            });

            on<Provide<BlockerTask>>().then([this] {  //
                events.push_back("emitting blocker simple task");
                emit<Task>(std::make_unique<SimpleTask<0>>("from blocker"), 10);
            });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                // Emit a blocker task that will use SimpleTask<0> to block the complex task
                events.push_back("emitting blocker task");
                emit<Task>(std::make_unique<BlockerTask>(), 50);
            });
            on<Trigger<Step<2>>, Priority::LOW>().then([this] {
                // Emit the complex task that should be blocked by the blocker task
                events.push_back("emitting low priority complex task");
                emit<Task>(std::make_unique<ComplexTask>("low priority"), 10);
            });
            on<Trigger<Step<3>>, Priority::LOW>().then([this] {
                // Emit another complex task that should have high enough priority to execute over the blocker
                events.push_back("emitting high priority complex task");
                emit<Task>(std::make_unique<ComplexTask>("high priority"), 100);
            });
            on<Startup>().then([this] {
                emit(std::make_unique<Step<1>>());
                emit(std::make_unique<Step<2>>());
                emit(std::make_unique<Step<3>>());
            });
        }
    };

}  // namespace

TEST_CASE("Test that if a task is optional then it does not need to be executed for the other tasks to run",
          "[director][priority][optional][!mayfail]") {

    NUClear::PowerPlant::Configuration config;
    config.thread_count = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "emitting blocker task",
        "emitting blocker simple task",
        "from blocker",
        "emitting low priority complex task",
        "emitting tasks from complex low priority",
        "from complex low priority",
        "emitting high priority complex task",
        "emitting tasks from complex high priority",
        "from complex high priority",
        "from complex high priority",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
