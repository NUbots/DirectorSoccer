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
    template <int i>
    struct ComplexTask {};

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment)
            : TestBase<TestReactor>(std::move(environment)) {

            // First complex task needs both simple tasks
            on<Provide<ComplexTask<1>>, Needs<SimpleTask<1>>, Needs<SimpleTask<2>>>().then([this] {
                events.push_back("emitting complex task 1");
                emit<Task>(std::make_unique<SimpleTask<1>>("complex 1"));
                emit<Task>(std::make_unique<SimpleTask<2>>("complex 1"));
            });
            // Second complex task only needs one
            on<Provide<ComplexTask<2>>, Needs<SimpleTask<1>>>().then([this] {
                events.push_back("emitting complex task 2");
                emit<Task>(std::make_unique<SimpleTask<1>>("complex 2"));
            });

            // Monitor which events run and by who
            on<Provide<SimpleTask<1>>>().then([this](const SimpleTask<1>& t) { events.push_back("p1: " + t.msg); });
            on<Provide<SimpleTask<2>>>().then([this](const SimpleTask<2>& t) { events.push_back("p2: " + t.msg); });
            on<Start<SimpleTask<1>>>().then([this](const SimpleTask<1>& t) { events.push_back("start 1: " + t.msg); });
            on<Start<SimpleTask<2>>>().then([this](const SimpleTask<2>& t) { events.push_back("start 2: " + t.msg); });
            on<Stop<SimpleTask<1>>>().then([this](const SimpleTask<1>& t) { events.push_back("stop 1: " + t.msg); });
            on<Stop<SimpleTask<2>>>().then([this](const SimpleTask<2>& t) { events.push_back("stop 2: " + t.msg); });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                // Emit a complex task that uses both simple tasks
                events.push_back("requesting complex task 1");
                emit<Task>(std::make_unique<ComplexTask<1>>(), 50);
            });
            on<Trigger<Step<2>>, Priority::LOW>().then([this] {
                // Emit a higher priority complex task that just uses one
                events.push_back("requesting complex task 2");
                emit<Task>(std::make_unique<ComplexTask<2>>(), 100);
            });
            on<Startup>().then([this] {
                emit(std::make_unique<Step<1>>());
                emit(std::make_unique<Step<2>>());
            });
        }
    };

}  // namespace

TEST_CASE("Tests that if a provider loses one of its dependent needs it stops running everything",
          "[director][priority][needs][!mayfail]") {

    NUClear::PowerPlant::Configuration config;
    config.thread_count = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "requesting complex task 1",
        "emitting complex task 1",
        "start 1: complex 1",
        "start 2: complex 1",
        "p1: complex 1",
        "p2: complex 1",
        "requesting complex task 2",
        "stop 2: complex 1",
        "emitting complex task 2",
        "p1: complex 2",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
