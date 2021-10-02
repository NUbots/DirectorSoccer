#include "FieldLineDetector.hpp"

#include "extension/Configuration.hpp"

namespace module::vision {

using extension::Configuration;

FieldLineDetector::FieldLineDetector(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), config{} {

    on<Configuration>("FieldLineDetector.yaml").then([this](const Configuration& cfg) {
        // Use configuration here from file FieldLineDetector.yaml
        this->log_level = cfg["log_level"].as<NUClear::LogLevel>();
    });
}

}  // namespace module::vision
