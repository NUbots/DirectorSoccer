#include "FieldLineDetector.hpp"

#include "extension/Configuration.hpp"

namespace module::vision {

    using extension::Configuration;

    FieldLineDetector::FieldLineDetector(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), config{} {

        on<Configuration>("FieldLineDetector.yaml").then([this](const Configuration& cfg) {
            // Use configuration here from file FieldLineDetector.yaml
            this->log_level = cfg["log_level"].as<NUClear::LogLevel>();
        });

        on<Trigger<GreenHorizon>, With<FieldDescription>>().then(
            "Visual Mesh",
            [this](const GreenHorizon& horizon, const FieldDescription& field) {
                // Create lines from each set of points
                // Cluster lines that are close enough together
                // Remove clusters that are too small
                // For each cluster that passes, get the average line and then that cluster
                // is defined by that line equation and all its points.
                // Make sure points can be in multiple lines
                // Find any points that are apart of more than one line cluster
                //      Two lines makes an L shape
                //      Three lines makes a T shape
                // How to deal with center circle? Points that were thrown out, try to make a circle out of them?


                // Other idea
                // Try to make lines, check what points are close on the line, if too few are then try with a different
                // pair Keep going until ??? not all points will be on lines because of center circle...

                // Convenience variables
                const auto& cls                                     = horizon.mesh->classifications;
                const auto& neighbours                              = horizon.mesh->neighbourhood;
                const Eigen::Matrix<float, 3, Eigen::Dynamic>& rays = horizon.mesh->rays;
                const int FIELD_LINE_INDEX                          = horizon.class_map.at("line");


                // Emit a goals message to see the lines in nusight :D
            });
    }

}  // namespace module::vision
