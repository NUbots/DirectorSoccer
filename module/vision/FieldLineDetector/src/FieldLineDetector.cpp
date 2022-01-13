#include "FieldLineDetector.hpp"

#include "extension/Configuration.hpp"

#include "utility/vision/Regression.hpp"

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
                // Create clusters normally
                // perform least squares on them
                // if the distance is too large between the line and some points then we have a T or L or center circle
                //  try circle fit first
                //   if not a circle, partition the points based on the line
                //  perform least squares on each partition
                // combine lines that are essentially the same
                // not sure on: for each line, go through all the points and add them to the line if they fit (to get
                // points on multiple lines)
                //  these are Ls or Ts, figure out which based on if the point is reasonably close to the endpoint of
                //  both lines (L)


                // Convenience variables
                const auto& cls                                     = horizon.mesh->classifications;
                const auto& neighbours                              = horizon.mesh->neighbourhood;
                const Eigen::Matrix<float, 3, Eigen::Dynamic>& rays = horizon.mesh->rays;
                const int FIELD_LINE_INDEX                          = horizon.class_map.at("line");

                // *** SETTING UP CLUSTERS ***

                // Get some indices to partition
                std::vector<int> indices(horizon.mesh->indices.size());
                std::iota(indices.begin(), indices.end(), 0);

                // Partition the indices such that we only have the field line points that dont have field line points
                // surrounding them
                auto boundary = utility::vision::visualmesh::partition_points(
                    indices.begin(),
                    indices.end(),
                    neighbours,
                    [&](const int& idx) {
                        return idx == int(indices.size()) || (cls(BALL_INDEX, idx) >= config.confidence_threshold);
                    });

                // Discard indices that are not on the boundary of the lines
                indices.resize(std::distance(indices.begin(), boundary));

                log<NUClear::DEBUG>(fmt::format("Partitioned {} points", indices.size()));

                // Cluster the field line points
                // These clusters can then be processed to determine where the lines are
                std::vector<std::vector<int>> clusters;
                utility::vision::visualmesh::cluster_points(indices.begin(),
                                                            indices.end(),
                                                            neighbours,
                                                            config.cluster_points,
                                                            clusters);

                log<NUClear::DEBUG>(fmt::format("Found {} clusters", clusters.size()));

                // Get rid of clusters above the green horizon - the field lines won't be above the field
                auto green_boundary = utility::vision::visualmesh::check_green_horizon_side(clusters.begin(),
                                                                                            clusters.end(),
                                                                                            horizon.horizon.begin(),
                                                                                            horizon.horizon.end(),
                                                                                            rays,
                                                                                            false,
                                                                                            true);
                clusters.resize(std::distance(clusters.begin(), green_boundary));

                log<NUClear::DEBUG>(fmt::format("Found {} clusters below green horizon", clusters.size()));

                // No lines?
                if (clusters.empty()) {
                    log<NUClear::DEBUG>("Found no field lines.");
                    return;
                }

                // ** FIT LINES TO CLUSTERS **

                // Loop for each cluster - there may be clusters added if Ts or Ls are split
                while (!clusters.empty()) {
                    // Grab a cluster and delete it
                    cluster = clusters.back();
                    clusters.pop_back();

                    // Grab all of the data points for this cluster in a vector
                    std::vector<Eigen::Vector2f> data_points;
                    for (const auto& idx : cluster) {
                        Eigen::Vector3f ray = horizon->mesh.Hcw * rays.col(idx);
                        data_points.push_back(Eigen::Vector2f(ray.x(), ray.y()));
                    }
                    auto result = utility::vision::linreg(data_points);

                    // Returned false, ie it broke...
                    if (!result.first) {
                        // do something...
                        continue;
                    }
                    // Get the info
                    float slope                             = 0.0;
                    float intercept                         = 0.0;
                    float coefficient                       = 0.0;
                    std::tie(slope, intercept, coefficient) = result.second;

                    // If coefficient is close enough to 1, then make a line out of it
                    // If not, try for a circle
                    // If not a circle, try splitting it. Add the split to the clusters to be dealt with in another loop
                }

                // Emit a goals message to see the lines in nusight :D
            });
    }

}  // namespace module::vision
