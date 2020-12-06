#ifndef MODULE_VISION_GREENHORIZONDETECTOR_HPP
#define MODULE_VISION_GREENHORIZONDETECTOR_HPP

#include <Eigen/Core>
#include <nuclear>
#include <vector>

namespace module {
namespace vision {

    class GreenHorizonDetector : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the GreenHorizonDetector reactor.
        explicit GreenHorizonDetector(std::unique_ptr<NUClear::Environment> environment);

    private:
        struct {
            float confidence_threshold;
            uint cluster_points;
            float distance_offset;
            bool debug;
        } config;
    };

}  // namespace vision
}  // namespace module

#endif  // MODULE_VISION_GREENHORIZONDETECTOR_HPP