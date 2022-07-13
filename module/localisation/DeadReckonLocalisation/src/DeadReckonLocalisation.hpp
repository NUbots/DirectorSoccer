#ifndef MODULE_LOCALISATION_DEADRECKONLOCALISATION_HPP
#define MODULE_LOCALISATION_DEADRECKONLOCALISATION_HPP

#include <nuclear>

#include "message/input/Sensors.hpp"
#include "message/vision/Ball.hpp"


namespace module::localisation {
    using VisionBalls = message::vision::Balls;
    using VisionBall  = message::vision::Ball;
    using message::input::Sensors;

    class DeadReckonLocalisation : public NUClear::Reactor {
    private:
        struct Config {
            Config()               = default;
            float smoothing_factor = 0.0f;
        } cfg;

        /// @brief Time between sensor measurement updates
        // float dt = 1 / 90;

        /// @brief Rotation clockwise in field space {f} z axis
        float theta = 0;

        /// @brief Filtered rotation clockwise in field space {f} z axis
        float filtered_theta = 0;

        /// @brief Time of last sensor measurement update
        NUClear::clock::time_point last_update = NUClear::clock::now();

        /// @brief Time since startup (in seconds)
        float time_since_startup = 0;

        /// @brief Accurate flag
        bool accurate = true;

        /// @brief Bool to indicate if the robot is currently getting up
        bool is_getting_up = false;


    public:
        /// @brief Called by the powerplant to build and setup the DeadReckonLocalisation reactor.
        explicit DeadReckonLocalisation(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module::localisation

#endif  // MODULE_LOCALISATION_BALLLOCALISATION_HPP
