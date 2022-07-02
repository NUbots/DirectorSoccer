// /*
//  * This file is part of the NUbots Codebase.
//  *
//  * The NUbots Codebase is free software: you can redistribute it and/or modify
//  * it under the terms of the GNU General Public License as published by
//  * the Free Software Foundation, either version 3 of the License, or
//  * (at your option) any later version.
//  *
//  * The NUbots Codebase is distributed in the hope that it will be useful,
//  * but WITHOUT ANY WARRANTY; without even the implied warranty of
//  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  * GNU General Public License for more details.
//  *
//  * You should have received a copy of the GNU General Public License
//  * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
//  *
//  * Copyright 2013 NUbots <nubots@nubots.net>
//  */

// #include "FallingRelax.hpp"

// #include <cmath>

// #include "extension/Configuration.hpp"
// #include "extension/Script.hpp"

// #include "message/behaviour/ServoCommand.hpp"
// #include "message/input/Sensors.hpp"

// #include "utility/behaviour/Action.hpp"
// #include "utility/input/LimbID.hpp"
// #include "utility/input/ServoID.hpp"

// namespace module::behaviour::skills {

//     // internal only callback messages to start and stop our action
//     struct Falling {};
//     struct KillFalling {};

//     using extension::Configuration;
//     using extension::ExecuteScriptByName;

//     using message::input::Sensors;

//     using utility::behaviour::ActionPriorities;
//     using utility::behaviour::RegisterAction;
//     using utility::input::LimbID;
//     using utility::input::ServoID;

//     FallingRelax::FallingRelax(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

//         // do a little configurating
//         on<Configuration>("FallingRelax.yaml").then([this](const Configuration& config) {
//             log_level          = config["log_level"].as<NUClear::LogLevel>();
//             // Store falling angle as a cosine so we can compare it directly to the z axis value
//             const auto fallingAngle = config["FALLING_ANGLE"].as<float>();
//             FALLING_ANGLE           = std::cos(fallingAngle);

//             // When falling the acceleration should drop below this value
//             FALLING_ACCELERATION = config["FALLING_ACCELERATION"].as<float>();

//             // Once the acceleration has stabalized, we are no longer falling
//             RECOVERY_ACCELERATION = config["RECOVERY_ACCELERATION"].as<std::vector<float>>();

//             PRIORITY = config["PRIORITY"].as<float>();
//             log<NUClear::INFO>("FallingRelax - Config");
//         });

// //         on<Trigger<Sensors>>([this](const Sensors& sensors) {
//             log<NUClear::INFO>("FallingRelax - H1");
//             if (!falling && !sensors.empty() && fabs(sensors.back()->Htw(2, 2)) < FALLING_ANGLE) {

//                 // We might be falling, check the accelerometer
//                 double magnitude = 0;

//                 for (const auto& sensor : sensors) {
//                     magnitude += sensor->accelerometer.norm();
//                 }

//                 magnitude /= sensors.size();

//                 if (magnitude < FALLING_ACCELERATION) {
//                     falling = true;
//                     updatePriority(PRIORITY);
//                 }
//             }
//             else if (falling) {
//                 // We might be recovered, check the accelerometer
//                 double magnitude = 0;

//                 for (const auto& sensor : sensors) {
//                     magnitude += sensor->accelerometer.norm();
//                 }

//                 magnitude /= sensors.size();

//                 // See if we recover
//                 if (magnitude > RECOVERY_ACCELERATION[0] && magnitude < RECOVERY_ACCELERATION[1]) {
//                     falling = false;
//                     updatePriority(0);
//                 }
//             }
//         });

//         on<Trigger<Falling>>().then([this] {
//             NUClear::log<NUClear::INFO>("FallingRelax - H2");
//             emit(std::make_unique<ExecuteScriptByName>(id, "Relax.yaml"));
//         });

//         on<Trigger<KillFalling>>().then([this] {
//             falling = false;
//             updatePriority(0);
//         });

//         emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction{
//             id,
//             "Falling Relax",
//             {std::pair<float, std::set<LimbID>>(
//                 0.0f,
//                 {LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM, LimbID::HEAD})},
//             [this](const std::set<LimbID>& /*unused*/) { emit(std::make_unique<Falling>()); },
//             [this](const std::set<LimbID>& /*unused*/) { emit(std::make_unique<KillFalling>()); },
//             [](const std::set<ServoID>& /*unused*/) {
//                 // Ignore
//             }}));
//     }

//     void FallingRelax::updatePcfg.fallen_angle   = config["fallen_angle"].as<float>();
//             cfg.getup_priority = config["getup_priority"].as<float>();


// /*
//  * This file is part of the NUbots Codebase.
//  *
//  * The NUbots Codebase is free software: you can redistribute it and/or modify
//  * it under the terms of the GNU General Public License as published by
//  * the Free Software Foundation, either version 3 of the License, or
//  * (at your option) any later version.
//  *
//  * The NUbots Codebase is distributed in the hope that it will be useful,
//  * but WITHOUT ANY WARRANTY; without even the implied warranty of
//  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  * GNU General Public License for more details.
//  *
//  * You should have received a copy of the GNU General Public License
//  * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
//  *
//  * Copyright 2013 NUbots <nubots@nubots.net>
//  */


// #include "FallingRelax.hpp"

// #include <Eigen/Core>
// #include <Eigen/Geometry>
// #include <cmath>

// #include "extension/Configuration.hpp"
// #include "extension/Script.hpp"

// #include "message/behaviour/ServoCommand.hpp"
// #include "message/input/Sensors.hpp"
// #include "message/motion/FallingRelaxCommand.hpp"

// #include "utility/behaviour/Action.hpp"
// #include "utility/input/LimbID.hpp"

// namespace module::behaviour::skills {

//     using extension::Configuration;
//     using extension::ExecuteScriptByName;

//     using message::input::Sensors;
//     using message::motion::ExecuteFallingRelax;
//     using message::motion::KillFallingRelax;

//     using utility::behaviour::ActionPriorities;
//     using utility::behaviour::RegisterAction;
//     using LimbID  = utility::input::LimbID;
//     using ServoID = utility::input::ServoID;

//     FallingRelax::FallingRelax(std::unique_ptr<NUClear::Environment> environment)
//         : Reactor(std::move(environment)), subsumption_id(size_t(this) * size_t(this) - size_t(this)) {
//         // do a little configurating
//         on<Configuration>("FallingRelax.yaml").then([this](const Configuration& config) {
//             log<NUClear::INFO>("FallingRelax - Config");
//             log_level          = config["log_level"].as<NUClear::LogLevel>();
//             cfg.falling_angle   = config["falling_angle"].as<float>();
//             // cfg.recovery_angle   = config["recovery_angle"].as<float>();
//             cfg.relax_fall_priority = config["relax_fall_priority"].as<float>();

//         });

//         emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction{
//             subsumption_id,
//             "Falling Relax",
//             {std::pair<float, std::set<LimbID>>(
//                 0,
//                 {LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM, LimbID::HEAD})},
//             [this](const std::set<LimbID>& /* limbs */) { emit(std::make_unique<ExecuteFallingRelax>()); },
//             [this](const std::set<LimbID>& /* limbs */) { emit(std::make_unique<KillFallingRelax>()); },
//             [this](const std::set<ServoID>& /* servos */) { emit(std::make_unique<KillFallingRelax>()); }}));

//         on<Trigger<Sensors>>().then("FallingRelax Fallen Check", [this](const Sensors& sensors) {
//             // Transform to torso {t} from world {w} space
//             Eigen::Matrix4d Hwt = sensors.Htw.inverse();
//             // Basis Z vector of torso {t} in world {w} space
//             Eigen::Vector3d uZTw = Hwt.block(0, 2, 3, 1);
//             // Basis X vector of torso {t} in world {w} space
//             Eigen::Vector3d uXTw = Hwt.block(0, 0, 3, 1);

//             // Check if angle between torso z axis and world z axis is greater than config value cfg.falling_angle
//             if (!relaxed && std::acos(Eigen::Vector3d::UnitZ().dot(uZTw)) > cfg.falling_angle) {
//                 update_priority(cfg.relax_fall_priority);
//             }
//         });

//         on<Trigger<ExecuteFallingRelax>, Single>().then("Execute Relax", [this]() {
//             relaxed = true;
//             log<NUClear::INFO>("Execute");
//             emit(std::make_unique<ExecuteScriptByName>(
//                     subsumption_id,
//                     std::vector<std::string>({"Relax.yaml"})));

//         });

//         on<Trigger<KillFallingRelax>>().then([this] {
//             log<NUClear::INFO>("Kill");
//             relaxed = false;
//             update_priority(0);
//         });
//     }

//     void FallingRelax::update_priority(const float& priority) {
//         emit(std::make_unique<ActionPriorities>(ActionPriorities{subsumption_id, {priority}}));
//     }

// }  // namespace module::behaviour::skills


/*
 * This file is part of the NUbots Codebase.
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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#include "FallingRelax.hpp"

#include <cmath>

#include "extension/Configuration.hpp"
#include "extension/Script.hpp"

#include "message/behaviour/ServoCommand.hpp"
#include "message/input/Sensors.hpp"
#include "message/motion/FallingRelaxCommand.hpp"

#include "utility/behaviour/Action.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"

namespace module::behaviour::skills {

    // internal only callback messages to start and stop our action
    // struct Falling {};
    // struct KillFalling {};

    using extension::Configuration;
    using extension::ExecuteScriptByName;

    using message::input::Sensors;
    using message::motion::ExecuteFallingRelax;
    using message::motion::KillFallingRelax;

    using utility::behaviour::ActionPriorities;
    using utility::behaviour::RegisterAction;
    using utility::input::LimbID;
    using utility::input::ServoID;

    FallingRelax::FallingRelax(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), subsumption_id(size_t(this) * size_t(this) - size_t(this)) {
        // do a little configurating
        on<Configuration>("FallingRelax.yaml").then([this](const Configuration& config) {
            log<NUClear::INFO>("FallingRelax - Config");
            log_level          = config["log_level"].as<NUClear::LogLevel>();
            const auto fallingAngle   = config["falling_angle"].as<float>();
            cfg.falling_angle   = std::cos(fallingAngle);
            cfg.falling_acceleration   = config["falling_acceleration"].as<float>();
            cfg.recovery_acceleration   = config["recovery_acceleration"].as<std::vector<float>>();
            cfg.relax_fall_priority = config["relax_fall_priority"].as<float>();

        });

        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction{
            subsumption_id,
            "Falling Relax",
            {std::pair<float, std::set<LimbID>>(
                0,
                {LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM, LimbID::HEAD})},
            [this](const std::set<LimbID>& /* limbs */) { emit(std::make_unique<ExecuteFallingRelax>()); },
            [this](const std::set<LimbID>& /* limbs */) { emit(std::make_unique<KillFallingRelax>()); },
            [this](const std::set<ServoID>& /* servos */) { emit(std::make_unique<KillFallingRelax>()); }}));

        on<Last<5, Trigger<Sensors>>, Single>().then("FallingRelax Fallen Check",
            [this](const std::list<std::shared_ptr<const Sensors>>& sensors) {

            //Transform to torso {t} from world {w} space
            Eigen::Matrix4d Hwt = sensors.back()->Htw.inverse();
            // Basis Z vector of torso {t} in world {w} space
            Eigen::Vector3d uZTw = Hwt.block(0, 2, 3, 1);

            double magnitude = 0;
            for (const auto& sensor : sensors) {
                    magnitude += sensor->accelerometer.norm();
                }
            magnitude /= sensors.size();

            NUClear::log<NUClear::DEBUG>("Gravity is ", magnitude);
            NUClear::log<NUClear::DEBUG>("Z angle is ", Eigen::Vector3d::UnitZ().dot(uZTw));

            //if (!relaxed && !sensors.empty() && fabs(sensors.back()->Htw(2, 2)) < cfg.falling_angle) {
            if (!relaxed && !sensors.empty() && acos(Eigen::Vector3d::UnitZ().dot(uZTw)) > cfg.falling_angle) {
                NUClear::log<NUClear::DEBUG>("Inside");

                // We might be falling, check the accelerometer
                double magnitude = 0;

                for (const auto& sensor : sensors) {
                    magnitude += sensor->accelerometer.norm();
                }

                magnitude /= sensors.size();

                if (magnitude < cfg.falling_acceleration) {
                    relaxed = true;
                    update_priority(cfg.relax_fall_priority);
                    NUClear::log<NUClear::DEBUG>("Magnitude is less than gravity");
                    NUClear::log<NUClear::DEBUG>("Gravity magnitude was ", magnitude);
                }
            }
            // else if (relaxed && Eigen::Vector3d::UnitZ().dot(uZTw) < cfg.falling_angle) {
            //     // We might be recovered, check the accelerometer
            //     double magnitude = 0;

            //     for (const auto& sensor : sensors) {
            //         magnitude += sensor->accelerometer.norm();
            //     }

            //     magnitude /= sensors.size();

            //     // See if we recover
            //     if (magnitude > cfg.recovery_acceleration[0] && magnitude < cfg.recovery_acceleration[1]) {
            //         relaxed = false;
            //         update_priority(0);
            //     }
            // }
        });
        on<Trigger<ExecuteFallingRelax>, Single>().then("Execute Relax", [this]() {
            relaxed = true;
            emit(std::make_unique<ExecuteScriptByName>(
                    subsumption_id,
                    std::vector<std::string>({"Relax.yaml"})));

        });

        on<Trigger<KillFallingRelax>>().then([this] {
            relaxed = false;
            update_priority(0);
        });
    }

        void FallingRelax::update_priority(const float& priority) {
            emit(std::make_unique<ActionPriorities>(ActionPriorities{subsumption_id, {priority}}));
        }

}  // namespace module::behaviour::skills
