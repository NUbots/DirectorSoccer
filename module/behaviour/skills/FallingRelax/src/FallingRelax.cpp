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
#include "message/support/nusight/DataPoint.hpp"

#include "utility/behaviour/Action.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/nusight/NUhelpers.hpp"

namespace module::behaviour::skills {

    // internal only callback messages to start and stop our action
    struct Falling {};
    struct KillFalling {};

    using extension::Configuration;
    using extension::ExecuteScriptByName;

    using message::input::Sensors;

    using utility::behaviour::ActionPriorities;
    using utility::behaviour::RegisterAction;

    using utility::input::LimbID;
    using utility::input::ServoID;
    using utility::nusight::graph;

    FallingRelax::FallingRelax(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), subsumption_id(size_t(this) * size_t(this) - size_t(this)) {

        // do a little configurating
        on<Configuration>("FallingRelax.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.falling_angle        = config["falling_angle"].as<float>();
            cfg.falling_acceleration = config["falling_acceleration"].as<float>();

            // Once the acceleration has stabalized, we are no longer falling
            cfg.recovery_acceleration = config["recovery_acceleration"].as<std::vector<float>>();

            cfg.falling_relax_priority = config["falling_relax_priority"].as<float>();
            log<NUClear::DEBUG>("here");
        });

        on<Last<5, Trigger<Sensors>>, Single>([this](const std::list<std::shared_ptr<const Sensors>>& sensors) {
            double magnitude;
            // Transform to torso {t} from world {w} space
            Eigen::Matrix4d Hwt = sensors.back()->Htw.inverse();
            // Basis Z vector of torso {t} in world {w} space
            Eigen::Vector3d uZTw = Hwt.block(0, 2, 3, 1);

            log<NUClear::DEBUG>("here");
            if (!falling && std::acos(Eigen::Vector3d::UnitZ().dot(uZTw)) > cfg.falling_angle) {

                // We might be falling, check the accelerometer
                magnitude = 0;

                for (const auto& sensor : sensors) {
                    magnitude += sensor->accelerometer.norm();
                }

                magnitude /= sensors.size();

                if (magnitude < cfg.falling_acceleration) {
                    log<NUClear::DEBUG>("Fallen");
                    falling = true;
                    update_priority(cfg.falling_relax_priority);
                }
            }
            else if (falling) {
                // We might be recovered, check the accelerometer
                magnitude = 0;

                for (const auto& sensor : sensors) {
                    magnitude += sensor->accelerometer.norm();
                }

                magnitude /= sensors.size();

                // See if we recover
                if (magnitude > cfg.recovery_acceleration[0] && magnitude < cfg.recovery_acceleration[1]) {
                    falling = false;
                    update_priority(0);
                }
            }

            // Emit graph of fallen state

            emit(graph("FallingRelax angle", fabs(sensors.back()->Htw(2, 2))));
            emit(graph("FallingRelax trigger angle", cfg.falling_angle));
            emit(graph("magnitude", magnitude));
            emit(graph("falling", falling));
        });

        on<Trigger<Falling>>().then(
            [this] { emit(std::make_unique<ExecuteScriptByName>(subsumption_id, "Relax.yaml")); });

        on<Trigger<KillFalling>>().then([this] {
            falling = false;
            update_priority(0);
        });

        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction{
            subsumption_id,
            "Falling Relax",
            {std::pair<float, std::set<LimbID>>(
                0.0f,
                {LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM, LimbID::HEAD})},
            [this](const std::set<LimbID>& /*unused*/) { emit(std::make_unique<Falling>()); },
            [this](const std::set<LimbID>& /*unused*/) { emit(std::make_unique<KillFalling>()); },
            [](const std::set<ServoID>& /*unused*/) {
                // Ignore
            }}));
    }

    void FallingRelax::update_priority(const float& priority) {
        emit(std::make_unique<ActionPriorities>(ActionPriorities{subsumption_id, {priority}}));
    }

}  // namespace module::behaviour::skills
