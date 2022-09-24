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

#include "SimpleWalkPathPlanner.hpp"

#include <Eigen/Geometry>
#include <cmath>

#include "extension/Configuration.hpp"

#include "message/behaviour/MotionCommand.hpp"
#include "message/input/Sensors.hpp"
#include "message/motion/WalkCommand.hpp"
#include "message/vision/Ball.hpp"

#include "utility/behaviour/Action.hpp"
#include "utility/behaviour/MotionCommand.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/localisation/transform.hpp"
#include "utility/math/comparison.hpp"
#include "utility/math/coordinates.hpp"
#include "utility/nusight/NUhelpers.hpp"

namespace module::behaviour::planning {

    using extension::Configuration;

    using message::behaviour::MotionCommand;
    using message::input::Sensors;
    using message::motion::DisableWalkEngineCommand;
    using message::motion::EnableWalkEngineCommand;
    using message::motion::StopCommand;
    using message::motion::WalkCommand;
    using message::motion::WalkStopped;
    using VisionBalls = message::vision::Balls;

    using utility::behaviour::ActionPriorities;
    using utility::behaviour::RegisterAction;
    using utility::input::LimbID;
    using utility::input::ServoID;
    using utility::math::coordinates::reciprocalSphericalToCartesian;
    using utility::math::coordinates::sphericalToCartesian;


    SimpleWalkPathPlanner::SimpleWalkPathPlanner(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), subsumption_id(size_t(this) * size_t(this) - size_t(this)) {

        // do a little configurating
        on<Configuration>("SimpleWalkPathPlanner.yaml").then([this](const Configuration& config) {
            log_level                      = config["log_level"].as<NUClear::LogLevel>();
            cfg.max_turn_speed             = config["max_turn_speed"].as<float>();
            cfg.min_turn_speed             = config["min_turn_speed"].as<float>();
            cfg.forward_speed              = config["forward_speed"].as<float>();
            cfg.rotate_speed_x             = config["rotate_speed_x"].as<float>();
            cfg.rotate_speed_y             = config["rotate_speed_y"].as<float>();
            cfg.rotate_speed               = config["rotate_speed"].as<float>();
            cfg.walk_to_ready_speed_x      = config["walk_to_ready_speed_x"].as<float>();
            cfg.walk_to_ready_speed_y      = config["walk_to_ready_speed_y"].as<float>();
            cfg.walk_to_ready_rotation     = config["walk_to_ready_rotation"].as<float>();
            cfg.walk_path_planner_priority = config["walk_path_planner_priority"].as<float>();
        });

        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(
            RegisterAction{subsumption_id,
                           "Simple Walk Path Planner",
                           {
                               // Limb sets required by the walk engine:
                               std::pair<double, std::set<LimbID>>(
                                   0,
                                   {LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM}),
                           },
                           [this](const std::set<LimbID>& givenLimbs) {
                               if (givenLimbs.find(LimbID::LEFT_LEG) != givenLimbs.end()) {
                                   // Enable the walk engine.
                                   emit<Scope::DIRECT>(std::make_unique<EnableWalkEngineCommand>(subsumption_id));
                               }
                           },
                           [this](const std::set<LimbID>& takenLimbs) {
                               if (takenLimbs.find(LimbID::LEFT_LEG) != takenLimbs.end()) {
                                   // Shut down the walk engine, since we don't need it right now.
                                   emit<Scope::DIRECT>(std::make_unique<DisableWalkEngineCommand>(subsumption_id));
                               }
                           },
                           [](const std::set<ServoID>& /*unused*/) {
                               // nothing
                           }}));

        on<Trigger<WalkStopped>>().then([this] { update_priority(0); });

        // Used to maintain the position of the last seen ball
        // TODO(LocalisationTeam): Should be implemented outside of the path planner in something like
        // BallLocalisation
        on<Trigger<VisionBalls>, With<Sensors>>().then([this](const VisionBalls& balls, const Sensors& sensors) {
            if (balls.balls.size() > 0) {
                // Get the latest vision ball measurement in camera space
                Eigen::Vector3f rBCc =
                    reciprocalSphericalToCartesian(balls.balls[0].measurements[0].srBCc.cast<float>());
                // Transform the vision ball measurement into torso space
                Eigen::Affine3f Htc(sensors.Htw.cast<float>() * balls.Hcw.inverse().cast<float>());
                rBTt = Htc * rBCc;
            }
        });

        // Freq should be equal to the main loop in soccer strategy. TODO (Bryce Tuppurainen): Potentially
        // change this value to a define in an included header if this will be used again for added design cohesion if
        // this will be something we change in future
        on<Every<30, Per<std::chrono::seconds>>, Sync<SimpleWalkPathPlanner>>().then([this]() {
            switch (static_cast<int>(latest_command.type.value)) {
                case message::behaviour::MotionCommand::Type::STAND_STILL:
                    emit(std::make_unique<StopCommand>(subsumption_id));
                    return;

                case message::behaviour::MotionCommand::Type::DIRECT_COMMAND: walk_directly(); return;

                case message::behaviour::MotionCommand::Type::BALL_APPROACH: vision_walk_path(); return;

                // TODO(MotionTeam): Walk to a given position and heading on the field, avoiding obstacles
                case message::behaviour::MotionCommand::Type::WALK_TO_STATE: return;

                case message::behaviour::MotionCommand::Type::ROTATE_ON_SPOT: rotate_on_spot(); return;

                case message::behaviour::MotionCommand::Type::WALK_TO_READY: walk_to_ready(); return;
                default:  // This line should be UNREACHABLE
                    log<NUClear::ERROR>(
                        fmt::format("Invalid walk path planning command {}.", latest_command.type.value));
                    emit(std::make_unique<StopCommand>(subsumption_id));
                    return;
            }
        });

        // Save the plan cmd into latest_command
        on<Trigger<MotionCommand>, Sync<SimpleWalkPathPlanner>>().then(
            [this](const MotionCommand& cmd) { latest_command = cmd; });
    }

    //-------- Add extra behaviours for path planning here and in the switch case of this file

    void SimpleWalkPathPlanner::walk_directly() {
        emit(std::move(std::make_unique<WalkCommand>(subsumption_id, latest_command.walk_command)));
        update_priority(cfg.walk_path_planner_priority);
    }

    void SimpleWalkPathPlanner::vision_walk_path() {
        // Obtain the unit vector to ball in torso space and scale by cfg.forward_speed
        Eigen::Vector3f walk_command = cfg.forward_speed * (rBTt.normalized());

        // Overide the z component of walk_command with angular velocity, which is just the angular displacement to
        // ball, saturated with value cfg.max_turn_speed float
        walk_command.z() = std::atan2(walk_command.y(), walk_command.x());
        walk_command.z() = utility::math::clamp(cfg.min_turn_speed, walk_command.z(), cfg.max_turn_speed);

        std::unique_ptr<WalkCommand> command =
            std::make_unique<WalkCommand>(subsumption_id, walk_command.cast<double>());
        emit(std::move(command));
        update_priority(cfg.walk_path_planner_priority);
    }

    void SimpleWalkPathPlanner::rotate_on_spot() {
        std::unique_ptr<WalkCommand> command =
            std::make_unique<WalkCommand>(subsumption_id,
                                          Eigen::Vector3d(cfg.rotate_speed_x, cfg.rotate_speed_y, cfg.rotate_speed));
        emit(std::move(command));
        update_priority(cfg.walk_path_planner_priority);
    }

    void SimpleWalkPathPlanner::walk_to_ready() {
        std::unique_ptr<WalkCommand> command = std::make_unique<WalkCommand>(
            subsumption_id,
            Eigen::Vector3d(cfg.walk_to_ready_speed_x, cfg.walk_to_ready_speed_y, cfg.walk_to_ready_rotation));
        emit(std::move(command));
        update_priority(cfg.walk_path_planner_priority);
    }

    void SimpleWalkPathPlanner::update_priority(const float& priority) {
        emit(std::make_unique<ActionPriorities>(ActionPriorities{subsumption_id, {priority}}));
    }

}  // namespace module::behaviour::planning
