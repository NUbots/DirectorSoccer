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

#include "message/behaviour/KickPlan.hpp"
#include "message/behaviour/MotionCommand.hpp"
#include "message/behaviour/Subsumption.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/motion/KickCommand.hpp"
#include "message/motion/WalkCommand.hpp"
#include "message/support/FieldDescription.hpp"
#include "message/vision/Ball.hpp"

#include "utility/behaviour/Action.hpp"
#include "utility/behaviour/MotionCommand.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/localisation/transform.hpp"
#include "utility/motion/splines/SmoothSpline.hpp"
#include "utility/nusight/NUhelpers.hpp"


namespace module::behaviour::planning {

    using extension::Configuration;

    using message::behaviour::KickPlan;
    using message::behaviour::MotionCommand;
    using message::behaviour::WantsToKick;
    using message::input::Sensors;
    using message::localisation::Ball;
    using message::localisation::Field;
    using message::motion::DisableWalkEngineCommand;
    using message::motion::EnableWalkEngineCommand;
    using message::motion::KickFinished;
    using message::motion::StopCommand;
    using message::motion::WalkCommand;
    using message::motion::WalkStopped;
    using message::support::FieldDescription;

    using VisionBalls = message::vision::Balls;

    using utility::behaviour::ActionPriorities;
    using utility::behaviour::RegisterAction;
    using utility::input::LimbID;
    using utility::input::ServoID;
    using utility::localisation::fieldStateToTransform3D;
    using utility::motion::splines::SmoothSpline;
    using utility::nusight::graph;


    SimpleWalkPathPlanner::SimpleWalkPathPlanner(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , latestCommand(utility::behaviour::StandStill())
        , subsumptionId(size_t(this) * size_t(this) - size_t(this))
        , currentTargetPosition(Eigen::Vector2d::Zero())
        , currentTargetHeading(Eigen::Vector2d::Zero())
        , targetHeading(Eigen::Vector2d::Zero(), KickPlan::KickType::SCRIPTED)
        , timeBallLastSeen(NUClear::clock::now()) {

        // do a little configurating
        on<Configuration>("SimpleWalkPathPlanner.yaml").then([this](const Configuration& file) {
            turnSpeed            = file.config["turnSpeed"].as<float>();
            forwardSpeed         = file.config["forwardSpeed"].as<float>();
            sideSpeed            = file.config["sideSpeed"].as<float>();
            a                    = file.config["a"].as<float>();
            b                    = file.config["b"].as<float>();
            search_timeout       = file.config["search_timeout"].as<float>();
            robot_ground_space   = file.config["robot_ground_space"].as<bool>();
            ball_approach_dist   = file.config["ball_approach_dist"].as<float>();
            slowdown_distance    = file.config["slowdown_distance"].as<float>();
            useLocalisation      = file.config["useLocalisation"].as<bool>();
            slow_approach_factor = file.config["slow_approach_factor"].as<float>();

            emit(std::make_unique<WantsToKick>(false));
        });

        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction{
            subsumptionId,
            "Simple Walk Path Planner",
            {
                // Limb sets required by the walk engine:
                std::pair<double, std::set<LimbID>>(0, {LimbID::LEFT_LEG, LimbID::RIGHT_LEG}),
                std::pair<double, std::set<LimbID>>(0, {LimbID::LEFT_ARM, LimbID::RIGHT_ARM}),
            },
            [this](const std::set<LimbID>& givenLimbs) {
                if (givenLimbs.find(LimbID::LEFT_LEG) != givenLimbs.end()) {
                    // Enable the walk engine.
                    emit<Scope::DIRECT>(std::move(std::make_unique<EnableWalkEngineCommand>(subsumptionId)));
                }
            },
            [this](const std::set<LimbID>& takenLimbs) {
                if (takenLimbs.find(LimbID::LEFT_LEG) != takenLimbs.end()) {
                    // Shut down the walk engine, since we don't need it right now.
                    emit<Scope::DIRECT>(std::move(std::make_unique<DisableWalkEngineCommand>(subsumptionId)));
                }
            },
            [this](const std::set<ServoID>&) {
                // nothing
            }}));

        on<Trigger<WalkStopped>>().then([this] {
            emit(std::make_unique<ActionPriorities>(ActionPriorities{subsumptionId, {0, 0}}));
        });


        on<Trigger<VisionBalls>>().then([this](const VisionBalls& balls) {
            if (balls.balls.size() > 0) {
                timeBallLastSeen = NUClear::clock::now();
            }
        });

        on<Every<20, Per<std::chrono::seconds>>,
           With<Ball>,
           With<Field>,
           With<Sensors>,
           With<WantsToKick>,
           With<KickPlan>,
           With<FieldDescription>,
           Sync<SimpleWalkPathPlanner>>()
            .then([this](const Ball& ball,
                         const Field& field,
                         const Sensors& sensors,
                         const WantsToKick& wantsTo,
                         const KickPlan& kickPlan,
                         const FieldDescription& fieldDescription) {
                if (wantsTo.kick) {
                    emit(std::make_unique<StopCommand>(subsumptionId));
                    return;
                }

                if (latestCommand.type == message::behaviour::MotionCommand::Type::STAND_STILL) {


                    emit(std::make_unique<StopCommand>(subsumptionId));


                    return;
                }
                else if (latestCommand.type == message::behaviour::MotionCommand::Type::DIRECT_COMMAND) {
                    std::unique_ptr<WalkCommand> command =
                        std::make_unique<WalkCommand>(subsumptionId, latestCommand.walk_command);
                    emit(std::move(command));
                    emit(std::make_unique<ActionPriorities>(ActionPriorities{subsumptionId, {40, 11}}));
                    return;
                }
                else if (latestCommand.type == message::behaviour::MotionCommand::Type::WALK_TO_STATE) {
                    emit(std::make_unique<WalkCommand>(subsumptionId, Eigen::Vector3d(forwardSpeed, 0.0, turnSpeed)));
                    emit(std::make_unique<ActionPriorities>(ActionPriorities{subsumptionId, {40, 11}}));
                    return;
                }

                // ************** NEW PATH GENERATION BASED ON SPLINES ****************
                Eigen::Affine3d Htw(sensors.Htw);

                auto now = NUClear::clock::now();
                float timeSinceBallLastSeen =
                    std::chrono::duration_cast<std::chrono::nanoseconds>(now - timeBallLastSeen).count()
                    * (1.0f / std::nano::den);


                Eigen::Vector3d walk_command = Eigen::Vector3d::Zero();
                log("Lost ball? ", lost_ball);
                // If we've seen the ball recently, lets walk to it
                if (timeSinceBallLastSeen < search_timeout) {
                    // If we have seen the ball again after losing it, calculate the trajectories again
                    if (lost_ball) {
                        lost_ball  = false;
                        start_time = NUClear::clock::now();
                        Eigen::Vector3d rBWw(ball.position.x(), ball.position.y(), fieldDescription.ball_radius);
                        rBTt = (Htw * rBWw).head<2>();

                        generateWalkPath(rBTt, rBTt);
                    }

                    // if (!lostBall) {
                    log("We saw the ball ", timeSinceBallLastSeen, " seconds ago");
                    log("Ball vector is ", rBTt.transpose());
                    // }
                    double time =
                        std::chrono::duration_cast<std::chrono::nanoseconds>(NUClear::clock::now() - start_time).count()
                        * (1.0f / std::nano::den);

                    // We're resetting the time rn
                    if (time > (Eigen::Vector2d(2.0, 2.0).norm() / forwardSpeed)) {
                        start_time = NUClear::clock::now();
                        time = std::chrono::duration_cast<std::chrono::nanoseconds>(NUClear::clock::now() - start_time)
                                   .count()
                               * (1.0f / std::nano::den);
                    }

                    // log("Time elapsed: ", time);
                    log("Position moving to: ",
                        Eigen::Vector2d(trajectoryX.pos(time), trajectoryY.pos(time)).transpose());
                    // log("Velocity moving to: ",
                    //     Eigen::Vector2d(trajectoryX.vel(time), trajectoryY.vel(time)).transpose());
                    log("End position moving to: ",
                        Eigen::Vector2d(trajectoryX.pos((Eigen::Vector2d(2.0, 2.0).norm() / forwardSpeed)),
                                        trajectoryY.pos((Eigen::Vector2d(2.0, 2.0).norm() / forwardSpeed)))
                            .transpose());

                    Eigen::Vector2d xyPos(samplePath(time));
                    walk_command.x()      = xyPos.x();
                    walk_command.y()      = xyPos.y();
                    double turn_threshold = 5 * M_PI / 180;  // 5 degrees
                    if (std::abs(walk_command.y()) > turn_threshold) {
                        walk_command.z() = walk_command.y() > 0 ? turnSpeed : -turnSpeed;
                    }
                }
                // Otherwise we've lost the ball
                else {
                    lost_ball = true;
                }
                log(walk_command);
                std::unique_ptr<WalkCommand> command = std::make_unique<WalkCommand>(subsumptionId, walk_command);


                emit(std::move(command));
                emit(std::make_unique<ActionPriorities>(ActionPriorities{subsumptionId, {40, 11}}));
            });

        on<Trigger<MotionCommand>, Sync<SimpleWalkPathPlanner>>().then([this](const MotionCommand& cmd) {
            // save the plan
            latestCommand = cmd;
        });
    }

    // Generates a path that is stored in splines
    void SimpleWalkPathPlanner::generateWalkPath(Eigen::Vector2d rBTt, Eigen::Vector2d rGTt) {
        trajectoryX.reset();
        trajectoryY.reset();

        // Calculate walk time
        double walk_time = rGTt.norm() / forwardSpeed;

        // Calculate the estimated position of the ball along the trajectory and adjust time accordingly
        double ratio = rBTt.norm() / rGTt.norm();

        // Generate the walk trajectory for X
        trajectoryX.addPoint(0.0, 0.0, forwardSpeed, 0.0);
        trajectoryX.addPoint(walk_time * ratio, rBTt.x(), forwardSpeed, 0.0);
        trajectoryX.addPoint(walk_time, rGTt.x(), forwardSpeed, 0.0);

        // Generate the walk trajectory for Y
        trajectoryY.addPoint(0.0, 0.0, sideSpeed, 0.0);
        trajectoryY.addPoint(walk_time * ratio, rBTt.y(), sideSpeed, 0.0);
        trajectoryY.addPoint(walk_time, rGTt.y(), sideSpeed, 0.0);

        log("Made trajectories, ", trajectoryY.size(), ":", trajectoryX.size());
    }

    Eigen::Vector2d SimpleWalkPathPlanner::samplePath(double time) {
        log("Trajectories are of size, ", trajectoryY.size(), ":", trajectoryX.size());
        return Eigen::Vector2d(trajectoryX.vel(time), trajectoryY.vel(time));
    }

}  // namespace module::behaviour::planning
