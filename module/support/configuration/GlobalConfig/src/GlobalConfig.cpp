/*
 * This file is part of NUbots Codebase.
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

#include "GlobalConfig.hpp"

#include "extension/Configuration.hpp"

#include "message/support/GlobalConfig.hpp"

namespace module::support::configuration {
    using extension::Configuration;

    GlobalConfig::GlobalConfig(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("GlobalConfig.yaml").then([this](const Configuration& config) {
            auto msg       = std::make_unique<message::support::GlobalConfig>();
            msg->player_id = config["player_id"].as<uint32_t>();
            msg->team_id   = config["team_id"].as<uint32_t>();
            emit(msg);
        });
    }
}  // namespace module::support::configuration
