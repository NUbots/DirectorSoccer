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
 * Copyright 2022 NUbots <nubots@nubots.net>
 */

#include <memory>

#include "Director.hpp"

#include "extension/Behaviour.hpp"

namespace module::extension {

    using component::DirectorTask;

    bool Director::direct_priority(const std::shared_ptr<DirectorTask>& incumbent,
                                   const std::shared_ptr<DirectorTask>& challenger) {
        return incumbent->optional == challenger->optional ? incumbent->priority < challenger->priority
                                                           : incumbent->optional;
    }

}  // namespace module::extension
