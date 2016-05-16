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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_LOCALISATION_MMKFBALLLOCALISATION_H
#define MODULES_LOCALISATION_MMKFBALLLOCALISATION_H

#include <nuclear>
#include <armadillo>
#include "BallModel.h"

namespace module {
namespace localisation {

    class MMKFBallLocalisation : public NUClear::Reactor {
    private:
        ReactionHandle emit_data_handle;
        
        //filter configuration
        size_t numFilters;
        std::vector<utility::math::filter::UKF<mmball::BallModel>> ballFilters;
        
        //model configuration
        double ballDragCoefficient;
        double sphericalMinDist;
        std::vector<arma::mat> processNoiseModels;
        
        //dynamic data
        double lastUpdateTime;
        arma::vec odometryUpdate;
        arma::mat odometryNoiseMatrix;

    public:
        /// @brief Called by the powerplant to build and setup the KFBallLocalisation reactor.
        explicit MMKFBallLocalisation(std::unique_ptr<NUClear::Environment> environment);
    };

}
}
#endif

