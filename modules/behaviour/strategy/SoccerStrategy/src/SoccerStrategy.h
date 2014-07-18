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

#ifndef MODULES_BEHAVIOUR_STRATEGY_SOCCERSTRATEGGY_H
#define MODULES_BEHAVIOUR_STRATEGY_SOCCERSTRATEGGY_H

#include <nuclear>
#include <armadillo>

#include "messages/localisation/FieldObject.h"
#include "messages/support/FieldDescription.h"
#include "messages/motion/KickCommand.h"
#include "messages/motion/WalkCommand.h"
#include "messages/input/gameevents/GameEvents.h"
#include "messages/vision/VisionObjects.h"

#include "utility/math/geometry/Polygon.h"

namespace modules {
    namespace behaviour {
        namespace strategy {

		enum class GameStatePrimary : char {
			INITIAL,
			READY,
			SET,
			PLAYING,
			TIMEOUT,
			FINISHED
		};

		enum class GameStateSecondary : char {
			NORMAL,
			PENALTY_SHOOTOUT,
			OVERTIME,
			PENALTY_KICK,
			FREE_KICK,
			GOAL_KICK,
			CORNER_KICK,
			THROW_IN,
			PAUSED
		};

		GameStatePrimary& operator++(GameStatePrimary& gameState) {
			switch (gameState) {
				case GameStatePrimary::INITIAL:
					return(gameState = GameStatePrimary::READY);
				case GameStatePrimary::READY:
					return(gameState = GameStatePrimary::SET);
				case GameStatePrimary::SET:
					return(gameState = GameStatePrimary::PLAYING);
				case GameStatePrimary::PLAYING:
					return(gameState = GameStatePrimary::TIMEOUT);
				case GameStatePrimary::TIMEOUT:
					return(gameState = GameStatePrimary::FINISHED);
				case GameStatePrimary::FINISHED:
				default:
					return(gameState = GameStatePrimary::INITIAL);
			}
		}

		GameStatePrimary operator++(GameStatePrimary& gameState, int) {
			GameStatePrimary tmp(gameState);
			++gameState;
			return(tmp);
		}

		GameStateSecondary& operator++(GameStateSecondary& gameState) {
			switch (gameState) {
				case GameStateSecondary::NORMAL:
					return(gameState = GameStateSecondary::PENALTY_SHOOTOUT);
				case GameStateSecondary::PENALTY_SHOOTOUT:
					return(gameState = GameStateSecondary::OVERTIME);
				case GameStateSecondary::OVERTIME:
					return(gameState = GameStateSecondary::PENALTY_KICK);
				case GameStateSecondary::PENALTY_KICK:
					return(gameState = GameStateSecondary::FREE_KICK);
				case GameStateSecondary::FREE_KICK:
					return(gameState = GameStateSecondary::GOAL_KICK);
				case GameStateSecondary::GOAL_KICK:
					return(gameState = GameStateSecondary::CORNER_KICK);
				case GameStateSecondary::CORNER_KICK:
					return(gameState = GameStateSecondary::THROW_IN);
				case GameStateSecondary::THROW_IN:
					return(gameState = GameStateSecondary::PAUSED);
				case GameStateSecondary::PAUSED:
				default:
					return(gameState = GameStateSecondary::NORMAL);
			}
		}

		GameStateSecondary operator++(GameStateSecondary& gameState, int) {
			GameStateSecondary tmp(gameState);
			++gameState;
			return(tmp);
		}

		typedef struct {
			GameStatePrimary primaryGameState;
			GameStateSecondary secondaryGameState;

			bool selfInZone;
			bool ballInZone;
			bool goalInRange;
			bool kicker;
			bool pickedUp;
			bool penalised;
			bool kickOff;
			bool ballSeen;
			bool ballLost;
			bool teamBallSeen;
			bool ballApproaching;
			bool ballApproachingGoal;
			bool ballHasMoved;
			bool kickPosition;

			arma::vec2 ballGoalIntersection;
			arma::vec2 ballGoalieIntersection;
			arma::vec2 ballSelfIntersection;
			messages::localisation::Ball ball;

			arma::mat22 transform;
			arma::vec2 position;
			arma::vec2 heading;
			arma::vec2 targetPosition;
			arma::vec2 targetHeading;

			bool inPosition;
			bool outOfPosition;
			bool correctHeading;

			std::chrono::system_clock::time_point timeBallLastSeen;
		} State;

		struct Zone {
			arma::vec2 defaultPosition;
			arma::vec2 startPosition;
			utility::math::geometry::Polygon zone;

			Zone() {}
		};

		struct SoccerStrategyConfig {
			static constexpr const char* CONFIGURATION_PATH = "SoccerStrategy.yaml";
		};

		/**
		* High level behaviour for robot soccer.
		*
		* @author Alex Biddulph
		*/
		class SoccerStrategy : public NUClear::Reactor {
		private:
			NUClear::clock::time_point timeSinceLastSeen;

			std::vector<Zone> ZONES;
			int MY_ZONE;
			float MAX_BALL_DISTANCE;
			float KICK_DISTANCE_THRESHOLD;
			float BALL_CERTAINTY_THRESHOLD;
			float BALL_SELF_INTERSECTION_REGION;
			float BALL_MOVEMENT_THRESHOLD;
			float BALL_TIMEOUT_THRESHOLD;
			bool IS_GOALIE;
			float ANGLE_THRESHOLD;
			float POSITION_THRESHOLD_TIGHT;
			float POSITION_THRESHOLD_LOOSE;

			messages::support::FieldDescription FIELD_DESCRIPTION;

			State currentState, previousState;

			bool gameStateButtonStatus;
			bool gameStateButtonStatusPrev;
			bool penalisedButtonStatus;
			bool penalisedButtonStatusPrev;

			bool feetOffGround;
			bool isKicking;
			bool isDiving;
			bool isGettingUp;
			bool isWalking;

			arma::vec2 findOptimalPosition(const utility::math::geometry::Polygon& zone, const arma::vec2& point);
			void findSelf();
			void findBall();
			void goToPoint(const arma::vec2& position, const arma::vec2& heading);
			void sideStepToPoint(const arma::vec2& position);
			void kickBall(const arma::vec2& direction);
			void diveForBall(const arma::vec2& target);
			void approachBall(const arma::vec2& haading);

			void updateGameState(const std::shared_ptr<const messages::input::gameevents::GameState>& gameController);

			//NEW
			bool penalised(const std::shared_ptr<const messages::input::gameevents::GameState>& gameState);
			void playSoccer(const arma::vec2& localisationBall, const messages::vision::Ball& visionBall, const messages::localisation::Self& self, const std::shared_ptr<const messages::input::gameevents::GameState>& gameState);
			void playGoalie(const arma::vec2& localisationBall, const messages::localisation::Self& self);
			void searchForBall(const messages::localisation::Ball& localisationBall, const messages::localisation::Self& self, const std::shared_ptr<const messages::input::gameevents::GameState>& gameState);

			void walkToStartPosition(const messages::localisation::Self& self);
			void stopWalking();
			void findSelfAndBall();
			void spin();


			arma::vec2 enemyGoal;

		public:
			explicit SoccerStrategy(std::unique_ptr<NUClear::Environment> environment);
		};

		}  // strategy
	}  // behaviours
}  // modules

#endif  // MODULES_BEHAVIOUR_STRATEGY_SOCCERSTRATEGGY_H

