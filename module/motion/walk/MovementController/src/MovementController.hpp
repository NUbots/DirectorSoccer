#ifndef MODULE_MOTION_WALK_MOVEMENTCONTROLLER_H
#define MODULE_MOTION_WALK_MOVEMENTCONTROLLER_H

#include <nuclear>

#include "FootController.hpp"
#include "TorsoController.hpp"

namespace module {
    namespace motion {
        namespace walk {
            class MovementController : public NUClear::Reactor {

            public:
                /// @brief Called by the powerplant to build and setup the MovementController reactor.
                explicit MovementController(std::unique_ptr<NUClear::Environment> environment);

            private:
                // Controller for the swing foot
                FootController foot_controller;
                // Controller for the support foot to move the torso
                TorsoController torso_controller;
                struct {
                    double time_horizon;
                    double support_gain;
                    double swing_gain;
                    double swing_lean_gain;
                } config;
            };
        }  // namespace walk
    }      // namespace motion
}  // namespace module

#endif  // MODULE_MOTION_WALK_MOVEMENTCONTROLLER_H