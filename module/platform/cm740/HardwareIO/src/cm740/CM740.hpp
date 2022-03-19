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

#ifndef CM740_CM740_HPP
#define CM740_CM740_HPP

#include <cstdint>
#include <utility>
#include <vector>

#include "CM740Data.hpp"
#include "FSR.hpp"
#include "RawSensors.hpp"
#include "Servo.hpp"

#include "extension/Configuration.hpp"

namespace CM740 {

    /**
     * Contains all of the dynamixel IDs in the system
     */
    namespace ID {
        enum ID {
            CM740            = 200,
            R_SHOULDER_PITCH = 1,
            L_SHOULDER_PITCH = 2,
            R_SHOULDER_ROLL  = 3,
            L_SHOULDER_ROLL  = 4,
            R_ELBOW          = 5,
            L_ELBOW          = 6,
            R_HIP_YAW        = 7,
            L_HIP_YAW        = 8,
            R_HIP_ROLL       = 9,
            L_HIP_ROLL       = 10,
            R_HIP_PITCH      = 11,
            L_HIP_PITCH      = 12,
            R_KNEE           = 13,
            L_KNEE           = 14,
            R_ANKLE_PITCH    = 15,
            L_ANKLE_PITCH    = 16,
            R_ANKLE_ROLL     = 17,
            L_ANKLE_ROLL     = 18,
            HEAD_YAW         = 19,
            HEAD_PITCH       = 20,
            R_FSR            = 111,
            L_FSR            = 112,
            BROADCAST        = 254
        };
    }  // namespace ID

    /**
     * @brief The main class that others will use to interact with the CM740 and attached devices.
     *
     * @details
     *  This is the main access point for all users of this CM740 driver. Note that it is build for a little endian
     *  system, and if it is used on a big endian system, the code will need to be reviewed. This is because it is
     *  reading the 2 byte values as they are on the CM740 (which is little endian).
     *
     * @author Trent Houliston
     */
    class CM740 {
    private:
        /// Our UART class that we will communicate through
        UART uart;
        /// Which servos we should be building for
        std::array<bool, 20> enabledServoIds{};

        /// Our Prebuilt bulk read command
        std::vector<uint8_t> bulkReadCommand;

        /**
         * @brief Builds a bulk read packet to read all of the sensors.
         */
        void buildBulkReadPacket();

    public:
        void setConfig(const extension::Configuration& config);

        /// The CM740
        CM740Data cm740;
        /// The Right Shoulder Pitch Servo
        Servo rShoulderPitch;
        /// The Left Shoulder Pitch Servo
        Servo lShoulderPitch;
        /// The Right Shoulder Roll Servo
        Servo rShoulderRoll;
        /// The Left Shoulder Roll Servo
        Servo lShoulderRoll;
        /// The Right Elbow Servo
        Servo rElbow;
        /// The Left Elbow Servo
        Servo lElbow;
        /// The Right Hip Yaw Servo
        Servo rHipYaw;
        /// The Left Hip Yaw Servo
        Servo lHipYaw;
        /// The Right Hip Roll Servo
        Servo rHipRoll;
        /// The Left Hip Roll Servo
        Servo lHipRoll;
        /// The Right Hip Pitch Servo
        Servo rHipPitch;
        /// The Left Hip Pitch Servo
        Servo lHipPitch;
        /// The Right Knee Servo
        Servo rKnee;
        /// The Left Knee Servo
        Servo lKnee;
        /// The Right Ankle Pitch Servo
        Servo rAnklePitch;
        /// The Left Ankle Pitch Servo
        Servo lAnklePitch;
        /// The Right Ankle Roll Servo
        Servo rAnkleRoll;
        /// The Left Ankle Roll Servo
        Servo lAnkleRoll;
        /// The Head Pan Servo
        Servo headPan;
        /// The Head Tilt Servo
        Servo headTilt;
        /// The Right Foot FSR
        FSR rFSR;
        /// The Left Foot FSR
        FSR lFSR;

        /**
         * @brief Gets the device with the given sensor id
         *
         * @param id the ID of the device to get (e.g. 200 for the CM740 itself)
         *
         * @return the CM740Interface object that controls this id
         */
        CM740Interface& operator[](int id);

        /**
         * @brief Constructs a new CM740 instance and sets up communication with the CM740.
         *
         * @param name the file handle for the device the CM740 is connected to (e.g. /dev/ttyUSB0)
         */
        explicit CM740(const char* name);

        /**
         * @brief Pings all of the attached devices, and returns a map containing their ID and if they were contactable.
         *
         * @return a map containing IDs and if they were contactable (returned no error code)
         */
        std::vector<std::pair<uint8_t, bool>> selfTest();

        /**
         * @brief This reads all of the sensors in a predefined pattern of what is considered "Interesting"
         *
         * @return A BulkReadResuts object containing all of the sensor data as it was read from the device (no
         * trasforms)
         */
        BulkReadResults bulkRead();

        /**
         * @brief This sends a raw command to the UART that the dynamixels are on without expecting a response
         */
        void sendRawCommand(std::vector<uint8_t>& packet);
    };
}  // namespace CM740

#endif
