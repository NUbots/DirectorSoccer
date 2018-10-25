#include "HardwareIO.h"

#include "extension/Configuration.h"

#include "NUgus.h"
#include "dynamixel/v2/Dynamixel.hpp"

#include "message/platform/nugus/StatusReturn.h"

namespace module {
namespace platform {
    namespace nugus {

        using extension::Configuration;

        HardwareIO::HardwareIO(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)), opencr(), byte_wait(0), packet_wait(0), packet_queue() {

            on<Configuration>("HardwareIO.yaml").then([this](const Configuration& config) {
                // Make sure OpenCR is operating at the correct baud rate (based on config params)
                if (opencr.connected()) {
                    opencr.close();
                }

                opencr.open(config["opencr"]["device"], config["opencr"]["baud"]);
                byte_wait   = config["opencr"]["byte_wait"];
                packet_wait = config["opencr"]["packet_wait"];

                // Initialise packet_queue map
                packet_queue[NUgus::ID::OPENCR] = std::vector<PacketTypes>();
                for (int i = 0; i < 20; ++i) {
                    packet_queue[i] = std::vector<PacketTypes>();
                }
            });

            on<Startup>().then("HardwareIO Startup", [this] {
                // Set the OpenCR to not return a status packet when written to (to allow consecutive writes)
                opencr.write(dynamixel::v2::Write(NUgus::ID::OpenCR, OpenCR::Address::STATUS_RETURN_LEVEL, uint8_t(1)));

                // Set the OpenCRs delay time to 0 (it may not have been configured before)
                opencr.write(dynamixel::v2::Write(NUgus::ID::OpenCR, OpenCR::Address::RETURN_DELAY_TIME, uint8_t(0)));

                // Find OpenCR firmware and model versions
                packet_queue[NUgus::ID::OPENCR].push_back(PacketTypes::MODEL_INFORMATION);
                opencr.write(dynamixel::v2::Read(NUgus::ID::OPENCR, OpenCR::Address::MODEL_NUMBER_L, 3));

                // Enable power to the servos
                opencr.write(dynamixel::v2::Write(NUgus::ID::OpenCR, OpenCR::Address::DYNAMIXEL_POWER, uint8_t(1)));

                // Wait about 300ms for the dynamixels to start up
                std::this_thread::sleep_for(std::chrono::milliseconds(300));

                // Set the dynamixels to not return a status packet when written to (to allow consecutive writes)
                std::array<dynamixel::v2::SyncWriteData<uint8_t>, 20> data;
                for (int i = 0; i < 20; ++i) {
                    data[i] = dynamixel::v2::SyncWriteData<uint8_t>(i, 1);
                }

                opencr.write(
                    dynamixel::v2::SyncWrite<uint8_t, 20>(NUgus::R_SHOULDER_PITCH::Address::STATUS_RETURN_LEVEL, data));

                // Now that the dynamixels should have started up, set their delay time to 0 (it may not have been
                // configured before)
                for (int i = 0; i < 20; ++i) {
                    data[i] = dynamixel::v2::SyncWriteData<uint8_t>(i, 0);
                }

                opencr.write(
                    dynamixel::v2::SyncWrite<uint8_t, 20>(NUgus::R_SHOULDER_PITCH::Address::RETURN_DELAY_TIME, data));

                // Set up indirect addressing for read addresses
                std::array<dynamixel::v2::SyncWriteData<std::array<uint16_t, 17>>, 20> read_data;

                for (int i = 0; i < 20; ++i) {
                    read_data[i] = dynamixel::v2::SyncWriteData<std::array<uint16_t, 17>>(
                        i,
                        {NUgus::L_SHOULDER_PITCH::Address::TORQUE_ENABLE,
                         NUgus::L_SHOULDER_PITCH::Address::HARDWARE_ERROR_STATUS,
                         NUgus::L_SHOULDER_PITCH::Address::PRESENT_PWM_L,
                         NUgus::L_SHOULDER_PITCH::Address::PRESENT_PWM_H,
                         NUgus::L_SHOULDER_PITCH::Address::PRESENT_CURRENT_L,
                         NUgus::L_SHOULDER_PITCH::Address::PRESENT_CURRENT_H,
                         NUgus::L_SHOULDER_PITCH::Address::PRESENT_VELOCITY_L,
                         NUgus::L_SHOULDER_PITCH::Address::PRESENT_VELOCITY_2,
                         NUgus::L_SHOULDER_PITCH::Address::PRESENT_VELOCITY_3,
                         NUgus::L_SHOULDER_PITCH::Address::PRESENT_VELOCITY_H,
                         NUgus::L_SHOULDER_PITCH::Address::PRESENT_POSITION_L,
                         NUgus::L_SHOULDER_PITCH::Address::PRESENT_POSITION_2,
                         NUgus::L_SHOULDER_PITCH::Address::PRESENT_POSITION_3,
                         NUgus::L_SHOULDER_PITCH::Address::PRESENT_POSITION_H,
                         NUgus::L_SHOULDER_PITCH::Address::PRESENT_VOLTAGE_L,
                         NUgus::L_SHOULDER_PITCH::Address::PRESENT_VOLTAGE_H,
                         NUgus::L_SHOULDER_PITCH::Address::PRESENT_TEMPERATURE});
                }

                opencr.write(dynamixel::v2::SyncWrite<std::array<uint16_t, 17>, 20>(
                    NUgus::L_SHOULDER_PITCH::Address::INDIRECT_ADDRESS_1_L, read_data));

                // Set up indirect addressing for write addresses
                std::array<dynamixel::v2::SyncWriteData<std::array<uint16_t, 11>>, 20> write_data1;
                std::array<dynamixel::v2::SyncWriteData<std::array<uint16_t, 24>>, 20> write_data2;

                for (int i = 0; i < 20; ++i) {
                    write_data1[i] = dynamixel::v2::SyncWriteData<std::array<uint16_t, 11>>(
                        i,
                        {NUgus::L_SHOULDER_PITCH::Address::TORQUE_ENABLE,
                         NUgus::L_SHOULDER_PITCH::Address::VELOCITY_I_GAIN_L,
                         NUgus::L_SHOULDER_PITCH::Address::VELOCITY_I_GAIN_H,
                         NUgus::L_SHOULDER_PITCH::Address::VELOCITY_P_GAIN_L,
                         NUgus::L_SHOULDER_PITCH::Address::VELOCITY_P_GAIN_H,
                         NUgus::L_SHOULDER_PITCH::Address::VELOCITY_D_GAIN_L,
                         NUgus::L_SHOULDER_PITCH::Address::VELOCITY_D_GAIN_H,
                         NUgus::L_SHOULDER_PITCH::Address::POSITION_I_GAIN_L,
                         NUgus::L_SHOULDER_PITCH::Address::POSITION_I_GAIN_H,
                         NUgus::L_SHOULDER_PITCH::Address::POSITION_P_GAIN_L,
                         NUgus::L_SHOULDER_PITCH::Address::POSITION_P_GAIN_H});

                    write_data2[i] = dynamixel::v2::SyncWriteData<std::array<uint16_t, 24>>(
                        i,
                        {NUgus::L_SHOULDER_PITCH::Address::FEEDFORWARD_1ST_GAIN_L,
                         NUgus::L_SHOULDER_PITCH::Address::FEEDFORWARD_1ST_GAIN_H,
                         NUgus::L_SHOULDER_PITCH::Address::FEEDFORWARD_2ND_GAIN_L,
                         NUgus::L_SHOULDER_PITCH::Address::FEEDFORWARD_2ND_GAIN_H,
                         NUgus::L_SHOULDER_PITCH::Address::GOAL_PWM_L,
                         NUgus::L_SHOULDER_PITCH::Address::GOAL_PWM_H,
                         NUgus::L_SHOULDER_PITCH::Address::GOAL_CURRENT_L,
                         NUgus::L_SHOULDER_PITCH::Address::GOAL_CURRENT_H,
                         NUgus::L_SHOULDER_PITCH::Address::GOAL_VELOCITY_L,
                         NUgus::L_SHOULDER_PITCH::Address::GOAL_VELOCITY_2,
                         NUgus::L_SHOULDER_PITCH::Address::GOAL_VELOCITY_3,
                         NUgus::L_SHOULDER_PITCH::Address::GOAL_VELOCITY_H,
                         NUgus::L_SHOULDER_PITCH::Address::PROFILE_ACCELERATION_L,
                         NUgus::L_SHOULDER_PITCH::Address::PROFILE_ACCELERATION_2,
                         NUgus::L_SHOULDER_PITCH::Address::PROFILE_ACCELERATION_3,
                         NUgus::L_SHOULDER_PITCH::Address::PROFILE_ACCELERATION_H,
                         NUgus::L_SHOULDER_PITCH::Address::PROFILE_VELOCITY_L,
                         NUgus::L_SHOULDER_PITCH::Address::PROFILE_VELOCITY_2,
                         NUgus::L_SHOULDER_PITCH::Address::PROFILE_VELOCITY_3,
                         NUgus::L_SHOULDER_PITCH::Address::PROFILE_VELOCITY_H,
                         NUgus::L_SHOULDER_PITCH::Address::GOAL_POSITION_L,
                         NUgus::L_SHOULDER_PITCH::Address::GOAL_POSITION_2,
                         NUgus::L_SHOULDER_PITCH::Address::GOAL_POSITION_3,
                         NUgus::L_SHOULDER_PITCH::Address::GOAL_POSITION_H});
                }

                opencr.write(dynamixel::v2::SyncWrite<std::array<uint16_t, 11>, 20>(
                    NUgus::L_SHOULDER_PITCH::Address::INDIRECT_ADDRESS_18_L, write_data1));
                opencr.write(dynamixel::v2::SyncWrite<std::array<uint16_t, 24>, 20>(
                    NUgus::L_SHOULDER_PITCH::Address::INDIRECT_ADDRESS_29_L, write_data2));
            });

            on<Shutdown>().then("HardwareIO Startup", [this] {
                // Close our connection to the OpenCR
                if (opencr.connected()) {
                    opencr.close();
                }
            });

            // This trigger gets the sensor data from the CM730
            on<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>, Single, Priority::HIGH>().then(
                "Hardware Loop", [this] {
                    // Our final sensor output
                    auto sensors = std::make_unique<DarwinSensors>();

                    // Write out servo data
                    // SYNC_WRITE (write the same memory addresses on all devices)

                    // Get updated servo data
                    // SYNC_READ (read the same memory addresses on all devices)
                    for (int i = 0; i < 20; ++i) {
                        packet_queue[i].push_back(PacketTypes::SERVO_DATA);
                    }
                    opencr.write(dynamixel::v2::SyncReadCommand<20>(NUgus::L_SHOULDER_PITCH::Address::INDIRECT_DATA_1_L,
                                                                    sizeof(DynamixelServoReadData),
                                                                    NUgus::servo_ids()));

                    // Get OpenCR data
                    // READ (only reading from a single device here)
                    packet_queue[NUgus::ID::OPENCR].push_back(PacketTypes::OPENCR_DATA);
                    opencr.write(
                        dynamixel::v2::ReadCommand(NUgus::ID::OPENCR, NUgus::OPENCR::Address::LED, sizeof(OpenCRData)));
                });

            // This trigger writes the servo positions to the hardware
            on<Trigger<std::vector<ServoTarget>>, With<DarwinSensors>>().then(
                [this](const std::vector<ServoTarget>& commands, const DarwinSensors& sensors) {
                    // Loop through each of our commands and update servo state information accordingly
                    for (const auto& command : commands) {
                    }
                });

            on<Trigger<ServoTarget>>().then([this](const ServoTarget command) {
                auto commandList = std::make_unique<std::vector<ServoTarget>>();
                commandList->push_back(command);

                // Emit it so it's captured by the reaction above
                emit<Scope::DIRECT>(std::move(commandList));
            });

            // If we get a HeadLED command then write it
            on<Trigger<DarwinSensors::HeadLED>>().then([this](const DarwinSensors::HeadLED& led) {
                // Update our internal state
            });

            // If we get a EyeLED command then write it
            on<Trigger<DarwinSensors::EyeLED>>().then([this](const DarwinSensors::EyeLED& led) {
                // Update our internal state
            });

            // If we get a EyeLED command then write it
            on<Trigger<DarwinSensors::LEDPanel>>().then([this](const DarwinSensors::LEDPanel& led) {
                // Update our internal state
            });

            on<Trigger<message::platform::nugus::StatusReturn>>().then(
                [this](const message::platform::nugus::StatusReturn& packet) {
                    // Figure out what the contents of the message are
                    if (packet_queue[packet.id].size() > 0) {
                        // Pop the front of the packet queue
                        auto info = packet_queue[packet.id].front();
                        packet_queue[packet.id].erase(packet_queue[packet.id].begin());

                        switch (info.front()) {
                            // Handles OpenCR model and version information
                            case PacketTypes::MODEL_INFORMATION:
                                uint16_t model  = (packet.data[1] << 8) | packet.data[0];
                                uint8_t version = packet.data[2];
                                log<NUClear::INFO>(fmt::format("OpenCR Model...........: {:#06X}", model));
                                log<NUClear::INFO>(fmt::format("OpenCR Firmware Version: {:#04X}", version));
                                break;

                            // Handles OpenCR sensor data
                            case PacketTypes::OPENCR_DATA:
                                // Work out a battery charged percentage
                                sensors->battery =
                                    std::max(0.0f, (sensors->voltage - flatVoltage) / (chargedVoltage - flatVoltage));
                                break;

                            // Handles servo data
                            case PacketTypes::SERVO_DATA:
                                // Parse our data
                                NUgus::DynamixelServoReadData data =
                                    *(reinterpret_cast<NUgus::DynamixelServoReadData*>(packet.data));

                                servoState[packet.id].torqueEnabled   = data.torqueEnable;
                                servoState[packet.id].errorFlags      = data.hardwareErrorStatus;
                                servoState[packet.id].presentPwm      = data.presentPwm;
                                servoState[packet.id].presentCurrent  = data.presentCurrent;
                                servoState[packet.id].presentVelocity = data.presentVelocity;
                                servoState[packet.id].presentPosition = data.presentPosition;
                                servoState[packet.id].voltage         = data.presentVoltage;
                                servoState[packet.id].temperature     = data.presentTemperature;

                                break;

                            // What is this??
                            default: log<NUClear::WARN>("Unknown packet data received") break;
                        }
                    }

                    else {
                        log<NUClear::WARN>(fmt::format("Unexpected packet data received for ID {}.", packet.id));
                    }
                });

            // When we receive data back from the OpenCR it will arrive here
            // Run a state machine to handle reception of packet header and data
            on<IO>(opencr.native_handle(), IO::READ).then([this] {
                enum class Phases : uint8_t { IDLE, HEADER_SYNC, PREAMBLE, DATA, FINISH, TIMEOUT };

                static constexpr uint8_t packet_header              = {0xFF, 0xFF, 0xFD, 0x00};
                static Phases current_phase                         = Phases::IDLE;
                static uint8_t sync_point                           = 0;
                static NUClear::clock::time_point packet_start_time = NUClear::clock::now();
                static std::chrono::microseconds timeout            = std::chrono::microseconds(packet_wait);
                static std::vector<uint8_t> response;
                static uint16_t packet_length = 0;

                std::array<uint8_t, 128> buf;
                uint8_t num_bytes;

                num_bytes = opencr.read(buf, 128);

                // Quick lambda to emit a completed StatusReturn message (to reduce code duplication)
                auto emit_msg = [this, &]() -> void {
                    std::unique_ptr<message::platform::nugus::StatusReturn> msg;
                    msg->magic       = 0x00FDFFFF;
                    msg->id          = response[4];
                    msg->length      = packet_length;
                    msg->instruction = response[7];
                    msg->error       = static_cast<message::platform::nugus::StatusReturn::CommandError>(response[8]);
                    std::copy(
                        response.begin() + 9, response.begin() + packet_length - 4, std::back_inserter(msg->data));
                    msg->checksum  = (response[response.size() - 1] << 8) | response[response.size() - 2];
                    msg->timestamp = NUClear::clock::now();

                    // Check CRC
                    if (dynamixel::v2::calculateChecksum(response.data()) == msg->checksum) {
                        emit(msg);
                    }
                    else {
                        log<NUClear::WARN>("Invalid CRC detected.")
                    }
                };

                // Quick lambda to reset state machine state (to reduce code duplication)
                auto reset_state = [this, &]() -> void {
                    sync_point    = 0;
                    packet_length = 0;
                    current_phase = Phases::IDLE;
                    response.clear();
                };

                for (uint8_t i = 0; i < num_bytes; ++i) {
                    switch (current_phase) {
                        // Idle phase
                        // We haven't seen any data, so we are waiting for the
                        // first byte of the header
                        case Phases::IDLE:
                            // If we match the first byte of the header then
                            // transition to the HEADER_SYNC phase
                            if (packet_header[sync_point] == buf[i]) {
                                response.push_back(packet_header[sync_point]);
                                sync_point++;
                                current_phase     = Phases::HEADER_SYNC;
                                packet_start_time = NUClear::clock::now();
                                timeout           = std::chrono::microseconds(packet_wait);
                            }
                            break;

                        // Header Sync phase
                        // We have matched the first byte of the header
                        // now match the next three bytes
                        case Phases::HEADER_SYNC:
                            if (NUClear::clock::now() < (packet_start_time + timeout)) {
                                if (packet_header[sync_point] == buf[i]) {
                                    response.push_back(packet_header[sync_point]);
                                    sync_point++;
                                }

                                // Header has been matched
                                // Now read in the rest of the packet
                                if (sync_point == 4) {
                                    sync_point        = 0;
                                    current_phase     = Phases::PREAMBLE;
                                    packet_start_time = NUClear::clock::now();
                                    timeout           = std::chrono::microseconds(byte_wait * 5 + 2000 + packet_wait);
                                }
                            }
                            else {
                                current_phase = Phases::TIMEOUT;
                            }
                            break;

                        // Preamble phase
                        // We have the full header, now we are looking for the next 5 bytes
                        // Packet ID, Length, Instruction ID, and Error
                        case Phases::PREAMBLE:
                            if (NUClear::clock::now() < (packet_start_time + timeout)) {
                                response.push_back(buf[i]);

                                // We just read in the expected length of the packet
                                if (response.size() == 7) {
                                    packet_length = (response[6] << 8) | response[5];
                                }

                                // We now have the header and the packet preamble
                                // Time to get the packet parameters and CRC
                                if (response.size() == 9) {
                                    current_phase = Phases::DATA;
                                    timeout = std::chrono::microseconds(byte_wait * packet_length + 2000 + packet_wait);
                                }
                            }
                            else {
                                current_phase = Phases::TIMEOUT;
                            }
                            break;

                        // Data phase
                        // Header and preamble have been received
                        // Now we read in the message parameters and CRC
                        // We should be looking for (packet_length - 2) bytes
                        case Phases::DATA:
                            if (NUClear::clock::now() < (packet_start_time + timeout)) {
                                response.push_back(buf[i]);

                                // We now have all of our data
                                if (response.size() == 7 + packet_length) {
                                    current_phase = Phases::FINISH;
                                }
                            }
                            else {
                                current_phase = Phases::TIMEOUT;
                            }
                            break;

                        // Finish phase
                        // We have now received a complete message
                        // However, it looks like we have more data to process
                        // Package up the current message, and reset our buf counter and phase
                        case Phases::FINISH:
                            emit_msg();

                            // Set up for next message
                            i--;  // Decrement counter to account for the next increment
                            reset_state();
                            break;

                        // Timeout phase
                        // It took too long to read in the full packet .... Giving up
                        case Phases::TIMEOUT:
                            log<NUClear::WARN>("Packet timeout occurred.");
                            reset_state();
                            break;

                        // Yea, dont know what happened here
                        default: reset_state(); break;
                    }
                }

                // Our input buffer ended at the exact end of the packet
                if (response.size() == 7 + packet_length) {
                    emit_msg();
                    reset_state();
                }
            });
        }
    }  // namespace nugus
}  // namespace platform
}  // namespace module
