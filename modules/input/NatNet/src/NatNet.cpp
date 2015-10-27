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
 * Copyright 2015 NUbots <nubots@nubots.net>
 */

#include "NatNet.h"
#include "Parse.h"

#include "messages/support/Configuration.h"

namespace modules {
namespace input {

    using messages::support::Configuration;

    NatNet::NatNet(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("NatNet.yaml").then([this] (const Configuration& config) {

            // We are updating to a new multicast address
            if (multicastAddress != config["multicast_address"].as<std::string>()
                || dataPort != config["data_port"].as<uint32_t>()
                || commandPort != config["command_port"].as<uint32_t>()) {

                if (commandHandle) {
                    commandHandle.unbind();
                }
                if (dataHandle) {
                    dataHandle.unbind();
                }

                // Set our new variables
                multicastAddress = config["multicast_address"].as<std::string>();
                dataPort = config["data_port"].as<uint16_t>();
                commandPort = config["command_port"].as<uint16_t>();

                log<NUClear::INFO>("Connecting to NatNet network", multicastAddress);

                // Create a listening UDP port for commands
                std::tie(commandHandle, std::ignore, commandFd) = on<UDP>().then("NatNet Command", [this] (UDP::Packet packet) {
                    process(packet.data);
                });

                // Create a listening UDP port for data
                std::tie(dataHandle, std::ignore, std::ignore) = on<UDP::Multicast>(multicastAddress, dataPort).then("NatNet Data", [this] (UDP::Packet packet) {

                    // Test if we are "connected" to this remote
                    // And if we are we can use the data
                    if (remote == packet.remote.address && version != 0) {
                        process(packet.data);
                    }
                    // We have started connecting but haven't received a return ping
                    else if (remote == packet.remote.address && version == 0) {
                        // TODO maybe set a timeout here to try again
                    }
                    // We haven't connected to anything yet
                    else if (remote == 0) {
                        // This is now our remote
                        remote = packet.remote.address;

                        // Send a ping command
                        sendCommand(Packet::Type::PING);
                    }
                    else if (remote != packet.remote.address) {
                        log<NUClear::WARN>("There is more than one NatNet server running on this network");
                    }
                });
            }
        });
    }

    void NatNet::sendCommand(Packet::Type type, std::vector<char> data) {
        if (remote > 0) {
            // Make a vector to hold our packet
            std::vector<char> packet(sizeof(Packet) - 1);

            // Fill in the header
            Packet* header = reinterpret_cast<Packet*>(packet.data());
            header->type = type;
            header->length = data.size();

            // Fill in the data
            packet.insert(packet.end(), data.begin(), data.end());

            // Work out our remotes address
            sockaddr_in address;
            memset(&address, 0, sizeof(sockaddr_in));
            address.sin_family = AF_INET;
            address.sin_port = htons(commandPort);
            address.sin_addr.s_addr = htonl(remote);

            // Send to our remote server
            ::sendto(commandFd, packet.data(), packet.size(), 0, reinterpret_cast<sockaddr*>(&address), sizeof(sockaddr));
        }
        else {
            log<NUClear::WARN>("NatNet is not yet connected to a remote server");
        }
    }

    void NatNet::processFrame(const Packet& packet) {

        // Our output motion capture object
        auto mocap = std::make_unique<MotionCapture>();

        // Our pointer as we move through the data
        const char* ptr = &packet.data;

        // Read frame number
        mocap->frameNumber = ReadData<uint32_t>::read(ptr, version);

        // Read the markersets
        mocap->markerSets = ReadData<std::vector<MotionCapture::MarkerSet>>::read(ptr, version);

        // Read the free floating markers
        mocap->markers = ReadData<std::vector<MotionCapture::Marker>>::read(ptr, version);

        // Read the Rigid Bodies
        mocap->rigidBodies = ReadData<std::vector<MotionCapture::RigidBody>>::read(ptr, version);

        // Read the skeletons
        if(version >= 0x02010000) {
            mocap->skeletons = ReadData<std::vector<MotionCapture::Skeleton>>::read(ptr, version);
        }

        // Read the labeled markers
        if(version >= 0x02030000) {
            mocap->labeledMarkers = ReadData<std::vector<MotionCapture::LabeledMarker>>::read(ptr, version);
        }

        // Read the force plates
        if(version >= 0x02090000) {
            mocap->forcePlates = ReadData<std::vector<MotionCapture::ForcePlate>>::read(ptr, version);
        }

        // Read our metadata
        mocap->latency     = ReadData<float>::read(ptr, version);
        mocap->timecode    = ReadData<uint32_t>::read(ptr, version);
        mocap->timecodeSub = ReadData<uint32_t>::read(ptr, version);

        // In version 2.9 timestamp went from a float to a double
        if (version >= 0x02090000) {
            mocap->timestamp = ReadData<double>::read(ptr, version);
        }
        else {
            mocap->timestamp = ReadData<float>::read(ptr, version);
        }

        short params = ReadData<short>::read(ptr, version);
        mocap->recording            = (params & 0x01) == 0x01;
        mocap->trackedModelsChanged = (params & 0x01) == 0x02;

        // TODO there is an eod thing here

        // Apply the model information we have to the objects
        for (auto& markerSet : mocap->markerSets) {

            auto model = markerSetModels.find(markerSet.name);

            // We have a model
            if(model != markerSetModels.end()) {
            }
            // We need to update our models
            else {
                // Inform that we are updating our models
                log<NUClear::INFO>("NatNet models are out of date, updating before resuming data");

                // Request model definitions
                sendCommand(Packet::Type::REQUEST_MODEL_DEFINITIONS);

                // Stop processing
                return;
            }
        }

        for (auto& rigidBody : mocap->rigidBodies) {

            auto model = rigidBodyModels.find(rigidBody.id);

            // We have a model
            if(model != rigidBodyModels.end()) {

                auto parent = std::find_if(mocap->rigidBodies.begin(), mocap->rigidBodies.end(), [model] (const MotionCapture::RigidBody& rb) {
                    return rb.id == model->second.id;
                });

                rigidBody.name = model->second.name;
                rigidBody.parent = parent == mocap->rigidBodies.end() ? nullptr : &*parent;
                rigidBody.offset = model->second.offset;
            }
            // We need to update our models
            else {
                // Inform that we are updating our models
                log<NUClear::INFO>("NatNet models are out of date, updating before resuming data");

                // Request model definitions
                sendCommand(Packet::Type::REQUEST_MODEL_DEFINITIONS);

                // Stop processing
                return;
            }
        }

        // Now we reverse link all our rigid bodies
        for (auto& rigidBody : mocap->rigidBodies) {
            if(rigidBody.parent) {
                rigidBody.parent->children.push_back(&rigidBody);
            }
        }

        for (auto& skeleton : mocap->skeletons) {

            auto model = skeletonModels.find(skeleton.id);

            // We have a model
            if(model != skeletonModels.end()) {

                for (auto& bone : skeleton.bones) {
                    auto boneModel = model->second.boneModels.find(bone.id);
                    // We have a model for this bone
                    if (boneModel != model->second.boneModels.end()) {

                        auto parent = std::find_if(skeleton.bones.begin(), skeleton.bones.end(), [boneModel] (const MotionCapture::RigidBody& rb) {
                            return rb.id == boneModel->second.id;
                        });

                        bone.name = boneModel->second.name;
                        bone.parent = parent == skeleton.bones.end() ? nullptr : &*parent;
                        bone.offset = boneModel->second.offset;
                    }
                    // We need to update our models
                    else {
                        // Inform that we are updating our models
                        log<NUClear::INFO>("NatNet models are out of date, updating before resuming data");

                        // Request model definitions
                        sendCommand(Packet::Type::REQUEST_MODEL_DEFINITIONS);

                        // Stop processing
                        return;
                    }
                }
            }
            // We need to update our models
            else {
                // Inform that we are updating our models
                log<NUClear::INFO>("NatNet models are out of date, updating before resuming data");

                // Request model definitions
                sendCommand(Packet::Type::REQUEST_MODEL_DEFINITIONS);

                // Stop processing
                return;
            }

            // Now we reverse link all our bones
            for (auto& rigidBody : skeleton.bones) {
                if(rigidBody.parent) {
                    rigidBody.parent->children.push_back(&rigidBody);
                }
            }
        }

        // Emit our frame
        emit(std::move(mocap));
    }

    void NatNet::processModel(const Packet& packet) {

        // Our pointer as we move through the data
        const char* ptr = &packet.data;

        uint32_t nModels = ReadData<uint32_t>::read(ptr, version);

        for (uint32_t i = 0; i < nModels; ++i) {
            // Read the type
            uint32_t type = ReadData<uint32_t>::read(ptr, version);

            // Parse the correct type
            switch (type) {
                // Marker Set
                case 0: {
                    MarkerSetModel m = ReadData<MarkerSetModel>::read(ptr, version);
                    markerSetModels[m.name] = m;
                } break;

                // Rigid Body
                case 1: {
                    RigidBodyModel m = ReadData<RigidBodyModel>::read(ptr, version);
                    rigidBodyModels[m.id] = m;
                } break;

                // Skeleton
                case 2: {
                    SkeletonModel m = ReadData<SkeletonModel>::read(ptr, version);
                    skeletonModels[m.id] = m;
                } break;

                // Bad packet
                default: {
                    log<NUClear::WARN>("NatNet received an unexpected model type", type);
                } break;
            }
        }
    }

    void NatNet::processPing(const Packet& packet) {

        // Extract the information from the packet
        std::string name(&packet.data);
        const char* appVersion = &packet.data + 256;
        const char* natNetVersion = appVersion + 4;

        // Update our version number
        version = (natNetVersion[0] << 24)
                | (natNetVersion[0] << 16)
                | (natNetVersion[0] << 8)
                | (natNetVersion[0] << 0);

        // Make our app version a string
        std::string strAppVersion = std::to_string(int(appVersion[0]))
                            + "." + std::to_string(int(appVersion[1]))
                            + "." + std::to_string(int(appVersion[2]))
                            + "." + std::to_string(int(appVersion[3]));

        // Make our natNetVersion a string
        std::string strNatVersion = std::to_string(int(natNetVersion[0]))
                            + "." + std::to_string(int(natNetVersion[1]))
                            + "." + std::to_string(int(natNetVersion[2]))
                            + "." + std::to_string(int(natNetVersion[3]));

        log<NUClear::INFO>("Connected to", name, strAppVersion);
        log<NUClear::INFO>("NatNet version", strNatVersion);

        // Request model definitions
        sendCommand(Packet::Type::REQUEST_MODEL_DEFINITIONS);
    }

    void NatNet::processResponse(const Packet& packet) {
        // if(gCommandResponseSize==4)
        //     memcpy(&gCommandResponse, &PacketIn.Data.lData[0], gCommandResponseSize);
        // else
        // {
        //     memcpy(&gCommandResponseString[0], &PacketIn.Data.cData[0], gCommandResponseSize);
        //     printf("Response : %s", gCommandResponseString);
        //     gCommandResponse = 0;   // ok
        // }
    }

    void NatNet::processString(const Packet& packet) {
        std::string str(&packet.data, packet.length);

        // TODO do something with this string?
    }


    void NatNet::process(const std::vector<char>& input) {

        // Get our packet
        const Packet& packet = *reinterpret_cast<const Packet*>(input.data());

        // Work out it's type
        switch(packet.type) {
            case Packet::Type::PING_RESPONSE:
                processPing(packet);
            break;

            case Packet::Type::RESPONSE:
                processResponse(packet);
            break;

            case Packet::Type::MODEL_DEF:
                processModel(packet);
            break;

            case Packet::Type::FRAME_OF_DATA:
                processFrame(packet);
            break;

            case Packet::Type::UNRECOGNIZED_REQUEST:
                log<NUClear::ERROR>("An unrecognized request was made to the NatNet server");
            break;

            case Packet::Type::MESSAGE_STRING:
                processString(packet);

            default:
                log<NUClear::ERROR>("The NatNet server sent an unexpected packet type");
            break;
        }
    }
}
}
