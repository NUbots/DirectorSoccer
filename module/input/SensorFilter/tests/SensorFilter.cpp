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

#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <catch.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <yaml-cpp/yaml.h>

#include "MotionModel.hpp"

#include "message/input/Sensors.hpp"
#include "message/motion/BodySide.hpp"
#include "message/motion/KinematicsModel.hpp"

#include "utility/input/ServoID.hpp"
#include "utility/math/filter/UKF.hpp"
#include "utility/math/quaternion.hpp"
#include "utility/motion/ForwardKinematics.hpp"
#include "utility/support/yaml_expression.hpp"

using module::input::MotionModel;
using utility::math::filter::UKF;
using utility::support::Expression;

void print_error(const int& line, const std::string& msg, const Eigen::Matrix<double, 10, 10>& covariance) {
    const double covariance_sigma_weight = 0.1 * 0.1 * 10;
    const Eigen::Matrix<double, 10, 10> state(covariance_sigma_weight
                                              * covariance.unaryExpr([](const double& c) { return std::abs(c); }));
    INFO(state.diagonal());
    INFO("Line " << line << ": " << msg << "\n");
}

TEST_CASE("Test MotionModel Orientation", "[module][input][SensorFilter][MotionModel][orientation]") {
    // Create our motion model
    UKF<double, MotionModel> filter;

    // Read in test data and ground truths
    std::vector<Eigen::Vector3d> gyro_readings;
    std::vector<Eigen::Vector3d> acc_readings;
    std::vector<Eigen::Quaterniond> quaternions;

    char comma;
    std::ifstream ifs("tests/gyroscope_measurements_no_noise.txt");
    while (ifs.good()) {
        Eigen::Vector3d gyro;
        ifs >> gyro.x() >> comma >> gyro.y() >> comma >> gyro.z();
        if (ifs.good()) {
            gyro_readings.emplace_back(gyro);
        }
    }
    ifs.close();
    ifs.open("tests/accelerometer_measurements_no_noise.txt");
    while (ifs.good()) {
        Eigen::Vector3d acc;
        ifs >> acc.x() >> comma >> acc.y() >> comma >> acc.z();
        if (ifs.good()) {
            acc_readings.emplace_back(acc);
        }
    }
    ifs.close();
    ifs.open("tests/orientation.txt");
    while (ifs.good()) {
        Eigen::Matrix3d rot_mat;
        // clang-format off
        double _1_1, _1_2, _1_3,
               _2_1, _2_2, _2_3,
               _3_1, _3_2, _3_3;
        ifs >> _1_1 >> comma >> _1_2 >> comma >> _1_3 >> comma
            >> _2_1 >> comma >> _2_2 >> comma >> _2_3 >> comma
            >> _3_1 >> comma >> _3_2 >> comma >> _3_3 >> comma;

        rot_mat << _1_1 , _1_2 , _1_3,
                   _2_1 , _2_2 , _2_3,
                   _3_1 , _3_2 , _3_3;
        //clang-format on
        if (ifs.good()) {
            quaternions.emplace_back(Eigen::Quaternion<double>(rot_mat));
        }
    }
    ifs.close();

    // Configure the motion model
    YAML::Node config = YAML::LoadFile("config/SensorFilter.yaml");
    // Set our process noise in our filter
    MotionModel<double>::StateVec process_noise;
    const auto& process         = config["motion_filter"]["noise"]["process"];
    process_noise.Rwt           = Eigen::Vector4d(process["rotation"].as<Expression>());
    process_noise.omegaTTt      = process["rotational_velocity"].as<Expression>();
    process_noise.omegaTTt_bias = process["gyroscope_bias"].as<Expression>();
    filter.model.process_noise  = process_noise;

    // Set our initial mean and covariance
    MotionModel<double>::StateVec mean;
    MotionModel<double>::StateVec covariance;
    const auto& initial      = config["motion_filter"]["initial"];
    mean.Rwt                 = Eigen::Vector4d(initial["mean"]["rotation"].as<Expression>());
    mean.omegaTTt            = initial["mean"]["rotational_velocity"].as<Expression>();
    mean.omegaTTt_bias       = initial["mean"]["gyroscope_bias"].as<Expression>();
    covariance.Rwt           = Eigen::Vector4d(initial["covariance"]["rotation"].as<Expression>());
    covariance.omegaTTt      = initial["covariance"]["rotational_velocity"].as<Expression>();
    covariance.omegaTTt_bias = initial["covariance"]["gyroscope_bias"].as<Expression>();
    filter.set_state(mean.getStateVec(), covariance.asDiagonal());
    // switch (filter.set_state(mean.getStateVec(), covariance.asDiagonal())) {
    //     case Eigen::Success: break;
    //     case Eigen::NumericalIssue:
    //         print_error(__LINE__,
    //                     "Cholesky decomposition failed. The provided data did not satisfy the "
    //                     "prerequisites.",
    //                     filter.getCovariance());
    //         break;
    //     case Eigen::NoConvergence:
    //         print_error(__LINE__,
    //                     "Cholesky decomposition failed. Iterative procedure did not converge.",
    //                     filter.getCovariance());
    //         break;
    //     case Eigen::InvalidInput:
    //         print_error(__LINE__,
    //                     "Cholesky decomposition failed. The inputs are invalid, or the algorithm has been "
    //                     "improperly called. When assertions are enabled, such errors trigger an assert.",
    //                     filter.getCovariance());
    //         break;
    //     default:
    //         print_error(__LINE__, "Cholesky decomposition failed. Some other reason.", filter.getCovariance());
    //         break;
    // }

    // Noise to be applied to gyroscope measurements
    Eigen::Matrix3d gyroscope_noise =
        Eigen::Vector3d(config["motion_filter"]["noise"]["measurement"]["gyroscope"].as<Expression>()).asDiagonal();

    // Noise to be applied to accelerometer measurements
    Eigen::Matrix3d accelerometer_noise =
        Eigen::Vector3d(config["motion_filter"]["noise"]["measurement"]["accelerometer"].as<Expression>()).asDiagonal();
    Eigen::Matrix3d accelerometer_magnitude_noise =
        Eigen::Vector3d(config["motion_filter"]["noise"]["measurement"]["accelerometer_magnitude"].as<Expression>())
            .asDiagonal();

    // Elapsed time between each sensor read
    constexpr double deltaT = 1.0 / 90.0;

    // // Set up for adding gaussian noise to the measurements
    // // Gyroscope datasheet says the MEMS device has a 0.03 dps/sqrt(Hz) noise density with a bandwidth of 50Hz
    // // Accelerometer datasheet says the MEMS device has a 220 micro-g/sqrt(Hz) noise density with a bandwidth 400Hz
    // std::random_device rd{};
    // std::mt19937 gen{rd()};
    // std::normal_distribution<> gyro_sensor_noise{0.0, 0.03 * std::sqrt(50.0) * M_PI / 180.0};
    // std::normal_distribution<> acc_sensor_noise{0.0, 22e-6 * std::sqrt(400) * module::input::G};

    // Vector of quaternion errors from each timestep
    std::vector<Eigen::Quaterniond> errors;
    std::vector<double> angular_errors;

    // Step through test data and get orientation predictions
    for (int i = 0; i < int(quaternions.size()); ++i) {
        // Perform the measurement update
        filter.measure(gyro_readings[i], gyroscope_noise, module::input::MeasurementType::GYROSCOPE());

        // Calculate accelerometer noise factor
        Eigen::Matrix3d acc_noise = accelerometer_noise
                                    + ((acc_readings[i].norm() - std::abs(module::input::G))
                                       * (acc_readings[i].norm() - std::abs(module::input::G)))
                                          * accelerometer_magnitude_noise;

        // Accelerometer measurement update
        filter.measure(acc_readings[i], acc_noise, module::input::MeasurementType::ACCELEROMETER());

        // Time update
        INFO("Running time update for step " << i);
        INFO("Gyroscope Measurement....: " << gyro_readings[i].transpose());
        INFO("Accelerometer Measurement: " << acc_readings[i].transpose() << " (" << acc_readings[i].norm() << ")");
        INFO("Expected Orientation.....: " << quaternions[i].coeffs().transpose());
        bool failed = false;
        filter.time(deltaT);
        // switch (filter.time(deltaT)) {
        //     case Eigen::Success: break;
        //     case Eigen::NumericalIssue:
        //         print_error(__LINE__,
        //                     "Cholesky decomposition failed. The provided data did not satisfy the "
        //                     "prerequisites.",
        //                     filter.getCovariance());
        //         failed = true;
        //         break;
        //     case Eigen::NoConvergence:
        //         print_error(__LINE__,
        //                     "Cholesky decomposition failed. Iterative procedure did not converge.",
        //                     filter.getCovariance());
        //         failed = true;
        //         break;
        //     case Eigen::InvalidInput:
        //         print_error(__LINE__,
        //                     "Cholesky decomposition failed. The inputs are invalid, or the algorithm has been "
        //                     "improperly called. When assertions are enabled, such errors trigger an assert.",
        //                     filter.getCovariance());
        //         failed = true;
        //         break;
        //     default:
        //         print_error(__LINE__, "Cholesky decomposition failed. Some other reason.", filter.getCovariance());
        //         failed = true;
        //         break;
        // }

        // if (!failed) {
            // Calculate difference between expected and predicted orientations
            const Eigen::Quaterniond& Rwt = MotionModel<double>::StateVec(filter.get()).Rwt;
            const double dot              = quaternions[i].dot(Rwt);

            INFO("Predicted Orientation....: " << Rwt.coeffs().transpose());

            angular_errors.emplace_back(std::acos(2.0 * dot * dot - 1.0));
            errors.emplace_back(utility::math::quaternion::difference(Rwt, quaternions[i]));
        // }
        // else {
        //     // UKF state unrecoverable. Print current average error and bail
        //     const Eigen::Quaterniond& Rwt = MotionModel<double>::StateVec(filter.get()).Rwt;
        //     const double dot              = quaternions[i].dot(Rwt);

        //     const double current_angular_error = std::acos(2.0 * dot * dot - 1.0);

        //     angular_errors.emplace_back(current_angular_error);
        //     errors.emplace_back(utility::math::quaternion::difference(Rwt, quaternions[i]));

        //     const Eigen::Quaterniond mean_error =
        //         utility::math::quaternion::mean(errors.begin(), errors.end()).normalized();
        //     const double mean_angular_error =
        //         std::accumulate(angular_errors.begin(), angular_errors.end(), 0.0) / double(angular_errors.size());

        //     INFO("Predicted Orientation....: " << Rwt.coeffs().transpose());
        //     INFO("Current Angular Error....: " << current_angular_error);
        //     INFO("Mean Error........: " << mean_error.coeffs().transpose());
        //     INFO("Mean Angular Error: " << mean_angular_error);

        //     const double covariance_sigma_weight = 0.1 * 0.1 * 10;
        //     const Eigen::Matrix<double, 10, 10> state(
        //         covariance_sigma_weight
        //         * filter.getCovariance().unaryExpr([](const double& c) { return std::abs(c); }));
        //     INFO(state.diagonal());

        //     FAIL("UKF State unrecoverable. Aborting");
        // }
    }

    const Eigen::Quaterniond mean_error = utility::math::quaternion::mean(errors.begin(), errors.end()).normalized();
    const double mean_angular_error =
        std::accumulate(angular_errors.begin(), angular_errors.end(), 0.0) / double(angular_errors.size());
    INFO("Mean Error........: " << mean_error.coeffs().transpose());
    INFO("Mean Angular Error: " << mean_angular_error);
    REQUIRE(mean_error.w() == Approx(1.0));
    REQUIRE(mean_error.x() == Approx(0.0));
    REQUIRE(mean_error.y() == Approx(0.0));
    REQUIRE(mean_error.z() == Approx(0.0));
    REQUIRE(mean_angular_error == Approx(0.0));
}
