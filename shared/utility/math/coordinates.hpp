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

#ifndef UTILITY_MATH_COORDINATES_HPP
#define UTILITY_MATH_COORDINATES_HPP

#include <Eigen/Core>
#include <armadillo>
#include <cmath>

namespace utility {
namespace math {

    /**
     * Functions to convert between coordinate representations.
     *
     * (r,phi,theta) represent radial distance, bearing (counter-clockwise from x-axis in xy-plane) and elevation
     * (measured from the xy plane) (in radians)
     * @author Alex Biddulph
     */
    namespace coordinates {
        inline arma::vec3 sphericalToCartesian(const arma::vec3& sphericalCoordinates) {
            double distance  = sphericalCoordinates[0];
            double cos_theta = cos(sphericalCoordinates[1]);
            double sin_theta = sin(sphericalCoordinates[1]);
            double cos_phi   = cos(sphericalCoordinates[2]);
            double sin_phi   = sin(sphericalCoordinates[2]);
            arma::vec3 result;

            result[0] = distance * cos_theta * cos_phi;
            result[1] = distance * sin_theta * cos_phi;
            result[2] = distance * sin_phi;

            return result;
        }

        template <typename T, typename U = typename T::Scalar>
        inline Eigen::Matrix<U, 3, 1> sphericalToCartesian(const Eigen::MatrixBase<T>& sphericalCoordinates) {
            double distance  = sphericalCoordinates.x();
            double cos_theta = std::cos(sphericalCoordinates.y());
            double sin_theta = std::sin(sphericalCoordinates.y());
            double cos_phi   = std::cos(sphericalCoordinates.z());
            double sin_phi   = std::sin(sphericalCoordinates.z());
            Eigen::Matrix<U, 3, 1> result;

            result.x() = distance * cos_theta * cos_phi;
            result.y() = distance * sin_theta * cos_phi;
            result.z() = distance * sin_phi;

            return result;
        }

        inline arma::vec3 cartesianToSpherical(const arma::vec3& cartesianCoordinates) {
            double x = cartesianCoordinates[0];
            double y = cartesianCoordinates[1];
            double z = cartesianCoordinates[2];
            arma::vec3 result;

            result[0] = sqrt(x * x + y * y + z * z);  // r
            result[1] = atan2(y, x);                  // theta
            if (result[0] == 0) {
                result[2] = 0;
            }
            else {
                result[2] = asin(z / (result[0]));  // phi
            }

            return result;
        }

        template <typename T, typename U = typename T::Scalar>
        inline Eigen::Matrix<U, 3, 1> cartesianToSpherical(const Eigen::MatrixBase<T>& cartesianCoordinates) {
            U x = cartesianCoordinates.x();
            U y = cartesianCoordinates.y();
            U z = cartesianCoordinates.z();
            Eigen::Matrix<U, 3, 1> result;

            result.x() = std::sqrt(x * x + y * y + z * z);  // r
            result.y() = std::atan2(y, x);                  // theta
            if (result.x() == static_cast<U>(0)) {
                result.z() = static_cast<U>(0);
            }
            else {
                result.z() = std::asin(z / (result.x()));  // phi
            }

            return result;
        }

        inline arma::vec4 sphericalToCartesian4(const arma::vec3& sphericalCoordinates) {
            arma::vec3 p = sphericalToCartesian(sphericalCoordinates);
            return arma::vec4({p[0], p[1], p[2], 1});
        }

        inline arma::vec4 cartesianToSpherical4(const arma::vec3& cartesianCoordinates) {
            arma::vec3 p = cartesianToSpherical(cartesianCoordinates);
            return arma::vec4({p[0], p[1], p[2], 1});
        }


        inline arma::vec2 cartesianToRadial(const arma::vec2& cartesianCoordinates) {
            double x = cartesianCoordinates[0];
            double y = cartesianCoordinates[1];

            return {sqrt(x * x + y * y), atan2(y, x)};
        }

        inline arma::vec2 spherical2Radial(const arma::vec3& sphericalCoordinates) {
            double dist          = sphericalCoordinates(0);
            double declination   = sphericalCoordinates(2);
            double flat_distance = dist * std::cos(declination);

            arma::vec2 result;
            result[0] = flat_distance;
            result[1] = sphericalCoordinates(1);

            return result;
        }
    }  // namespace coordinates
}  // namespace math
}  // namespace utility


#endif  // UTILITY_MATH_COORDINATES_HPP
