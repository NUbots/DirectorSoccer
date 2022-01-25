#ifndef UTILITY_VISION_REGRESSION_HPP
#define UTILITY_VISION_REGRESSION_HPP

// https://stackoverflow.com/a/19040841/11487433

#include <Eigen/Core>
#include <math.h>
#include <stdlib.h>
#include <utility>

namespace utility::vision {

    template <typename Scalar>
    inline static Scalar sqr(Scalar x) {
        return x * x;
    }

    // Linear regression
    // Returns a pair with success boolean and results
    // Results are (m,b,r) which are the slope of the line, the y-intercept of the line, and the correlation coefficient
    template <typename Scalar>
    std::pair<bool, std::tuple<Scalar, Scalar, Scalar>> linreg(std::vector<Eigen::Matrix<Scalar, 2, 1>> coords) {
        int n    = coords.size();
        Scalar m = 0.0;
        Scalar b = 0.0;
        Scalar r = 0.0;

        Scalar sumx  = 0.0; /* sum of x     */
        Scalar sumx2 = 0.0; /* sum of x**2  */
        Scalar sumxy = 0.0; /* sum of x * y */
        Scalar sumy  = 0.0; /* sum of y     */
        Scalar sumy2 = 0.0; /* sum of y**2  */

        for (int i = 0; i < n; i++) {
            sumx += coords[i][0];
            sumx2 += sqr(coords[i][0]);
            sumxy += coords[i][0] * coords[i][1];
            sumy += coords[i][1];
            sumy2 += sqr(coords[i][1]);
        }

        Scalar denom = (n * sumx2 - sqr(sumx));
        // Check for divide by zero
        if (denom == 0) {
            return std::make_pair(false, std::make_tuple(0, 0, 0));
        }

        m = (n * sumxy - sumx * sumy) / denom;
        b = (sumy * sumx2 - sumx * sumxy) / denom;
        r = (sumxy - sumx * sumy / n) / sqrt((sumx2 - sqr(sumx) / n) * (sumy2 - sqr(sumy) / n));

        return std::make_pair(true, std::make_tuple(m, b, r));
    }
}  // namespace utility::vision

#endif  // UTILITY_VISION_REGRESSION_HPP
