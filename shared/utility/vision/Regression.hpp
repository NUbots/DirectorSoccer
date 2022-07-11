

// working header file

// header file

#ifndef UTILITY_VISION_REGRESSION_HPP
#define UTILITY_VISION_REGRESSION_HPP

// https://stackoverflow.com/a/19040841/11487433

#include <Eigen/Core>  //was #include <Eigen>
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <tuple>
#include <utility>
#include <vector>


namespace utility::vision {

    std::vector<std::vector<float>> coords = {{2.9, 4.0},
                                              {6.7, 7.4},
                                              {4.9, 5.0},
                                              {7.9, 7.2},
                                              {9.8, 7.9},
                                              {6.9, 6.1},
                                              {6.1, 6.0},
                                              {6.2, 5.8},
                                              {6.0, 5.2},
                                              {5.1, 4.2},
                                              {4.7, 4.0},
                                              {4.4, 4.4},
                                              {5.8, 5.2}};

    template <typename Scalar>
    inline static Scalar sqr(Scalar x) {
        return x * x;
    }

    // Linear regression
    // Returns a pair with success boolean and results
    // Results are (m,b,r) which are the slope of the line, the y-intercept of the line, and the correlation coefficient
    template <typename Scalar>
    Scalar linreg_m(std::vector<std::vector<Scalar>>) {  // linreg(std::vector<Eigen::Matrix<Scalar, 12, 2 >> coords) {
        int n    = coords.size();                        // linreg(std::vector<Eigen::Matrix<Scalar, 12, 2 >> coords) {
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
            return 0;
        }

        m         = (n * sumxy - sumx * sumy) / denom;
        b         = (sumy * sumx2 - sumx * sumxy) / denom;
        r         = (sumxy - sumx * sumy / n) / sqrt((sumx2 - sqr(sumx) / n) * (sumy2 - sqr(sumy) / n));
        Scalar rr = r * r;


        return m;
    }


    template <typename Scalar>
    Scalar linreg_b(std::vector<std::vector<Scalar>>) {
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
            return 0;
        }

        m         = (n * sumxy - sumx * sumy) / denom;
        b         = (sumy * sumx2 - sumx * sumxy) / denom;
        r         = (sumxy - sumx * sumy / n) / sqrt((sumx2 - sqr(sumx) / n) * (sumy2 - sqr(sumy) / n));
        Scalar rr = r * r;

        return b;
    }


    template <typename Scalar>
    Scalar linreg_rr(std::vector<std::vector<Scalar>>) {
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
            return 0;
        }

        m         = (n * sumxy - sumx * sumy) / denom;
        b         = (sumy * sumx2 - sumx * sumxy) / denom;
        r         = (sumxy - sumx * sumy / n) / sqrt((sumx2 - sqr(sumx) / n) * (sumy2 - sqr(sumy) / n));
        Scalar rr = r * r;

        return rr;
    }

}  // namespace utility::vision

#endif  // UTILITY_VISION_REGRESSION_HPP