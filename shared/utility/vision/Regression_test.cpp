//working .cpp file

// .cpp file

// Test code.cpp : This file contains the 'main' function. Program execution begins and ends there.
#include "Regression.hpp"
#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include <iostream>
#include <tuple>

const float error = 0.01;

TEST_CASE("Square root function is correct", "[tag1]") {
	int num1 = 3;
	int num2 = -4;
	REQUIRE(sqr(num1) == 9);
	REQUIRE(sqr(num2) == 16);
}


TEST_CASE("Regression Functions", "[tag2]") {
	auto m2 = linreg_m<float>(coords);
	auto b2 = linreg_b<float>(coords);
	auto rr2 = linreg_rr<float>(coords);

	REQUIRE((0.695 - error) <= m2);
	REQUIRE((0.695 + error) >= m2);
	REQUIRE((1.43 - error) <= b2);
	REQUIRE((1.43 + error) >= b2);
	REQUIRE((0.821 - error) <= rr2);
	REQUIRE((0.821 + error) >= rr2);
}

























/*int main() {
	//std::cout << linreg<float>(coords) << std::endl;
	//std::cout << linreg_b<float>(coords) << std::endl;
	//std::cout << linreg_rr<float>(coords) << std::endl;

	//REQUIRE(linreg_m<float>(coords) == 0.695);

	//std::cout << linreg_m<float>(coords) << std::endl;

	//float m3 = linreg<float>(coords)

	//std::tuple <float, float> lukestuple(2.0, 5.3);
	//std::get<1>(lukestuple);

	////std::cout << linreg<float>(coords) << std::endl;
	//std::cout << std::get<1>(lukestuple) << std::endl;

	//Scalar linreg(coords);

	//for (int i; i < 1; i++)
		//std::cout << std::get<i>(Lukestuple) << std::endl;

	/*linreg<float>(coords);
	std::cout << std::get<0>(Lukestuple);

	std::tuple <float, float> mytuple(std::get<0>(Lukestuple), std::get<1>(Lukestuple));
	linreg<float>(coords);
	std::cout << std::get<0>(Lukestuple) << std::endl;*/

	//std::cout << linreg_m(coords)<<std::endl;
	//std::cout << linreg_b(coords) << std::endl;
	//std::cout << linreg_rr(coords) << std::endl;

//}