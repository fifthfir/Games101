#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

int main(){

    float a = 1.0, b = 2.0;
	std::cout<< a << std::endl;
	std::cout<< a << std::endl;
	std::cout<< std::sqrt(a) << std::endl;
	std::cout<< std::acos(-1) << std::endl;  // cos(?) = -1 >> Pi
	std::cout<< std::sin(30.0/180.0 * acos(-1)) << std::endl;  // sin(Pi/6)

	return 0;
}