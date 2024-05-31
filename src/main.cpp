#include <vector>
#include <eigen3/Eigen/Core>
#include <iostream>

int main() {

    std::vector<double> a = {-1, -2, -3, -4, -5, -6, -7, -8, -9};

    Eigen::VectorXd b = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(a.data(), a.size()).array() + 2;

    double c = (b.array() < 0.).select(0., b).matrix().norm();

    std::cout << c;


}