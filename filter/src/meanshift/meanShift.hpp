#ifndef MEANSHIFT_MEANSHIFT_HPP
#define MEANSHIFT_MEANSHIFT_HPP

#include <vector>

#include "Point.hpp"
#include "Cluster.hpp"


#define MAX_ITERATIONS 100


std::vector<Cluster> meanShift(const std::vector<Point> &points, float bandwidth);


#endif //MEANSHIFT_MEANSHIFT_HPP
