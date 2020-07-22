#include <vector>
#include <iostream>
#include <cmath>

#include "Point.hpp"
#include "Cluster.hpp"
#include "ClustersBuilder.hpp"
#include "meanShift.hpp"


std::vector<Cluster> meanShift(const std::vector<Point> &points, float bandwidth) {
    ClustersBuilder builder = ClustersBuilder(points, 0.4);
    long iterations = 0;
    unsigned long dimensions = points[0].dimensions();
    float radius = bandwidth * 3;
    float doubledSquaredBandwidth = 2 * bandwidth * bandwidth;
    while (!builder.allPointsHaveStoppedShifting() && iterations < MAX_ITERATIONS) {

#pragma omp parallel for default(none) \
shared(points, dimensions, builder, bandwidth, radius, doubledSquaredBandwidth) \
schedule(dynamic)

        for (long i = 0; i < points.size(); ++i) {
            if (builder.hasStoppedShifting(i))
                continue;

            Point newPosition(dimensions);
            Point pointToShift = builder.getShiftedPoint(i);
            float totalWeight = 0.0;
            for (auto &point : points) {
                float distance = pointToShift.euclideanDistance(point);
                if (distance <= radius) {
                    float gaussian = std::exp(-(distance * distance) / doubledSquaredBandwidth);
                    newPosition += point * gaussian;
                    totalWeight += gaussian;
                }
            }

            // the new position of the point is the weighted average of its neighbors
            newPosition /= totalWeight;
            builder.shiftPoint(i, newPosition);
        }
        ++iterations;
    }
    if (iterations == MAX_ITERATIONS)
        std::cout << "WARNING: reached the maximum number of iterations" << std::endl;
    return builder.buildClusters();
}

