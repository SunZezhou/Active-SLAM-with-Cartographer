#ifndef MEANSHIFT_CLUSTERBUILDER_HPP
#define MEANSHIFT_CLUSTERBUILDER_HPP

#include <vector>

#include "Point.hpp"
#include "Cluster.hpp"

class ClustersBuilder {
public:
    explicit ClustersBuilder(const std::vector<Point> &originalPoints, float clusterEps);

    Point &getShiftedPoint(long index);

    void shiftPoint(long index, const Point &newPosition);

    bool hasStoppedShifting(long index);

    bool allPointsHaveStoppedShifting();

    std::vector<Point>::iterator begin();

    std::vector<Point>::iterator end();

    std::vector<Cluster> buildClusters();

private:
    std::vector<Point> originalPoints;
    std::vector<Point> shiftedPoints;
    // vector of booleans such that the element in position i is false if the i-th point
    // has stopped to shift
    std::vector<bool> shifting;
    float clusterEps;
    float shiftingEps;
};


#endif //MEANSHIFT_CLUSTERBUILDER_HPP
