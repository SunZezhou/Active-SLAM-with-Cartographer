#ifndef MEANSHIFT_CLUSTER_HPP
#define MEANSHIFT_CLUSTER_HPP

#include <vector>

#include "Point.hpp"


class Cluster {
public:
    explicit Cluster(Point centroid);

    Point getCentroid() const;

    void addPoint(Point point, int index);

    long getSize() const;

    std::vector<Point>::iterator begin();

    std::vector<Point>::iterator end();

    float getSse() const;

    std::vector<int> origin_index;

private:
    std::vector<Point> points;
    Point centroid;
};


#endif //MEANSHIFT_CLUSTER_HPP
